/**
 * @file    swerve.c
 * @brief   舵轮模块（转向 + 行走）逆解、闭环与光电门复位算法实现
 * @note    本文件只有算法：不收发 CAN，结果写入实例结构体，由使用处用
 *          MotorSend() / VESC_SetCurrent() / Motor2006Recv() / VESC_ReceiveHandler() 收发。
 */
#include "swerve.h"

static void pid2_clear(PID2 *p)
{
    p->error_now = 0.0f;
    p->error_last = 0.0f;
    p->error_inter = 0.0f;
    p->pid_out = 0.0f;
}

void Swerve_ModuleInit(SwerveModule_t *m)
{
    if (m == NULL)
    {
        return;
    }

    m->steer.ID         = 0;
    m->steer.hcan       = NULL;
    m->steer.offset     = 0;
    m->steer.actual_pos = 0;
    m->steer.ready      = 0;
    pid2_clear(&m->steer.pos_pid);
    pid2_clear(&m->steer.vel_pid);

    m->drive.hcan     = NULL;
    m->drive.motor_id = 0;
    m->drive.epm      = 0;
    pid2_clear(&m->drive_pid);

    m->key_port   = NULL;
    m->key_pin    = 0;
    m->reset_flag = 0;

    m->x  = 0.0f;
    m->y  = 0.0f;
    m->steer_gear   = 36.0f * 4.0f;    
    m->zero_deg     = 0.0f;
    m->zero_bias_deg = 45.0f;           /* 默认：边沿在轮子朝前偏 +45° 处 */
    m->dir_steer    = 1.0f;
    m->dir_drive    = 1.0f;
    m->erpm_per_mps = 0.0f;             
    m->steer_deadband = 3.0f;
    m->reset_step_deg = 0.3f;          

    m->angle_deg        = 0.0f;
    m->target_angle_deg = 0.0f;
    m->target_erpm      = 0.0f;
    m->steer_current    = 0.0f;
    m->drive_current    = 0.0f;
}

float Swerve_SteerLoop(SwerveModule_t *m, float steer_cur_limit)
{
    if (m == NULL)
    {
        return 0.0f;
    }

    if (!m->steer.ready)
    {
        m->steer_current = 0.0f;
        return 0.0f;
    }

    m->angle_deg = m->steer.motor.Angle_DEG * m->dir_steer / m->steer_gear - m->zero_deg;

    float target_rotor_deg = (m->target_angle_deg + m->zero_deg)
                           * m->steer_gear / m->dir_steer;

    PID_Control2(m->steer.motor.Angle_DEG, target_rotor_deg, &m->steer.pos_pid);
    PID_Control2((float)m->steer.motor.Speed, m->steer.pos_pid.pid_out, &m->steer.vel_pid);

    float cur = m->steer.vel_pid.pid_out;
    if (cur >  steer_cur_limit) { cur =  steer_cur_limit; }
    if (cur < -steer_cur_limit) { cur = -steer_cur_limit; }
    m->steer_current = cur;

    return cur;
}

uint8_t Swerve_Reset(SwerveModule_t *m)
{
    if (m == NULL)
    {
        return 0u;
    }

    if (!(m->reset_flag & SWERVE_RESET_RUNNING))
    {
        m->reset_flag |= SWERVE_RESET_RUNNING;
        m->reset_flag |= (uint8_t)(HAL_GPIO_ReadPin(m->key_port, m->key_pin) << 4);
    }

    if (HAL_GPIO_ReadPin(m->key_port, m->key_pin))
    {
        m->target_angle_deg -= m->reset_step_deg;
    }
    else
    {
        m->target_angle_deg += m->reset_step_deg;
    }

    if ((uint8_t)(HAL_GPIO_ReadPin(m->key_port, m->key_pin) << 4)
        != (m->reset_flag & SWERVE_RESET_LAST_LVL))
    {
        /* 边沿这一瞬的编码器值 → 该轮真实朝向 = zero_bias_deg（每模块单独配） */
        m->zero_deg = m->steer.motor.Angle_DEG * m->dir_steer / m->steer_gear
                    - m->zero_bias_deg;
        m->target_angle_deg = 0.0f;
        m->reset_flag |= SWERVE_RESET_DONE;
    }

    m->reset_flag = (uint8_t)((m->reset_flag & ~SWERVE_RESET_LAST_LVL)
                              | (uint8_t)(HAL_GPIO_ReadPin(m->key_port, m->key_pin) << 4));

    return (m->reset_flag & SWERVE_RESET_DONE) ? 1u : 0u;
}

uint8_t Swerve_IsResetDone(const SwerveModule_t *m)
{
    if (m == NULL)
    {
        return 0u;
    }
    return (m->reset_flag & SWERVE_RESET_DONE) ? 1u : 0u;
}

void Swerve_SetZero(SwerveModule_t *m)
{
    if (m == NULL)
    {
        return;
    }
    m->zero_deg = m->steer.motor.Angle_DEG * m->dir_steer / m->steer_gear;
}

uint8_t Swerve_IsAligned(const SwerveModule_t *m)
{
    if (m == NULL)
    {
        return 0u;
    }
    return (fabsf(m->target_angle_deg - m->angle_deg) < m->steer_deadband) ? 1u : 0u;
}


float Swerve_ModuleControl(SwerveModule_t *m, float vx, float vy, float wz,
                           float steer_cur_limit, float vesc_cur_limit)
{
    if (m == NULL)
    {
        return 0.0f;
    }

    if (!m->steer.ready)
    {
        m->steer_current = 0.0f;
        m->drive_current = 0.0f;
        return 0.0f;
    }

    float vix = vx - wz * m->y;
    float viy = vy + wz * m->x;

    float speed      = sqrtf(vix * vix + viy * viy);        /* 轮速大小 m/s */

    m->angle_deg = m->steer.motor.Angle_DEG * m->dir_steer / m->steer_gear - m->zero_deg;

    if (speed > 0.01f)
    {
        float target_deg = atan2f(viy, vix) * 57.2957795f;  /* 目标朝向 deg */

        float err = target_deg - m->angle_deg;
        while (err >  180.0f) { err -= 360.0f; }
        while (err < -180.0f) { err += 360.0f; }

        if (fabsf(err) > 90.0f)          /* 最优角：最多转 90°（标准做法，原来 150° 要多转 60°） */
        {
            err   = (err > 0.0f) ? (err - 180.0f) : (err + 180.0f);
            speed = -speed;
        }
        m->target_angle_deg = m->angle_deg + err;

        /* 余弦补偿：轮子还没对准时按 cos^3 衰减轮速。
           偏 0° 全速、偏 45° 剩 35%、偏 90° 归零 —— 天然实现"先转完向再行走"，
           横向分力 ∝ sin(θ)·cos^3(θ)，比硬切断驱动平滑，不会边转边偏。 */
        {
            float a  = (err > 0.0f) ? err : -err;
            float cs;

            if (a > 90.0f) { a = 90.0f; }
            cs = cosf(a * 0.017453293f);
            if (cs < 0.0f) { cs = 0.0f; }
            speed *= cs * cs * cs;
        }
    }
    else
    {
        /* 零速度：保持上一周期的目标角。
           不能拿 atan2f(0,0)=0 当目标，否则被翻转支放到 ±180° 的轮子
           会被 180° 硬掰回正前方（而且此时翻转支被 speed>0.01 关掉了）。 */
        speed = 0.0f;
    }

    m->target_erpm = speed * m->erpm_per_mps * m->dir_drive;

    float cur = Swerve_SteerLoop(m, steer_cur_limit);

    PID_Control2((float)m->drive.epm, m->target_erpm, &m->drive_pid);

    float d_cur = m->drive_pid.pid_out;
    if (d_cur >  vesc_cur_limit) { d_cur =  vesc_cur_limit; }
    if (d_cur < -vesc_cur_limit) { d_cur = -vesc_cur_limit; }
    m->drive_current = d_cur;

    return cur;
}
