/**
 * @file    swerve.c
 * @brief   舵轮模块（转向 + 行走）逆解、闭环与光电门复位算法实现
 * @note    本文件只有算法：不收发 CAN，结果写入实例结构体，由使用处用
 *          MotorSend() / VESC_SetCurrent() / Motor2006Recv() / VESC_ReceiveHandler() 收发。
 */
#include "swerve.h"

/* PID 清零 */
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
    m->steer_gear   = 36.0f * 4.0f;     /* 2006 减速比 36 × 机构 4 = 144 */
    m->zero_deg     = 0.0f;
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

/*====================================================================
 *  只跑转向闭环：轮子角 -> 转子角 + 位置环 + 速度环（只算不发）
 *===================================================================*/
float Swerve_SteerLoop(SwerveModule_t *m, float steer_cur_limit)
{
    if (m == NULL)
    {
        return 0.0f;
    }

    /* 未收到反馈前不输出 */
    if (!m->steer.ready)
    {
        m->steer_current = 0.0f;
        return 0.0f;
    }

    /* 反馈：当前轮子朝向 */
    m->angle_deg = m->steer.motor.Angle_DEG * m->dir_steer / m->steer_gear - m->zero_deg;

    /* 目标：轮子角 -> 转子角 */
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

    /* ① 首周期：把当前光电门电平记进 bit4，作为"上一次电平" */
    if (!(m->reset_flag & SWERVE_RESET_RUNNING))
    {
        m->reset_flag |= SWERVE_RESET_RUNNING;
        m->reset_flag |= (uint8_t)(HAL_GPIO_ReadPin(m->key_port, m->key_pin) << 4);
    }

    /* ② 慢速扫描：按当前电平挑一个方向，每周期挪 reset_step_deg */
    if (HAL_GPIO_ReadPin(m->key_port, m->key_pin))
    {
        m->target_angle_deg -= m->reset_step_deg;
    }
    else
    {
        m->target_angle_deg += m->reset_step_deg;
    }

    /* ③ 电平跳变（边沿）= 扫过零位标记 -> 记录零位、完成复位 */
    if ((uint8_t)(HAL_GPIO_ReadPin(m->key_port, m->key_pin) << 4)
        != (m->reset_flag & SWERVE_RESET_LAST_LVL))
    {
        m->zero_deg = m->steer.motor.Angle_DEG * m->dir_steer / m->steer_gear - 45.0f;
        m->target_angle_deg = 0.0f;
        m->reset_flag |= SWERVE_RESET_DONE;
    }

    /* ④ 刷新"上一次电平" */
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

/*====================================================================
 *  手动标定零位（无光电门时用）
 *===================================================================*/
void Swerve_SetZero(SwerveModule_t *m)
{
    if (m == NULL)
    {
        return;
    }
    m->zero_deg = m->steer.motor.Angle_DEG * m->dir_steer / m->steer_gear;
}

/*====================================================================
 *  转向是否到位
 *===================================================================*/
uint8_t Swerve_IsAligned(const SwerveModule_t *m)
{
    if (m == NULL)
    {
        return 0u;
    }
    return (fabsf(m->target_angle_deg - m->angle_deg) < m->steer_deadband) ? 1u : 0u;
}

/*====================================================================
 *  单模块完整计算：逆解 + 归中 + 转向/行走闭环（只算不发）
 *===================================================================*/
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
    float target_deg = atan2f(viy, vix) * 57.2957795f;      /* 目标朝向 deg */

    m->angle_deg = m->steer.motor.Angle_DEG * m->dir_steer / m->steer_gear - m->zero_deg;

    float err = target_deg - m->angle_deg;
    while (err >  180.0f) { err -= 360.0f; }
    while (err < -180.0f) { err += 360.0f; }

    if ((fabsf(err) > 100.0f) && (fabsf(speed) > 0.01f))
    {
        err   = (err > 0.0f) ? (err - 180.0f) : (err + 180.0f);
        speed = -speed;
    }
    m->target_angle_deg = m->angle_deg + err;

    m->target_erpm = speed * m->erpm_per_mps * m->dir_drive;

    float cur = Swerve_SteerLoop(m, steer_cur_limit);

    PID_Control2((float)m->drive.epm, m->target_erpm, &m->drive_pid);

    float d_cur = m->drive_pid.pid_out;
    if (d_cur >  vesc_cur_limit) { d_cur =  vesc_cur_limit; }
    if (d_cur < -vesc_cur_limit) { d_cur = -vesc_cur_limit; }
    m->drive_current = d_cur;

    return cur;
}
