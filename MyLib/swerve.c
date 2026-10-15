/**
 * @file    swerve.c
 * @brief   舵轮模块（转向 + 行走）逆解与闭环算法实现
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

/*====================================================================
 *  模块初始化：默认值 + PID 清零（参数由使用处覆盖）
 *===================================================================*/
void Swerve_ModuleInit(SwerveModule_t *m)
{
    if (m == NULL)
    {
        return;
    }

    /* ---- 转向电机 ---- */
    m->steer.ID         = 0;
    m->steer.hcan       = NULL;
    m->steer.offset     = 0;
    m->steer.actual_pos = 0;
    m->steer.ready      = 0;
    pid2_clear(&m->steer.pos_pid);
    pid2_clear(&m->steer.vel_pid);

    /* ---- 行走电机 ---- */
    m->drive.hcan     = NULL;
    m->drive.motor_id = 0;
    m->drive.epm      = 0;
    pid2_clear(&m->drive_pid);

    /* ---- 安装 / 标定参数默认值 ---- */
    m->x  = 0.0f;
    m->y  = 0.0f;
    m->steer_gear   = 36.0f;    /* M2006 默认 36:1 */
    m->zero_deg     = 0.0f;
    m->dir_steer    = 1.0f;
    m->dir_drive    = 1.0f;
    m->erpm_per_mps = 0.0f;     /* 0 表示行走不输出，标定后再填 */
    m->steer_deadband = 3.0f;

    /* ---- 运行时状态 ---- */
    m->angle_deg        = 0.0f;
    m->target_angle_deg = 0.0f;
    m->target_erpm      = 0.0f;
    m->steer_current    = 0.0f;
    m->drive_current    = 0.0f;
}

/*====================================================================
 *  标定：记录当前轮子朝向为零位
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
 *  单模块计算：逆解 + 归中 + 转向/行走闭环（只算不发）
 *===================================================================*/
float Swerve_ModuleControl(SwerveModule_t *m, float vx, float vy, float wz,
                           float steer_cur_limit, float vesc_cur_limit)
{
    if (m == NULL)
    {
        return 0.0f;
    }

    /* 未收到反馈前保持停止，避免上电乱动 */
    if (!m->steer.ready)
    {
        m->steer_current = 0.0f;
        m->drive_current = 0.0f;
        return 0.0f;
    }

    /* ---- 1. 逆解：底盘速度 -> 本轮平移速度矢量 ---- */
    float vix = vx - wz * m->y;
    float viy = vy + wz * m->x;

    float speed      = sqrtf(vix * vix + viy * viy);        /* 轮速大小 m/s */
    float target_deg = atan2f(viy, vix) * 57.2957795f;      /* 目标朝向 deg */

    /* ---- 2. 当前轮子朝向 ---- */
    m->angle_deg = m->steer.motor.Angle_DEG * m->dir_steer / m->steer_gear - m->zero_deg;

    /* ---- 3. 归中：目标角收敛到当前角 ±180° 内 ---- */
    float err = target_deg - m->angle_deg;
    while (err >  180.0f) { err -= 360.0f; }
    while (err < -180.0f) { err += 360.0f; }

    if (fabsf(err) > 90.0f)     /* 翻转过去更近：目标角反向，轮速取反 */
    {
        err   = (err > 0.0f) ? (err - 180.0f) : (err + 180.0f);
        speed = -speed;
    }
    m->target_angle_deg = m->angle_deg + err;

    /* ---- 4. 目标轮速 -> 目标电转速 ---- */
    m->target_erpm = speed * m->erpm_per_mps * m->dir_drive;

    /* ---- 5. 转向：位置环 -> 速度环 -> 电流 ---- */
    float target_rotor_deg = (m->target_angle_deg + m->zero_deg)
                           * m->steer_gear / m->dir_steer;

    PID_Control2(m->steer.motor.Angle_DEG, target_rotor_deg, &m->steer.pos_pid);
    PID_Control2((float)m->steer.motor.Speed, m->steer.pos_pid.pid_out, &m->steer.vel_pid);

    float cur = m->steer.vel_pid.pid_out;
    if (cur >  steer_cur_limit) { cur =  steer_cur_limit; }
    if (cur < -steer_cur_limit) { cur = -steer_cur_limit; }
    m->steer_current = cur;

    /* ---- 6. 行走：速度环 -> 电流 ---- */
    PID_Control2((float)m->drive.epm, m->target_erpm, &m->drive_pid);

    float d_cur = m->drive_pid.pid_out;
    if (d_cur >  vesc_cur_limit) { d_cur =  vesc_cur_limit; }
    if (d_cur < -vesc_cur_limit) { d_cur = -vesc_cur_limit; }
    m->drive_current = d_cur;

    return cur;
}
