/**
 * @file    swerve.c
 * @brief   舵轮模块（转向 + 行走）逆解、闭环与光电门复位算法实现
 * @note    本文件只有算法：不收发 CAN，结果写入实例结构体，由使用处用
 *          MotorSend() / VESC_SetCurrent() / Motor2006Recv() / VESC_ReceiveHandler() 收发。
 */
#include "swerve.h"
#include "config.h"
#include "FreeRTOS.h"
#include "task.h"

volatile float g_chas_in_vx;
volatile float g_chas_in_vy;
volatile float g_chas_in_wz;
volatile float g_chas_act_vx;
volatile float g_chas_act_vy;
volatile float g_chas_act_wz;
volatile float g_chas_act_vx_now;
volatile float g_chas_act_vy_now;
volatile float g_chas_act_wz_now;
volatile float g_chas_distance_m;
volatile uint32_t g_chas_window_ms;
volatile uint32_t g_chas_window_n;

static float s_sum_in_vx;
static float s_sum_in_vy;
static float s_sum_in_wz;
static float s_sum_act_vx;
static float s_sum_act_vy;
static float s_sum_act_wz;
static uint32_t s_odom_n;
static TickType_t s_odom_t0;
static uint32_t s_vesc_status_seen[SWERVE_COUNT];

static void pid2_clear(PID2 *p)
{
    p->error_now = 0.0f;
    p->error_last = 0.0f;
    p->error_inter = 0.0f;
    p->pid_out = 0.0f;
}

void Swerve_ChassisVel(const SwerveModule_t *mods, float *vx, float *vy, float *wz)
{
    float sum_vx = 0.0f;
    float sum_vy = 0.0f;
    float num_wz = 0.0f;
    float den_wz = 0.0f;
    uint8_t i;

    if ((mods == NULL) || (vx == NULL) || (vy == NULL) || (wz == NULL))
    {
        return;
    }

    for (i = 0u; i < SWERVE_COUNT; i++)
    {
        const SwerveModule_t *m = &mods[i];
        float rad = m->angle_deg * (PI / 180.0f);
        float vix = m->wheel_mps * cosf(rad);
        float viy = m->wheel_mps * sinf(rad);

        sum_vx += vix;
        sum_vy += viy;
        num_wz += (m->x * viy) - (m->y * vix);
        den_wz += (m->x * m->x) + (m->y * m->y);
    }

    *vx = sum_vx / (float)SWERVE_COUNT;
    *vy = sum_vy / (float)SWERVE_COUNT;
    *wz = (den_wz > 1.0e-6f) ? (num_wz / den_wz) : 0.0f;
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
    m->drive_ff = 0.0f;
    m->drive_db = 0.0f;

    m->key_port   = NULL;
    m->key_pin    = 0;
    m->reset_flag = 0;
    m->reset_pend = 0u;
    m->reset_dbc  = 0u;
    m->reset_edge_cnt = 0u;
    m->reset_lvl1_cnt = 0u;
    m->rst_psi0   = 0.0f;
    m->rst_th1    = 0.0f;
    m->rst_th2    = 0.0f;

    m->x  = 0.0f;
    m->y  = 0.0f;
    m->steer_gear   = 36.0f * 4.0f;    
    m->zero_deg     = 0.0f;
    m->zero_bias_deg = 0.0f;          
    m->dir_steer    = 1.0f;
    m->dir_drive    = 1.0f;
    m->erpm_per_mps = 0.0f;             
    m->steer_deadband = 3.0f;
    m->reset_step_deg = 0.03f;          

    m->angle_deg        = 0.0f;
    m->target_angle_deg = 0.0f;
    m->target_erpm      = 0.0f;
    m->wheel_rpm        = 0.0f;
    m->wheel_mps        = 0.0f;
    m->target_rpm       = 0.0f;
    m->steer_current    = 0.0f;
    m->drive_current    = 0.0f;
}

float Swerve_SteerLoop(SwerveModule_t *m)
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

    float err_deg;

    m->angle_deg = m->steer.motor.Angle_DEG * m->dir_steer / m->steer_gear - m->zero_deg;

    if ((m->reset_flag & SWERVE_RESET_DONE) != 0u)
    {
        if (m->target_angle_deg > SWERVE_STEER_LIMIT_DEG)
        {
            m->target_angle_deg = SWERVE_STEER_LIMIT_DEG;
        }
        if (m->target_angle_deg < -SWERVE_STEER_LIMIT_DEG)
        {
            m->target_angle_deg = -SWERVE_STEER_LIMIT_DEG;
        }
    }

    err_deg = m->target_angle_deg - m->angle_deg;
    while (err_deg >  180.0f) { err_deg -= 360.0f; }
    while (err_deg < -180.0f) { err_deg += 360.0f; }

    float target_rotor_deg = m->steer.motor.Angle_DEG
                           + err_deg * m->steer_gear / m->dir_steer;

    PID_ControlDt(m->steer.motor.Angle_DEG, target_rotor_deg, &m->steer.pos_pid, CHASSIS_PID_DT_S);
    PID_ControlDt((float)m->steer.motor.Speed, m->steer.pos_pid.pid_out, &m->steer.vel_pid, CHASSIS_PID_DT_S);

    float cur = m->steer.vel_pid.pid_out;
    m->steer_current = cur;

    return cur;
}

uint8_t Swerve_KeyLevel(const SwerveModule_t *m)
{
    if ((m == NULL) || (m->key_port == NULL))
    {
        return 0u;
    }
    return (HAL_GPIO_ReadPin(m->key_port, m->key_pin) != GPIO_PIN_RESET) ? 1u : 0u;
}

uint8_t Swerve_Reset(SwerveModule_t *m)
{
    uint8_t lvl;
    float   psi;

    if (m == NULL)
    {
        return 0u;
    }

    if (m->reset_flag & SWERVE_RESET_DONE)
    {
        return 1u;
    }

    if (m->reset_flag & SWERVE_RESET_FAIL)
    {
        return 0u;
    }

    if (!(m->reset_flag & SWERVE_RESET_RUNNING))
    {
        m->reset_flag |= SWERVE_RESET_RUNNING;
        m->reset_flag |= (uint8_t)(Swerve_KeyLevel(m) << 4);
        m->reset_flag &= (uint8_t)~(SWERVE_RESET_ENTER | SWERVE_RESET_EXIT);
        m->target_angle_deg = 0.0f;
        m->reset_pend = 0u;
        m->reset_dbc  = 0u;
        m->reset_edge_cnt = 0u;
        m->reset_lvl1_cnt = 0u;
        m->rst_psi0   = m->steer.motor.Angle_DEG * m->dir_steer / m->steer_gear;
        m->rst_th1    = 0.0f;
        m->rst_th2    = 0.0f;
    }

    psi = m->steer.motor.Angle_DEG * m->dir_steer / m->steer_gear;

    if (!(m->reset_flag & SWERVE_RESET_ENTER)
        && ((m->rst_psi0 - psi) < SWERVE_RESET_LIMIT_DEG))
    {
        m->target_angle_deg -= m->reset_step_deg;
        if (m->target_angle_deg < (m->rst_psi0 - SWERVE_RESET_LIMIT_DEG - 5.0f))
        {
            m->target_angle_deg = m->rst_psi0 - SWERVE_RESET_LIMIT_DEG - 5.0f;
        }
    }

    lvl = Swerve_KeyLevel(m);
    if (lvl != 0u)
    {
        m->reset_lvl1_cnt++;
    }

    if (!(m->reset_flag & SWERVE_RESET_ENTER)
        && (lvl != ((m->reset_flag & SWERVE_RESET_LAST_LVL) ? 1u : 0u)))
    {
        m->reset_edge_cnt++;
        m->rst_th1 = psi;
        m->reset_flag |= SWERVE_RESET_ENTER;
    }

    if (m->reset_flag & SWERVE_RESET_ENTER)
    {
        m->zero_deg = m->rst_th1 - m->zero_bias_deg;
        m->target_angle_deg = 0.0f;
        m->reset_flag |= SWERVE_RESET_DONE;
    }
    else if ((m->rst_psi0 - psi) >= SWERVE_RESET_LIMIT_DEG)
    {
        m->reset_flag |= SWERVE_RESET_FAIL;
        m->target_angle_deg = m->angle_deg;
    }

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
    float err;

    if (m == NULL)
    {
        return 0u;
    }

    err = m->target_angle_deg - m->angle_deg;
    while (err >  180.0f) { err -= 360.0f; }
    while (err < -180.0f) { err += 360.0f; }

    return (fabsf(err) < m->steer_deadband) ? 1u : 0u;
}


/* ------------------------------------------------------------------ */
/* 行走速度环：P + I + 前馈，带条件积分抗饱和                          */
/* ------------------------------------------------------------------
 * 不用 PID_Control2 的原因：
 *   1) 最终限幅是 vesc_cur_limit（20 A），而 PID 内部按 output_limit 积分，
 *      两者不一致时会 windup：松油门后要等积分泄完才真正减速，表现为"冲"。
 *   2) 速度环靠纯积分补稳态误差会把积分顶到限幅，加前馈后积分只做微调。
 *   3) dt 必须等于底盘任务周期；旧代码写死 2 ms 而任务是 1 ms，Ki 实际放大 2 倍。
 */
static float swerve_drive_loop(SwerveModule_t *m, float rpm_now)
{
    PID2       *p = &m->drive_pid;
    const float dt = DRIVE_PID_DT_S;
    float err;
    float inter;
    float out;

    if (dt <= 0.0f)
    {
        return 0.0f;
    }

    err = m->target_rpm - rpm_now;

    p->error_last = p->error_now;
    p->error_now  = err;

    inter = p->error_inter;

    /* 死区：误差很小时不积分，避免匀速段来回抖动 */
    if ((m->drive_db <= 0.0f) || (fabsf(err) > m->drive_db))
    {
        inter += err * dt;
    }

    if (inter >  p->limit) { inter =  p->limit; }
    if (inter < -p->limit) { inter = -p->limit; }

    /* 前馈：稳态电流 ≈ 目标转速 × (A/rpm)，标定后填 m->drive_ff */
    out = p->Kp * err + p->Ki * inter + m->drive_ff * m->target_rpm;
    out += p->Kd * (p->error_now - p->error_last) / dt;

    if (out > SWERVE_VESC_CUR_LIMIT)
    {
        out = SWERVE_VESC_CUR_LIMIT;
        if (err < 0.0f) { p->error_inter = inter; }   /* 饱和时只允许往回积分 */
    }
    else if (out < -SWERVE_VESC_CUR_LIMIT)
    {
        out = -SWERVE_VESC_CUR_LIMIT;
        if (err > 0.0f) { p->error_inter = inter; }
    }
    else
    {
        p->error_inter = inter;
    }

    p->pid_out = out;
    return out;
}

float Swerve_ModuleControl(SwerveModule_t *m, float vx, float vy, float wz)
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
        float base_deg = atan2f(viy, vix) * 57.2957795f;  /* 目标朝向 deg */
        float cand_deg;
        float alt_deg;
        float err;
        float err_alt;

        cand_deg = base_deg;
        if (cand_deg > SWERVE_STEER_LIMIT_DEG)
        {
            cand_deg -= 180.0f;
            speed = -speed;
        }
        else if (cand_deg < -SWERVE_STEER_LIMIT_DEG)
        {
            cand_deg += 180.0f;
            speed = -speed;
        }

        alt_deg = cand_deg + 180.0f;
        while (alt_deg >  180.0f) { alt_deg -= 360.0f; }
        while (alt_deg < -180.0f) { alt_deg += 360.0f; }

        err     = cand_deg - m->angle_deg;
        err_alt = alt_deg  - m->angle_deg;
        while (err     >  180.0f) { err     -= 360.0f; }
        while (err     < -180.0f) { err     += 360.0f; }
        while (err_alt >  180.0f) { err_alt -= 360.0f; }
        while (err_alt < -180.0f) { err_alt += 360.0f; }

        if ((fabsf(alt_deg) <= SWERVE_STEER_LIMIT_DEG) &&
            ((fabsf(err_alt) + 10.0f) < fabsf(err)))
        {
            m->target_angle_deg = alt_deg;
            err   = err_alt;
            speed = -speed;
        }
        else
        {
            m->target_angle_deg = cand_deg;
        }

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

    m->wheel_rpm  = (float)m->drive.epm / (float)VESC_POLE_PAIRS;
    m->wheel_mps  = m->wheel_rpm * (PI * DRIVE_WHEEL_DIA / 60.0f) * m->dir_drive;
    m->target_rpm = speed * (60.0f / (PI * DRIVE_WHEEL_DIA)) * m->dir_drive;

    float cur = Swerve_SteerLoop(m);

    m->drive_current = swerve_drive_loop(m, m->wheel_rpm);

    return cur;
}

uint8_t Swerve_AllReady(const SwerveModule_t *mods)
{
    uint8_t i;

    if (mods == NULL)
    {
        return 0u;
    }
    for (i = 0u; i < SWERVE_COUNT; i++)
    {
        if (mods[i].steer.ready == 0u)
        {
            return 0u;
        }
    }
    return 1u;
}

uint8_t Swerve_ResetStep(SwerveModule_t *mods, int16_t *cur_out)
{
    uint8_t all_done = 1u;
    uint8_t i;

    if ((mods == NULL) || (cur_out == NULL))
    {
        return 1u;
    }
    for (i = 0u; i < SWERVE_COUNT; i++)
    {
        if (!Swerve_IsResetDone(&mods[i]))
        {
            Swerve_Reset(&mods[i]);
            all_done = 0u;
        }
        cur_out[i] = (int16_t)Swerve_SteerLoop(&mods[i]);
    }
    return all_done;
}

void Swerve_ControlStep(SwerveModule_t *mods, float vx, float vy, float wz, int16_t *cur_out)
{
    uint8_t i;

    if ((mods == NULL) || (cur_out == NULL))
    {
        return;
    }
    for (i = 0u; i < SWERVE_COUNT; i++)
    {
        cur_out[i] = (int16_t)Swerve_ModuleControl(&mods[i], vx, vy, wz);
    }
}

void Swerve_OdomReset(void)
{
    s_sum_in_vx  = 0.0f;
    s_sum_in_vy  = 0.0f;
    s_sum_in_wz  = 0.0f;
    s_sum_act_vx = 0.0f;
    s_sum_act_vy = 0.0f;
    s_sum_act_wz = 0.0f;
    s_odom_n     = 0u;
    s_odom_t0    = xTaskGetTickCount();
    g_chas_distance_m = 0.0f;
}

uint8_t Swerve_OdomStep(const SwerveModule_t *mods, float in_vx, float in_vy, float in_wz, float dt_s)
{
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
    uint8_t all_new = 1u;
    uint8_t i;
    uint32_t elapsed;

    Swerve_ChassisVel(mods, &vx, &vy, &wz);

    g_chas_act_vx_now = vx;
    g_chas_act_vy_now = vy;
    g_chas_act_wz_now = wz;

    for (i = 0u; i < SWERVE_COUNT; i++)
    {
        if (mods[i].drive.status_cnt == s_vesc_status_seen[i])
        {
            all_new = 0u;
            break;
        }
    }
    if (all_new != 0u)
    {
        for (i = 0u; i < SWERVE_COUNT; i++)
        {
            s_vesc_status_seen[i] = mods[i].drive.status_cnt;
        }
    }

    g_chas_distance_m += sqrtf(vx * vx + vy * vy) * dt_s;

    s_sum_in_vx  += in_vx;
    s_sum_in_vy  += in_vy;
    s_sum_in_wz  += in_wz;
    s_sum_act_vx += vx;
    s_sum_act_vy += vy;
    s_sum_act_wz += wz;
    s_odom_n++;

    elapsed = (uint32_t)(xTaskGetTickCount() - s_odom_t0);
    if (elapsed >= SWERVE_ODOM_WINDOW_MS)
    {
        if (s_odom_n > 0u)
        {
            const float inv = 1.0f / (float)s_odom_n;

            g_chas_in_vx  = s_sum_in_vx  * inv;
            g_chas_in_vy  = s_sum_in_vy  * inv;
            g_chas_in_wz  = s_sum_in_wz  * inv;
            g_chas_act_vx = s_sum_act_vx * inv;
            g_chas_act_vy = s_sum_act_vy * inv;
            g_chas_act_wz = s_sum_act_wz * inv;
        }
        g_chas_window_ms = elapsed;
        g_chas_window_n  = s_odom_n;

        s_sum_in_vx  = 0.0f;
        s_sum_in_vy  = 0.0f;
        s_sum_in_wz  = 0.0f;
        s_sum_act_vx = 0.0f;
        s_sum_act_vy = 0.0f;
        s_sum_act_wz = 0.0f;
        s_odom_n     = 0u;
        s_odom_t0    = xTaskGetTickCount();
    }

    return all_new;
}
