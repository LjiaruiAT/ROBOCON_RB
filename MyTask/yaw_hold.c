/**
 * @file    yaw_hold.c
 * @brief   底盘角速度内环 + 四轮残差自动标定实现
 */
#include "yaw_hold.h"
#include "app.h"        /* swerve[]、is_turn_flag、Swerve_IsAligned */
#include <math.h>

#define WZ_DEG2RAD      0.017453293f

/* ---------------- 调试量 ---------------- */
volatile float    g_wz_corr  = 0.0f;
volatile float    g_wz_err   = 0.0f;
volatile float    g_wz_meas  = 0.0f;
volatile float    g_wz_resid = 0.0f;
volatile float    g_wz_trim  = 0.0f;
volatile uint8_t  g_wz_ok    = 0u;

/* ---------------- 内部状态 ---------------- */
static uint8_t  s_inited = 0u;
static uint8_t  s_warm   = 0u;      /* 已连续收到的 IMU 帧数 */
static uint32_t s_cnt    = 0u;      /* 上次的 g_gyro_rx_cnt */
static uint32_t s_tick   = 0u;      /* 上次收到数据的时刻 */
static float    s_i      = 0.0f;    /* 积分项输出 rad/s */
static float    s_trim   = 0.0f;    /* 自动标定差分系数 */

static float wz_clamp(float v, float lim)
{
    if (v >  lim) { return  lim; }
    if (v < -lim) { return -lim; }
    return v;
}

void WzLoop_Init(void)
{
    s_inited      = 1u;
    s_warm        = 0u;
    s_cnt         = g_gyro_rx_cnt;
    s_tick        = HAL_GetTick();
    s_i           = 0.0f;
    s_trim        = 0.0f;
    g_wz_corr     = 0.0f;
    g_wz_err      = 0.0f;
    g_wz_resid    = 0.0f;
    g_wz_trim     = 0.0f;
    g_wz_ok       = 0u;
}

void WzLoop_Reset(void)
{
    WzLoop_Init();
}

float WzLoop_GetTrimDiff(void)
{
    return s_trim;
}

float WzLoop_Update(float wz_cmd, float vmag, float w_model, float dt)
{
#if (WZLOOP_EN == 0)
    (void)wz_cmd;
    (void)vmag;
    (void)w_model;
    (void)dt;
    return 0.0f;
#else
    uint32_t now;
    float    w_meas;
    float    e;
    float    corr;

    if (s_inited == 0u)
    {
        WzLoop_Init();
    }

    if (dt <= 0.0f)
    {
        dt = 0.002f;
    }

    /* ============ 1. IMU 判活 ============ */
    now = HAL_GetTick();

    if (g_gyro_rx_cnt != s_cnt)
    {
        s_cnt  = g_gyro_rx_cnt;
        s_tick = now;
        if (s_warm < WZLOOP_IMU_WARMUP)
        {
            s_warm++;
        }
    }

    if (((uint32_t)(now - s_tick) > WZLOOP_IMU_TIMEOUT_MS) ||
        (s_warm < WZLOOP_IMU_WARMUP))
    {
        s_i       = 0.0f;
        g_wz_corr = 0.0f;
        g_wz_ok   = 0u;
        return 0.0f;                 /* IMU 不可用时不干预，退回纯开环 */
    }


/* ============ 2. 角速度环 ============ */
w_meas = WZLOOP_GYRO_SIGN * JY61.AngularVelocity.Z * WZ_DEG2RAD;
e      = wz_cmd - w_meas;

/* 1. 计算比例项 */
float p = WZLOOP_KP * e;

/* 2. 预测下一次积分值 */
float i_next = s_i + WZLOOP_KI * e * dt;

/* 3. 积分限幅 */
i_next = wz_clamp(i_next, WZLOOP_I_MAX);

/* 4. 预测 PI 输出 */
float corr_test = p + i_next;

/* 5. 条件积分：防止积分饱和 */
if ((fabsf(corr_test) < WZLOOP_CORR_MAX) ||
    (corr_test >= WZLOOP_CORR_MAX && e < 0.0f) ||
    (corr_test <= -WZLOOP_CORR_MAX && e > 0.0f))
{
    s_i = i_next;
}

/* 6. 最终输出限幅 */
corr = wz_clamp(p + s_i, WZLOOP_CORR_MAX);

g_wz_corr = corr;
g_wz_err  = e;
g_wz_meas = w_meas;
g_wz_ok   = 1u;

    /* ============ 3. 残差自动标定（第二层） ============ */
#if (WZLOOP_TRIM_EN != 0)
    /* 只在直行、车速够、无打滑时学习 */
    if ((fabsf(wz_cmd) < 0.05f) &&
        (vmag > WZLOOP_TRIM_VMIN) &&
        (fabsf(w_model) < 1.0f))
    {
        float resid = w_meas - w_model;

        g_wz_resid = resid;

        if (fabsf(resid) < WZLOOP_TRIM_RESID_MAX)
        {
            s_trim = wz_clamp(s_trim + WZLOOP_TRIM_MU * resid * dt,
                              WZLOOP_TRIM_MAX);
        }
    }

    g_wz_trim = s_trim;
#else
    (void)vmag;
    (void)w_model;
#endif

    return corr;
#endif /* WZLOOP_EN */
}

/* ======================================================================
 * 转向未到位硬门控（默认不启用，见 CHASSIS_USE_TURN_GATE）
 *   任意一轮转向误差超限 → 四轮驱动电流全清
 * ====================================================================== */
void is_turn_down(void)
{
    uint8_t aligned = 1u;
    uint8_t i;

    for (i = 0u; i < SWERVE_COUNT; i++)
    {
        if (Swerve_IsAligned(&swerve[i]) == 0)
        {
            aligned = 0u;
            break;
        }
    }

    is_turn_flag = (aligned == 0u) ? 1 : 0;

    if (is_turn_flag != 0)
    {
        for (i = 0u; i < SWERVE_COUNT; i++)
        {
            swerve[i].drive_current = 0.0f;
        }
    }
}
