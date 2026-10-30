/**
 * @file    yaw_hold.h
 * @brief   底盘角速度内环 + 四轮残差自动标定
 *          （文件原名 yaw_hold，原为航向锁存环；已改为与上位机兼容的内环实现）
 *
 *  分层约定：
 *    上位机（雷达/LIO）：位姿、航向角闭环  → 下发 vx / vy / wz_cmd
 *    本模块（底盘内环）：让实测角速度跟随 wz_cmd
 *
 *  本模块**不修改 vx/vy 的方向**，也不做"自己走直"的航向锁存，
 *  避免和上位机的航向环重复闭环。
 *
 *  为什么能消除直行跑偏：
 *    wz_cmd = 0 时，车若因轮速不等而缓慢偏转，实测 ω ≠ 0，
 *    角速度环立刻输出反向 wz 把它压到 0。不论误差来源是轮径差、
 *    VESC 反馈标定、轻微打滑还是重心偏，都能压住。
 *
 *  第二层（默认关闭）：残差自动标定
 *    用 ω_IMU − ω_轮速反算 的残差做慢速积分，自动修正四轮 erpm_per_mps，
 *    让开环就已经接近直行。等内环验证后再开。
 */
#ifndef __YAW_HOLD_H__
#define __YAW_HOLD_H__

#include "main.h"
#include "JY61.h"

/* ======================================================================
 * 总开关：0 = Update() 恒返回 0，完全退回现状
 * ====================================================================== */
#define WZLOOP_EN                1

/* 陀螺符号：手推车头逆时针（俯视），AngularVelocity.Z 变正填 +1，否则 -1 */
#define WZLOOP_GYRO_SIGN         (+1.0f)

/* ======================================================================
 * 角速度环参数
 *   corr = Kp·(wz_cmd − ω_meas) + ∫Ki·(wz_cmd − ω_meas)·dt
 * ====================================================================== */
#define WZLOOP_KP                0.7f      /* 误差 0.1 rad/s → 立刻补 0.07 rad/s */
#define WZLOOP_KI                5.0f      /* 1/s²，约 0.2~0.5 s 消掉稳态偏差 */
#define WZLOOP_CORR_MAX          0.50f     /* 修正量限幅 rad/s */
#define WZLOOP_I_MAX             0.30f     /* 积分项限幅 rad/s */
#define WZLOOP_IMU_TIMEOUT_MS    100u      /* IMU 判活超时 */
#define WZLOOP_IMU_WARMUP        3u        /* 连续收到几帧才允许工作 */

/* ======================================================================
 * 残差自动标定（第二层，默认关）
 * ====================================================================== */
#define WZLOOP_TRIM_EN           0         /* 1 = 开启自动标定 */
#define WZLOOP_TRIM_MU           0.8f      /* 学习速率，越大收敛越快也越容易抖 */
#define WZLOOP_TRIM_MAX          0.06f     /* 单侧最大修正 ±6% */
#define WZLOOP_TRIM_VMIN         0.30f     /* 车速低于此值不学习 m/s */
#define WZLOOP_TRIM_RESID_MAX    0.15f     /* 残差超过此值判定为打滑，不学习 rad/s */

/* ======================================================================
 * 接口
 * ====================================================================== */

/**
 * @brief 初始化（上电调用一次；未调用时 Update() 会自动补一次）
 */
void WzLoop_Init(void);

/**
 * @brief 运行时复位（切换控制源、急停恢复时调用）
 */
void WzLoop_Reset(void);

/**
 * @brief 周期调用（放在 Chassis_Task 的 2 ms 循环里）
 * @param[in] wz_cmd   下发的自转角速度 (rad/s，逆时针正)
 * @param[in] vmag     车体线速度大小 (m/s)，仅自动标定用，可传 0
 * @param[in] w_model  轮速反算的车体角速度 (rad/s)，即 g_chas_act_wz_now；仅自动标定用
 * @param[in] dt       周期 (s)，例如 0.002f
 * @return 需要叠加到 wz 上的修正量 (rad/s)
 */
float WzLoop_Update(float wz_cmd, float vmag, float w_model, float dt);

/**
 * @brief 自动标定得到的差分系数（第二层），未开启时恒返回 0
 *        调用方按轮子 y 的符号应用：
 *            float s = 1.0f + ((m->y > 0.0f) ? trim : -trim);
 *            m->erpm_per_mps = DRIVE_ERPM_PER_MPS * s;
 */
float WzLoop_GetTrimDiff(void);

/**
 * @brief 转向未到位硬门控（可选）
 *        任意一轮转向误差超过 steer_deadband 时，把四轮驱动电流清零。
 *        是否调用由 config.h 的 CHASSIS_USE_TURN_GATE 决定，默认不启用
 *        —— swerve.c 里的三阶余弦补偿已经连续覆盖了这个场景。
 */
void is_turn_down(void);

/* ======================================================================
 * 调试量（Watch 里直接看）
 * ====================================================================== */
extern volatile float    g_wz_corr;       /*!< 本周期输出修正量 rad/s */
extern volatile float    g_wz_err;        /*!< 角速度误差 rad/s（指令 - 实测） */
extern volatile float    g_wz_meas;       /*!< 实测角速度 rad/s */
extern volatile float    g_wz_resid;      /*!< 残差 rad/s（IMU - 轮速反算） */
extern volatile float    g_wz_trim;       /*!< 自动标定当前差分系数 */
extern volatile uint8_t  g_wz_ok;         /*!< 1 = IMU 正常，内环在工作 */

#endif /* __YAW_HOLD_H__ */
