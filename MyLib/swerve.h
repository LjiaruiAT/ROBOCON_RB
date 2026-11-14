/**
 * @file    swerve.h
 * @brief   舵轮模块（转向 + 行走）：逆解、闭环与光电门复位算法库
 */
#ifndef __SWERVE_H__
#define __SWERVE_H__

#include "motor.h"
#include "motorEx.h"
#include "VESC.h"
#include "PID_old.h"
#include "can.h"
#include <math.h>

#define SWERVE_COUNT    4u        

#define SWERVE_RESET_RUNNING    0x20u  
#define SWERVE_RESET_LAST_LVL   0x10u   /*!< 上一次的光电门电平（跳变检测用） */
#define SWERVE_RESET_DONE       0x80u   /*!< 复位完成（已找到零位） */
#define SWERVE_RESET_FAIL       0x40u   /*!< 复位超出行程仍未找到零位 */
#define SWERVE_RESET_BACK       0x01u   /*!< 扫描已掉头一次（新逻辑不再使用） */
#define SWERVE_RESET_DIRN       0x02u   /*!< 扫描方向（新逻辑固定顺时针，不再使用） */
#define SWERVE_RESET_ENTER      0x04u   /*!< 已记录"进入遮挡"边沿 */
#define SWERVE_RESET_EXIT       0x08u   /*!< 已记录"离开遮挡"边沿 */
#define SWERVE_RESET_DEBOUNCE   1u      /*!< 电平去抖：连续 N 个周期一致才认一次跳变 */
#define SWERVE_RESET_LIMIT_DEG  90.0f   /*!< 复位扫描行程：单向顺时针 90° */
#define SWERVE_STEER_LIMIT_DEG   120.0f  /*!< 正常行驶时舵向目标绝对上限（硬软限位） */

/**
 * @brief 单个舵轮模块（一个实例 = 一个舵轮）
 */
typedef struct
{
    /* ---------------- 硬件对象（使用者填写/接线） ---------------- */
    Motor2006Ex_t steer;        /*!< 转向电机（默认 M2006） */
    VESC_t        drive;        /*!< 行走电机（默认 VESC） */
    PID2          drive_pid;    /*!< 行走速度环（输入 rpm，输出电流 A） */
    float         drive_ff;     /*!< 行走前馈 A/rpm：输出 += drive_ff × target_rpm（0 = 关闭） */
    float         drive_db;     /*!< 行走误差死区 (rpm)：|误差| 小于此值不积分（0 = 关闭） */

    /* ---------------- 光电门（复位用） ---------------- */
    GPIO_TypeDef *key_port;     /*!< 光电门 GPIO 端口 */
    uint16_t      key_pin;      /*!< 光电门 GPIO 引脚 */
    uint8_t       reset_flag;   /*!< 复位状态位（SWERVE_RESET_xxx） */
    uint8_t       reset_pend;   /*!< 去抖：候选电平（0/1） */
    uint8_t       reset_dbc;    /*!< 去抖：候选电平连续计数 */
    uint8_t       reset_edge_cnt; /*!< 扫描期间电平变化次数（去抖之前计数，诊断用） */
    uint16_t      reset_lvl1_cnt; /*!< 扫描期间电平=1 的周期数（诊断用） */
    float         rst_psi0;     /*!< 复位起始时的轮子角 (deg)，用于判断实际扫过多少 */
    float         rst_th1;      /*!< 进入遮挡时的轮子角 (deg) */
    float         rst_th2;      /*!< 离开遮挡时的轮子角 (deg) */

    /* ---------------- 安装 / 标定参数 ---------------- */
    float x;                    /*!< 安装坐标 x：前方为正 (m) */
    float y;                    /*!< 安装坐标 y：左方为正 (m) */
    float steer_gear;           /*!< 转向总减速比（M2006 36 × 机构 4 = 144） */
    float zero_deg;             /*!< 零位：轮子朝正前方时的轮子角 (deg) */
    float zero_bias_deg;        /*!< 光电门边沿触发时轮子的真实朝向 (deg，左转为正)；
                                     镜像装配的模块与直装侧差 180°（45 / -135） */
    float dir_steer;            /*!< 转向方向系数（±1） */
    float dir_drive;            /*!< 行走方向系数（±1，轮子反装取 -1） */
    float erpm_per_mps;         /*!< 换算系数：电转速 = 轮速(m/s) × 该值 */
    float steer_deadband;       /*!< 转向到位死区 (deg) */
    float reset_step_deg;       /*!< 复位扫描步长 (deg/周期)，示例取 0.3 */

    /* ---------------- 运行时状态（库计算，使用处读取/发送） ---------------- */
    float angle_deg;            /*!< 当前轮子朝向 (deg) */
    float target_angle_deg;     /*!< 逆解目标朝向 / 复位扫描目标 (deg) */
    float target_erpm;          /*!< 逆解目标电转速 */
    float wheel_rpm;
    float wheel_mps;
    float target_rpm;
    float steer_current;        /*!< 本周期转向电流指令 */
    float drive_current;        /*!< 本周期行走电流指令 (A) */
} SwerveModule_t;

/**
 * @brief 模块初始化：参数默认值 + PID 清零（填参前先调用一次）
 */
void Swerve_ModuleInit(SwerveModule_t *m);

void Swerve_ChassisVel(const SwerveModule_t *mods, float *vx, float *vy, float *wz);

uint8_t Swerve_AllReady(const SwerveModule_t *mods);
uint8_t Swerve_ResetStep(SwerveModule_t *mods, int16_t *cur_out);

void Swerve_ControlStep(SwerveModule_t *mods, float vx, float vy, float wz, int16_t *cur_out);

#define SWERVE_ODOM_WINDOW_MS   1000u

extern volatile float g_chas_in_vx;
extern volatile float g_chas_in_vy;
extern volatile float g_chas_in_wz;
extern volatile float g_chas_act_vx;
extern volatile float g_chas_act_vy;
extern volatile float g_chas_act_wz;
extern volatile float g_chas_act_vx_now;
extern volatile float g_chas_act_vy_now;
extern volatile float g_chas_act_wz_now;
extern volatile float g_chas_distance_m;
extern volatile uint32_t g_chas_window_ms;
extern volatile uint32_t g_chas_window_n;

void Swerve_OdomReset(void);

uint8_t Swerve_OdomStep(const SwerveModule_t *mods, float in_vx, float in_vy, float in_wz, float dt_s);

/**
 * @brief 单模块计算（建议 2ms 周期）：逆解 + 归中 + 转向/行走闭环，只算不发
 * @param[in]  vx, vy  底盘平移速度 (m/s，前正 / 左正)
 * @param[in]  wz      底盘自转速度 (rad/s，逆时针正)
 * @param[in]  steer_cur_limit  转向电流限幅
 * @param[in]  vesc_cur_limit   行走电流限幅 (A)
 * @return 本周期转向电流指令（行走电流见 m->drive_current）
 */
float Swerve_ModuleControl(SwerveModule_t *m, float vx, float vy, float wz);

/* ================= 光电门复位（上电找零位） ================= */

/**
 * @brief 复位流程一步（每个控制周期调用一次）
 * @note  核心逻辑取自示例工程 Reset_Function()：
 *          ① 首周期记录光电门初始电平；
 *          ② 按电平方向慢速扫描（改 target_angle_deg，步长 reset_step_deg）；
 *          ③ 检测到电平跳变的瞬间 -> 记录当前姿态为零位、清零目标角、置 DONE 位。
 * @return 1 已复位完成；0 仍在扫描
 */
uint8_t Swerve_Reset(SwerveModule_t *m);

/**
 * @brief 读光电门逻辑电平（已按 key_inv 处理）：1 = 遮挡，0 = 未遮挡
 */
uint8_t Swerve_KeyLevel(const SwerveModule_t *m);

/**
 * @brief 复位是否已完成
 */
uint8_t Swerve_IsResetDone(const SwerveModule_t *m);

/**
 * @brief 只跑转向闭环（复位阶段用）：以 target_angle_deg 为目标，
 *        内部完成"轮子角->转子角"换算 + 位置环 + 速度环，只算不发
 * @return 本周期转向电流指令
 */
float Swerve_SteerLoop(SwerveModule_t *m);

/* ================= 工具 ================= */

/**
 * @brief 手动标定零位：把该轮子摆正朝前（+x）后调用（无光电门时用）
 */
void Swerve_SetZero(SwerveModule_t *m);

/**
 * @brief 转向是否到位（误差 < steer_deadband）
 */
uint8_t Swerve_IsAligned(const SwerveModule_t *m);


#endif /* __SWERVE_H__ */
