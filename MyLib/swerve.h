/**
 * @file    swerve.h
 * @brief   舵轮模块（转向 + 行走）：逆解与闭环算法库（纯算法，不含收发）
 * @note    用法与 motorEx 一致 —— 实例在 MyTask 侧定义，收发直接用底层库：
 *
 *            1) 定义实例并填参数：
 *               SwerveModule_t swerve[4];
 *               Swerve_ModuleInit(&swerve[i]);
 *               swerve[i].steer.ID = 0x201;  swerve[i].steer.hcan = &hcan1;
 *               swerve[i].drive.motor_id = 1; swerve[i].drive.hcan = &hcan2;
 *               swerve[i].x = 0.215f; swerve[i].y = 0.225f; ...
 *
 *            2) CAN 回调里直接调底层库（不经过本库）：
 *               Motor2006Recv(&swerve[i].steer, hcan, ID, Recv);      // 转向
 *               VESC_ReceiveHandler(&swerve[i].drive, hcan, ID, Recv); // 行走
 *
 *            3) 周期任务里逐模块计算（只算不发）：
 *               cur[i] = Swerve_ModuleControl(&swerve[i], vx, vy, wz, 限幅, 限幅);
 *
 *            4) 发送也直接用底层库：
 *               MotorSend(&hcan1, 0x200, cur);                        // 转向广播
 *               VESC_SetCurrent(&swerve[i].drive, swerve[i].drive_current); // 行走
 */
#ifndef __SWERVE_H__
#define __SWERVE_H__

#include "motor.h"
#include "motorEx.h"
#include "VESC.h"
#include "PID_old.h"
#include "can.h"
#include <math.h>

#define SWERVE_COUNT    4u          /*!< 舵轮数量（供使用处定义数组用） */

/**
 * @brief 单个舵轮模块（一个实例 = 一个舵轮）
 */
typedef struct
{
    /* ---------------- 硬件对象（使用者填写/接线） ---------------- */
    Motor2006Ex_t steer;        /*!< 转向电机（默认 M2006） */
    VESC_t        drive;        /*!< 行走电机（默认 VESC） */
    PID2          drive_pid;    /*!< 行走速度环（输入 ERPM，输出电流 A） */

    /* ---------------- 安装 / 标定参数 ---------------- */
    float x;                    /*!< 安装坐标 x：前方为正 (m) */
    float y;                    /*!< 安装坐标 y：左方为正 (m) */
    float steer_gear;           /*!< 转向减速比（M2006 = 36.0f） */
    float zero_deg;             /*!< 零位：轮子朝正前方时的轮子角 (deg) */
    float dir_steer;            /*!< 转向方向系数（±1） */
    float dir_drive;            /*!< 行走方向系数（±1，轮子反装取 -1） */
    float erpm_per_mps;         /*!< 换算系数：电转速 = 轮速(m/s) × 该值 */
    float steer_deadband;       /*!< 转向到位死区 (deg) */

    /* ---------------- 运行时状态（库计算，使用处读取/发送） ---------------- */
    float angle_deg;            /*!< 当前轮子朝向 (deg) */
    float target_angle_deg;     /*!< 逆解目标朝向 (deg) */
    float target_erpm;          /*!< 逆解目标电转速 */
    float steer_current;        /*!< 本周期转向电流指令 */
    float drive_current;        /*!< 本周期行走电流指令 (A) */
} SwerveModule_t;

/**
 * @brief 模块初始化：参数默认值 + PID 清零（填参前先调用一次）
 */
void Swerve_ModuleInit(SwerveModule_t *m);

/**
 * @brief 单模块计算（建议 2ms 周期）：逆解 + 归中 + 转向/行走闭环，只算不发
 * @param[in]  vx, vy  底盘平移速度 (m/s，前正 / 左正)
 * @param[in]  wz      底盘自转速度 (rad/s，逆时针正)
 * @param[in]  steer_cur_limit  转向电流限幅
 * @param[in]  vesc_cur_limit   行走电流限幅 (A)
 * @return 本周期转向电流指令（行走电流见 m->drive_current）
 */
float Swerve_ModuleControl(SwerveModule_t *m, float vx, float vy, float wz,
                           float steer_cur_limit, float vesc_cur_limit);

/**
 * @brief 标定零位：把该轮子摆正朝前（+x）后调用
 */
void Swerve_SetZero(SwerveModule_t *m);

/**
 * @brief 转向是否到位（误差 < steer_deadband）
 */
uint8_t Swerve_IsAligned(const SwerveModule_t *m);

#endif /* __SWERVE_H__ */
