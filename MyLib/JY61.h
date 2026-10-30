/**
 * @file    JY61.h
 * @brief   WT901 / JY61 姿态传感器驱动（只做接收解码，不含控制）
 * @note    WitMotion 串口协议，数据帧 11 字节：
 *            0x55 | ID | D0L D0H | D1L D1H | D2L D2H | D3L D3H | SUM
 *            SUM = 前 10 字节相加取低 8 位
 *          ID = 0x51 加速度 / 0x52 角速度 / 0x53 角度 / 0x54 磁场
 */
#ifndef __JY61_H__
#define __JY61_H__

#include "main.h"

#pragma pack(1)
typedef struct {
    uint8_t head;
    uint8_t ID;
    int16_t X, Y, Z, Temp;
    uint8_t sum;
} JY61Frame_t;                 /* 必须正好 11 字节 */
#pragma pack()

typedef struct { float X, Y, Z; } JY61Vec3f_t;

typedef struct {
    JY61Vec3f_t Acceleration;      /*!< 0x51，单位 g */
    JY61Vec3f_t AngularVelocity;   /*!< 0x52，单位 deg/s */
    struct {
        float Yaw, Pitch, Roll;    /*!< 0x53，单位 deg，范围 ±180 */
        float lastYaw;             /*!< 上一帧 Yaw，用于圈数判断 */
        int32_t rand;              /*!< 圈数 */
        float Multiturn;           /*!< 连续航向角 deg（接上圈数，无 ±180 跳变） */
    } Angle;
    float Temp;                    /*!< deg C */
} JY61_Typedef;

extern JY61_Typedef      JY61;
extern volatile uint32_t g_gyro_rx_cnt;   /*!< 合法 0x52 帧计数（判活/调试用） */
extern float             Yaw_offset;      /*!< 航向零位偏置，可从外部设置 */

/**
 * @brief 从一段连续字节流中解析出所有合法帧
 * @param[in] Gyro 目标结构体
 * @param[in] data 数据缓冲区
 * @param[in] len  实际收到的字节数（不是缓冲区大小）
 */
void JY61_Receive(JY61_Typedef *Gyro, uint8_t *data, uint8_t len);

#endif /* __JY61_H__ */
