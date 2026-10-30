/**
 * @file    JY61.c
 * @brief   WT901 / JY61 接收解码（只做输入，不做任何控制）
 * @note    在 UART 空闲中断回调里调用 JY61_Receive(&JY61, buf, size)
 */
#include "JY61.h"

JY61_Typedef      JY61;
volatile uint32_t g_gyro_rx_cnt = 0u;
float             Yaw_offset    = 0.0f;

/* 前 10 字节校验和 */
static uint8_t sum10(const uint8_t *data)
{
    uint8_t sum = 0u;
    uint8_t i;
    for (i = 0u; i < 10u; i++)
    {
        sum = (uint8_t)(sum + data[i]);
    }
    return sum;
}

void JY61_Receive(JY61_Typedef *Gyro, uint8_t *data, uint8_t len)
{
    uint8_t i = 0u;

    if ((Gyro == NULL) || (data == NULL))
    {
        return;
    }

    /* 剩余不足 11 字节直接退出，避免越界读 */
    while ((uint16_t)i + 11u <= (uint16_t)len)
    {
        JY61Frame_t *pack;

        if (data[i] != 0x55u)          /* 找帧头 */
        {
            i++;
            continue;
        }

        pack = (JY61Frame_t *)(data + i);

        if (pack->sum != sum10(data + i))   /* 校验失败：只回退 1 字节重新找帧头 */
        {
            i++;
            continue;
        }

        switch (pack->ID)
        {
        case 0x51:                                  /* 加速度，±16 g */
            Gyro->Acceleration.X = pack->X / 32768.0f * 16.0f;
            Gyro->Acceleration.Y = pack->Y / 32768.0f * 16.0f;
            Gyro->Acceleration.Z = pack->Z / 32768.0f * 16.0f;
            break;

        case 0x52:                                  /* 角速度，±2000 deg/s */
            Gyro->AngularVelocity.X = pack->X / 32768.0f * 2000.0f;
            Gyro->AngularVelocity.Y = pack->Y / 32768.0f * 2000.0f;
            Gyro->AngularVelocity.Z = pack->Z / 32768.0f * 2000.0f;
            g_gyro_rx_cnt++;
            break;

        case 0x53:                                  /* 角度，±180 deg */
            Gyro->Angle.lastYaw = Gyro->Angle.Yaw;
            Gyro->Angle.Roll    = pack->X / 32768.0f * 180.0f;
            Gyro->Angle.Pitch   = pack->Y / 32768.0f * 180.0f;
            Gyro->Angle.Yaw     = pack->Z / 32768.0f * 180.0f;
            Gyro->Temp          = pack->Temp / 340.0f + 36.53f;

            /* Yaw 在 ±180 处跳变，用圈数接成连续值 */
            {
                float diff = Gyro->Angle.Yaw - Gyro->Angle.lastYaw;
                if (diff >  180.0f) { Gyro->Angle.rand--; }
                if (diff < -180.0f) { Gyro->Angle.rand++; }
                Gyro->Angle.Multiturn = Gyro->Angle.Yaw
                                      + (float)Gyro->Angle.rand * 360.0f
                                      - Yaw_offset;
            }
            break;

        default:
            break;
        }

        i = (uint8_t)(i + 11u);       /* 跳过整帧 */
    }
}
