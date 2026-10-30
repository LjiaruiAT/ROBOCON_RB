/**
 * @file    imu_probe.h
 * @brief   IMU 串口探测：同时监听 4 个候选串口，定位 IMU 到底接在哪个口
 * @note    已配置：UART4(PC10/11)、USART6(PC6/7)、USART2(PD5/6)
 *          新手工初始化：USART3(PD8/PD9)——参考工程有、本工程原本没有
 *          定位完成后改 JY61.h 的 IMU_HUART 即可，本文件可以停用
 */
#ifndef __IMU_PROBE_H__
#define __IMU_PROBE_H__

#include "main.h"
#include "usart.h"

/* 每个口的接收缓冲 */
extern uint8_t g_probe_buf4[64];   /* UART4  PC11 */
extern uint8_t g_probe_buf6[64];   /* USART6 PC7  */
extern uint8_t g_probe_buf2[64];   /* USART2 PD6  */
extern uint8_t g_probe_buf3[64];   /* USART3 PD9  */

/* 每个口的回调次数 / 累计字节数 */
extern volatile uint32_t g_probe_cb4, g_probe_cb6, g_probe_cb2, g_probe_cb3;
extern volatile uint32_t g_probe_n4,  g_probe_n6,  g_probe_n2,  g_probe_n3;
extern volatile uint32_t g_probe_e4,  g_probe_e6,  g_probe_e2,  g_probe_e3;

/* 4 个口的接收总次数：随便看哪个在涨就知道 IMU 在哪 */
extern volatile uint32_t g_probe_total;

void    ImuProbe_Init(void);

/* 在 HAL_UARTEx_RxEventCallback 里调用；返回 1 表示该口已处理并重启接收 */
uint8_t ImuProbe_OnRx(UART_HandleTypeDef *huart, uint16_t size);

/* 在 HAL_UART_ErrorCallback 里调用；返回 1 表示该口已处理并重启接收 */
uint8_t ImuProbe_OnErr(UART_HandleTypeDef *huart);

#endif /* __IMU_PROBE_H__ */
