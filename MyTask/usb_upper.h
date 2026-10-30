/**
 * @file    usb_upper.h
 * @brief   上位机（ROS 2）USB CDC 协议接入
 * @note    协议见 upper/roboconr2-no-sim/Robocon27_R2-no-sim/docs/protocol.md v1.5
 *          帧格式：AA | 55(下行)/56(上行) | cmd | len | data[len] | crc_lo | crc_hi
 *          CRC-16/Modbus，范围 = cmd + len + data，低字节在前
 */
#ifndef __USB_UPPER_H__
#define __USB_UPPER_H__

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* 协议常量                                                            */
/* ------------------------------------------------------------------ */
#define HOST_FRAME_HEAD0        0xAAu
#define HOST_FRAME_DIR_DOWN     0x55u   /* 上位机 -> 下位机 */
#define HOST_FRAME_DIR_UP       0x56u   /* 下位机 -> 上位机 */
#define HOST_FRAME_OVERHEAD     6u      /* 帧头2 + cmd1 + len1 + crc2 */
#define HOST_DATA_MAX           64u
#define HOST_FRAME_MAX          (HOST_FRAME_OVERHEAD + HOST_DATA_MAX)

#define HOST_CMD_VELOCITY       0x01u
#define HOST_CMD_ESTOP          0x02u
#define HOST_CMD_MECH           0x03u
#define HOST_CMD_HEARTBEAT      0x04u
#define HOST_CMD_START_STATE    0x12u
#define HOST_CMD_ACK            0x20u

/* 各命令数据区长度（protocol.md v1.5） */
#define HOST_LEN_VELOCITY       12u
#define HOST_LEN_ESTOP          0u
#define HOST_LEN_MECH           10u     /* v1.5：前 6 字节 + seq(4B) */
#define HOST_LEN_HEARTBEAT      2u
#define HOST_LEN_START_STATE    1u
#define HOST_LEN_ACK            6u      /* v1.5：cmd + status + seq(4B) */

/* 0x20 通用确认的 status */
#define HOST_ST_OK              0u
#define HOST_ST_BUSY            1u
#define HOST_ST_CRC_ERR         2u
#define HOST_ST_TIMEOUT         3u
#define HOST_ST_UNKNOWN_CMD     4u
#define HOST_ST_ESTOP           5u
#define HOST_ST_LIMIT           6u

/* ------------------------------------------------------------------ */
/* 上位机速度指令                                                      */
/* ------------------------------------------------------------------ */
typedef struct
{
    float    vx;            /*!< 车头前向速度 (m/s)          */
    float    vy;            /*!< 车体左向速度 (m/s)          */
    float    omega;         /*!< 绕 +Z 角速度 (rad/s)，逆时针正 */
    uint32_t sequence;      /*!< 每收到一条有效 0x01 递增     */
    uint32_t last_rx_tick;  /*!< 最近有效 0x01 的 HAL_GetTick() 毫秒值 */
    uint8_t  valid;         /*!< 是否收到过至少一条有效 0x01  */
} HostVelocity_t;

/* 速度、接收时刻/序号与安全状态作为同一个快照读取。 */
typedef struct
{
    HostVelocity_t velocity;
    uint8_t estop;
    uint8_t timeout;
} HostVelocitySnapshot_t;

/* 保留公开变量供 Watch 调试；控制代码必须使用快照接口。 */
extern volatile HostVelocity_t g_host_vel;

extern volatile uint8_t  g_host_estop;      /*!< 1 = 急停闭锁中            */
extern volatile uint8_t  g_host_timeout;    /*!< 1 = 300 ms 未收到有效 0x01 */

/* 最近一条 0x03 机构指令（本期无执行器，仅记录供调试/后续接管） */
extern volatile uint8_t  g_host_mech_id;
extern volatile uint8_t  g_host_mech_state;
extern volatile float    g_host_mech_param;
extern volatile uint32_t g_host_mech_seq;

/* 调试用发车开关：Watch 里改成 1 即发车，改回 0 即未发车 */
extern volatile uint8_t  g_host_start_override;

/* 接收统计，每秒刷新一次 */
extern volatile uint32_t g_host_rx_rate;      /*!< 每秒有效速度帧数          */
extern volatile uint32_t g_host_max_gap_ms;   /*!< 统计周期内最大帧间隔 (ms) */
extern volatile uint32_t g_host_max_gap_all_ms; /*!< 上电以来最大帧间隔 (ms) */
extern volatile uint32_t g_host_last_rx_ms;     /*!< 距最近一次有效 0x01 的毫秒数 */
extern volatile uint32_t g_host_stat_poll_cnt;  /*!< 1 s 统计窗口执行次数      */
extern volatile uint32_t g_usb_out_pps;         /*!< 每秒 CDC OUT 包数         */
extern volatile uint32_t g_usb_out_Bps;         /*!< 每秒 CDC 层字节数         */
extern volatile uint32_t g_host_ok_pps;         /*!< 每秒成功解析的帧数        */

/* 调试计数（Keil Watch / Ozone 观察用） */
extern volatile uint32_t g_host_rx_bytes;
extern volatile uint32_t g_host_frame_ok;
extern volatile uint32_t g_host_crc_err;
extern volatile uint32_t g_host_len_err;
extern volatile uint32_t g_host_unknown_cmd;
extern volatile uint32_t g_host_float_err;
extern volatile uint32_t g_host_timeout_cnt;
extern volatile uint32_t g_host_tx_drop; /*!< ACK队列满丢弃计数 */
extern volatile uint32_t g_host_usb_init_fail; /*!< USB接收任务创建失败次数 */
extern volatile uint32_t g_host_usb_heap_free; /*!< 最近初始化尝试后的RTOS空闲堆字节 */
extern volatile uint8_t  g_host_usb_rx_ready;  /*!< 接收任务已创建，不代表USB当前已枚举 */

/* ------------------------------------------------------------------ */
/* 接口                                                                */
/* ------------------------------------------------------------------ */

/**
 * @brief 初始化上位机链路、创建任务并使能已有 TIM1 1 ms 安全检查
 * @note  在 Task_Init() 中调用一次；不创建/重启 HAL 时基定时器。
 */
void UsbUpper_Init(void);

/**
 * @brief USB CDC 接收回调，由 usb_trans 的 USB_RecvTask 调用
 * @param src  本批原始字节
 * @param size 本批字节数（是"批次"不是"帧"，内部按流式状态机处理）
 * @note  运行在任务上下文，不在中断里
 */
void UsbUpper_OnUsbRx(uint8_t *src, uint16_t size);

/** @brief USB TX完成ISR通知，不在此回调中复用发送buffer */
void UsbUpper_OnUsbTxComplete(void);

/**
 * @brief 设置发车状态（0 = 未发车，1 = 发车）
 * @note  状态变化会立即上行一帧，之后按 10 Hz 重发当前值
 */
void UsbUpper_SetStartState(uint8_t start);

/** @brief 读取当前发车状态 */
uint8_t UsbUpper_GetStartState(void);

/** @brief 复制一致的速度、安全标志和毫秒时间戳快照，返回非零表示已读 */
uint8_t UsbUpper_GetVelocitySnapshot(HostVelocitySnapshot_t *out);

/** @brief 由 TIM1 1 ms ISR调用；返回当前主机速度是否不安全 */
uint8_t UsbUpper_WatchdogTick1ms(void);

/** @brief 当前上位机速度是否可用于底盘控制（基于真实毫秒超时） */
uint8_t UsbUpper_IsVelocityUsable(void);

/** @brief 快照是否不安全（急停 / 超时） */
uint8_t Chassis_HostUnsafe(const HostVelocitySnapshot_t *host, uint32_t now);

/** @brief TIM1 1 ms：上位机不安全时给行走 VESC 补零电流 */
void Chassis_HostStopFromISR(uint8_t unsafe);

/** @brief 下发单轮行走电流（含上位机安全门控），由 Chassis_Task 调用 */
void Chassis_SendDriveCurrent(uint8_t index);

#endif /* __USB_UPPER_H__ */
