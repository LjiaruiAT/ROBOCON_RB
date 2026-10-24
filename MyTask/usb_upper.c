/**
 * @file    usb_upper.c
 * @brief   上位机（ROS 2）USB CDC 协议实现
 *
 * 数据流：
 *   USB FS ISR -> CDC_Receive_FS -> CDC_RecvCplt_Handler -> kUsbRecvQueue
 *     -> USB_RecvTask -> UsbUpper_OnUsbRx -> 流式帧解析 -> g_host_vel
 *     -> Chassis_Task 读取 -> Swerve_ModuleControl()
 *
 * 上行：
 *   UsbUpper_Task(10 ms) -> 0x12 帧 -> CDC_Transmit_FS
 */
#include "usb_upper.h"
#include "config.h"
#include "usb_trans.h"
#include "usbd_cdc_if.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static TaskHandle_t s_upper_task;

/* ------------------------------------------------------------------ */
/* 对外变量                                                            */
/* ------------------------------------------------------------------ */
volatile HostVelocity_t g_host_vel;

volatile uint8_t  g_host_estop   = 0u;
volatile uint8_t  g_host_timeout = 0u;

volatile uint8_t  g_host_mech_id;
volatile uint8_t  g_host_mech_state;
volatile float    g_host_mech_param;
volatile uint32_t g_host_mech_seq;

volatile uint32_t g_host_rx_bytes;
volatile uint32_t g_host_frame_ok;
volatile uint32_t g_host_crc_err;
volatile uint32_t g_host_len_err;
volatile uint32_t g_host_unknown_cmd;
volatile uint32_t g_host_float_err;
volatile uint32_t g_host_timeout_cnt;

/* 调试用：Watch 窗口里改成 1 即发车，改回 0 即未发车 */
volatile uint8_t  g_host_start_override = 0u;

/* 接收统计：每秒自动刷新一次，Watch 里直接读，不用手数 */
volatile uint32_t g_host_rx_rate;        /*!< 每秒有效速度帧数          */
volatile uint32_t g_host_max_gap_ms;     /*!< 统计周期内最大帧间隔 (ms) */

static uint32_t   s_rate_cnt;
static uint32_t   s_gap_max;
static TickType_t s_rate_t0;
static uint8_t    s_rx_started;

/* 看门狗：连续 N 个底盘周期没收到新帧即超时（150 × 2ms = 300ms） */
#define HOST_WD_MISS_LIMIT   (HOST_WD_TIMEOUT_MS / CHASSIS_TASK_PERIOD_MS)

static uint32_t s_wd_last_seq;
static uint16_t s_wd_miss_cnt;

/* ------------------------------------------------------------------ */
/* 内部状态                                                            */
/* ------------------------------------------------------------------ */
static uint8_t  s_frame[HOST_FRAME_MAX];
static uint16_t s_frame_len;

static volatile uint8_t  s_start_state;
static volatile uint8_t  s_start_dirty;

/* 事件回执队列；0x12 只保留最新状态，不占用回执容量。 */
#define HOST_TXQ_DEPTH      16u
#define HOST_TX_ITEM_MAX    16u

typedef struct
{
    uint8_t len;
    uint8_t buf[HOST_TX_ITEM_MAX];
} HostTxItem_t;

static HostTxItem_t s_txq[HOST_TXQ_DEPTH];
static uint8_t      s_txq_head;
static uint8_t      s_txq_tail;

static uint8_t      s_tx_busy;
static TickType_t   s_last_tx;
static TickType_t   s_last_start_tx;
static uint8_t      s_tx_buf[HOST_TX_ITEM_MAX];
static uint8_t      s_tx_len;

/* ------------------------------------------------------------------ */
/* CRC-16/Modbus                                                       */
/* ------------------------------------------------------------------ */
static uint16_t host_crc16(const uint8_t *d, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    uint8_t  b;

    for (i = 0u; i < len; i++)
    {
        crc ^= (uint16_t)d[i];
        for (b = 0u; b < 8u; b++)
        {
            if ((crc & 0x0001u) != 0u)
            {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            }
            else
            {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

/* ------------------------------------------------------------------ */
/* 数值保护                                                            */
/* ------------------------------------------------------------------ */

/* 等价于 isfinite()，避免依赖编译器的 C99 数学宏 */
static uint8_t host_is_finite(float v)
{
    if (v != v)          { return 0u; }   /* NaN  */
    if (v >  3.0e38f)    { return 0u; }   /* +Inf */
    if (v < -3.0e38f)    { return 0u; }   /* -Inf */
    return 1u;
}

static float host_clamp(float v, float lim)
{
    if (v >  lim) { return  lim; }
    if (v < -lim) { return -lim; }
    return v;
}

/* ------------------------------------------------------------------ */
/* 看门狗：收到有效 0x01 就刷新时刻，由 tick 判断是否超时               */
/* ------------------------------------------------------------------ */
static void host_wd_reset(void)
{
    TickType_t now = xTaskGetTickCount();

    if (s_rx_started != 0u)
    {
        uint32_t gap = (uint32_t)(now - (TickType_t)g_host_vel.last_rx_tick);
        if (gap > s_gap_max) { s_gap_max = gap; }
        s_rate_cnt++;
    }
    else
    {
        s_rx_started = 1u;      /* 第一帧不计间隔，否则会把上电至今算进去 */
        s_rate_cnt   = 1u;
    }

    g_host_vel.last_rx_tick = (uint32_t)now;
    g_host_timeout          = 0u;
}

/* 每 1 秒结算一次接收频率与最大间隔 */
static void host_stat_poll(void)
{
    TickType_t now = xTaskGetTickCount();

    if ((now - s_rate_t0) >= pdMS_TO_TICKS(1000))
    {
        g_host_rx_rate    = s_rate_cnt;
        g_host_max_gap_ms = s_gap_max;

        s_rate_cnt = 0u;
        s_gap_max  = 0u;
        s_rate_t0  = now;
    }
}

/* ------------------------------------------------------------------ */
/* 上行队列                                                            */
/* ------------------------------------------------------------------ */
static uint8_t host_txq_push(const uint8_t *buf, uint8_t len)
{
    uint8_t next;

    if ((buf == NULL) || (len == 0u) || (len > HOST_TX_ITEM_MAX))
    {
        return 0u;
    }

    /* 生产者有两个任务：USB_RecvTask（回执）与 UsbUpper_Task（0x12） */
    taskENTER_CRITICAL();
    next = (uint8_t)((s_txq_head + 1u) % HOST_TXQ_DEPTH);
    if (next == s_txq_tail)
    {
        taskEXIT_CRITICAL();
        return 0u;                      /* 队列满，丢弃 */
    }
    memcpy(s_txq[s_txq_head].buf, buf, len);
    s_txq[s_txq_head].len = len;
    s_txq_head = next;
    taskEXIT_CRITICAL();
    return 1u;
}

static uint8_t host_txq_pop(void)
{
    taskENTER_CRITICAL();
    if (s_txq_tail == s_txq_head)
    {
        taskEXIT_CRITICAL();
        return 0u;
    }
    s_tx_len = s_txq[s_txq_tail].len;
    memcpy(s_tx_buf, s_txq[s_txq_tail].buf, s_tx_len);
    s_txq_tail = (uint8_t)((s_txq_tail + 1u) % HOST_TXQ_DEPTH);
    taskEXIT_CRITICAL();
    return 1u;
}

/**
 * @brief 组装 0x20 通用确认并入队
 * @param cmd    被确认的命令字
 * @param status 状态码（HOST_ST_xxx）
 * @param seq    原样回带下行 seq；无 seq 的命令（0x02 / 未知命令）填 0
 */
static void host_ack_push(uint8_t cmd, uint8_t status, uint32_t seq)
{
    uint8_t  f[HOST_FRAME_OVERHEAD + HOST_LEN_ACK];
    uint16_t crc;

    f[0] = HOST_FRAME_HEAD0;
    f[1] = HOST_FRAME_DIR_UP;
    f[2] = HOST_CMD_ACK;
    f[3] = HOST_LEN_ACK;
    f[4] = cmd;
    f[5] = status;
    f[6] = (uint8_t)(seq & 0xFFu);
    f[7] = (uint8_t)((seq >> 8) & 0xFFu);
    f[8] = (uint8_t)((seq >> 16) & 0xFFu);
    f[9] = (uint8_t)((seq >> 24) & 0xFFu);

    crc = host_crc16(&f[2], (uint16_t)(2u + HOST_LEN_ACK));
    f[10] = (uint8_t)(crc & 0xFFu);
    f[11] = (uint8_t)(crc >> 8);

    (void)host_txq_push(f, (uint8_t)sizeof(f));
}

/* ------------------------------------------------------------------ */
/* 命令分发                                                            */
/* ------------------------------------------------------------------ */
static void host_dispatch(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    switch (cmd)
    {
        case HOST_CMD_VELOCITY:
        {
            float vx;
            float vy;
            float om;

            if (len != 12u)
            {
                g_host_len_err++;
                return;
            }

            memcpy(&vx, &data[0], sizeof(float));
            memcpy(&vy, &data[4], sizeof(float));
            memcpy(&om, &data[8], sizeof(float));

            if ((host_is_finite(vx) == 0u) ||
                (host_is_finite(vy) == 0u) ||
                (host_is_finite(om) == 0u))
            {
                g_host_float_err++;
                return;             /* 非法值：不落地、不刷看门狗 */
            }

            vx = host_clamp(vx, HOST_VX_LIMIT);
            vy = host_clamp(vy, HOST_VY_LIMIT);
            om = host_clamp(om, HOST_OMEGA_LIMIT);

            /* 先刷时间戳：保证任何时刻看到 valid=1，last_rx_tick 已经是新值 */
            host_wd_reset();

            g_host_vel.vx    = vx;
            g_host_vel.vy    = vy;
            g_host_vel.omega = om;
            g_host_vel.sequence++;
            g_host_vel.valid = 1u;

            /* 非零速度指令解除急停闭锁 */
            if ((vx != 0.0f) || (vy != 0.0f) || (om != 0.0f))
            {
                g_host_estop = 0u;
            }

            break;
        }

        case HOST_CMD_ESTOP:
        {
            if (len != HOST_LEN_ESTOP)
            {
                g_host_len_err++;
                return;
            }
            g_host_estop = 1u;
            /* v1.5：急停必须回执；急停帧本身没有 seq 字段，填 0 */
            host_ack_push(HOST_CMD_ESTOP, HOST_ST_OK, 0u);
            break;
        }

        case HOST_CMD_MECH:
        {
            uint8_t  mech_id;
            uint8_t  mech_state;
            float    mech_param;
            uint32_t mech_seq;

            if (len != HOST_LEN_MECH)
            {
                g_host_len_err++;
                return;                 /* 长度不符：不回执，按失步处理 */
            }

            mech_id    = data[0];
            mech_state = data[1];
            memcpy(&mech_param, &data[2], sizeof(float));
            memcpy(&mech_seq,   &data[6], sizeof(uint32_t));

            /* 本期无机构执行器（等机械定稿）：只记录，回 OK */
            g_host_mech_id    = mech_id;
            g_host_mech_state = mech_state;
            g_host_mech_param = mech_param;
            g_host_mech_seq   = mech_seq;

            host_ack_push(HOST_CMD_MECH, HOST_ST_OK, mech_seq);
            break;
        }

        case HOST_CMD_HEARTBEAT:
        {
            if (len != HOST_LEN_HEARTBEAT)
            {
                g_host_len_err++;
                return;
            }
            /* 心跳不刷新速度看门狗，也不回执 */
            break;
        }

        default:
        {
            g_host_unknown_cmd++;
            /* v1.5：未识别命令必须回执。长度未知，seq 只能填 0 */
            host_ack_push(cmd, HOST_ST_UNKNOWN_CMD, 0u);
            break;
        }
    }
}

/* ------------------------------------------------------------------ */
/* 流式帧解析                                                          */
/* ------------------------------------------------------------------ */

/**
 * @brief 尝试从 s_frame 头部解析一帧
 * @return 0 = 数据不足，需要更多字节；>0 = 应丢弃的字节数（解析成功则为整帧长度）
 */
static uint16_t host_try_parse(void)
{
    uint8_t  cmd;
    uint8_t  dlen;
    uint16_t total;
    uint16_t crc_calc;
    uint16_t crc_recv;

    if (s_frame_len < 1u)
    {
        return 0u;
    }

    if (s_frame[0] != HOST_FRAME_HEAD0)
    {
        return 1u;                      /* 不是帧头，丢 1 字节 */
    }

    if (s_frame_len < 2u)
    {
        return 0u;
    }

    if (s_frame[1] != HOST_FRAME_DIR_DOWN)
    {
        return 1u;                      /* 0x56 回显或其他方向：丢 1 字节 */
    }

    if (s_frame_len < 4u)
    {
        return 0u;
    }

    cmd  = s_frame[2];
    dlen = s_frame[3];

    if (dlen > HOST_DATA_MAX)
    {
        g_host_len_err++;
        return 1u;                      /* 非法长度：丢 1 字节重新同步 */
    }

    total = (uint16_t)(HOST_FRAME_OVERHEAD + dlen);
    if (s_frame_len < total)
    {
        return 0u;
    }

    crc_calc = host_crc16(&s_frame[2], (uint16_t)(2u + dlen));
    crc_recv = (uint16_t)s_frame[4u + dlen]
             | (uint16_t)((uint16_t)s_frame[5u + dlen] << 8);

    if (crc_calc != crc_recv)
    {
        g_host_crc_err++;
        return 1u;                      /* CRC 错误：回退 1 字节重新找 AA */
    }

    host_dispatch(cmd, &s_frame[4], dlen);
    g_host_frame_ok++;
    return total;
}

static void host_feed(uint8_t byte)
{
    uint16_t consumed;

    /* 缓冲区满且无法解析，强制清空，防止死锁 */
    if (s_frame_len >= HOST_FRAME_MAX)
    {
        s_frame_len = 0u;
    }

    s_frame[s_frame_len++] = byte;
    g_host_rx_bytes++;

    for (;;)
    {
        if (s_frame_len == 0u)
        {
            return;
        }

        consumed = host_try_parse();
        if (consumed == 0u)
        {
            return;                     /* 数据不足，等下一字节 */
        }

        if (consumed >= s_frame_len)
        {
            s_frame_len = 0u;
            return;
        }

        memmove(s_frame, &s_frame[consumed], (size_t)(s_frame_len - consumed));
        s_frame_len = (uint16_t)(s_frame_len - consumed);
    }
}

void UsbUpper_OnUsbRx(uint8_t *src, uint16_t size)
{
    uint16_t i;

    if ((src == NULL) || (size == 0u))
    {
        return;
    }

    for (i = 0u; i < size; i++)
    {
        host_feed(src[i]);
    }
}

/* ------------------------------------------------------------------ */
/* 上行发送                                                            */
/* ------------------------------------------------------------------ */
/* 组装 0x12 发车状态帧，返回帧长 */
static uint8_t host_build_start_frame(uint8_t *out)
{
    uint16_t crc;

    out[0] = HOST_FRAME_HEAD0;
    out[1] = HOST_FRAME_DIR_UP;
    out[2] = HOST_CMD_START_STATE;
    out[3] = HOST_LEN_START_STATE;
    out[4] = (uint8_t)(s_start_state & 0x01u);

    crc = host_crc16(&out[2], 3u);
    out[5] = (uint8_t)(crc & 0xFFu);
    out[6] = (uint8_t)(crc >> 8);

    return (uint8_t)(HOST_FRAME_OVERHEAD + HOST_LEN_START_STATE);
}

static void host_tx_poll(void)
{
    TickType_t now = xTaskGetTickCount();
    uint8_t    f[HOST_FRAME_OVERHEAD + HOST_LEN_START_STATE];
    uint8_t    flen;

    /* 1. 0x12：状态翻转立即入队，否则按 10 Hz 入队 */
    if ((s_start_dirty != 0u) ||
        ((now - s_last_start_tx) >= pdMS_TO_TICKS(HOST_START_TX_MS)))
    {
        flen = host_build_start_frame(f);
        if (host_txq_push(f, flen) != 0u)
        {
            s_start_dirty   = 0u;
            s_last_start_tx = now;
        }
    }

    /* 2. 上一帧还没发出去：本周期不动 s_tx_buf，下周期再试 */
    if (s_tx_busy != 0u)
    {
        if ((now - s_last_tx) > pdMS_TO_TICKS(5))
        {
            s_tx_busy = 0u;             /* 兜底：USB 断开时不会永久卡住 */
        }
        return;
    }

    /* 3. 从队列取一条发送 */
    if (host_txq_pop() == 0u)
    {
        return;
    }

    if (CDC_Transmit_FS(s_tx_buf, (uint16_t)s_tx_len) == USBD_OK)
    {
        s_tx_busy = 1u;
        s_last_tx = now;
    }
    else
    {
        s_tx_len = 0u;                  /* 发送失败：丢弃，0x12 下周期会重发 */
    }
}

void UsbUpper_SetStartState(uint8_t start)
{
    uint8_t v = (start != 0u) ? 1u : 0u;

    if (v != s_start_state)
    {
        s_start_state = v;
        s_start_dirty = 1u;
    }
}

uint8_t UsbUpper_GetStartState(void)
{
    return (uint8_t)s_start_state;
}

/**
 * @brief 等待速度是否可用
 * @note  超时判据是"连续多少个底盘周期没有新帧"，不看任何时间戳。
 *        只依赖两点：本函数由 Chassis_Task 每个周期调用一次；
 *        USB 侧每收到一帧有效 0x01 就令 g_host_vel.sequence 加一。
 *        全部状态都在本任务的上下文里读写，不存在跨任务竞态。
 */
uint8_t UsbUpper_IsVelocityUsable(void)
{
    uint32_t seq;

    if (g_host_vel.valid == 0u)
    {
        s_wd_miss_cnt = 0u;
        return 0u;                      /* 还没接管 */
    }

    if (g_host_estop != 0u)
    {
        s_wd_miss_cnt = 0u;
        return 0u;                      /* 急停闭锁 */
    }

    seq = g_host_vel.sequence;

    if (seq != s_wd_last_seq)
    {
        /* 本周期有新帧 */
        s_wd_last_seq  = seq;
        s_wd_miss_cnt  = 0u;
        g_host_timeout = 0u;
        return 1u;
    }

    /* 本周期没有新帧 */
    if (s_wd_miss_cnt < HOST_WD_MISS_LIMIT)
    {
        s_wd_miss_cnt++;
    }

    if (s_wd_miss_cnt >= HOST_WD_MISS_LIMIT)
    {
        if (g_host_timeout == 0u)
        {
            g_host_timeout = 1u;
            g_host_timeout_cnt++;
        }
        return 0u;
    }

    return 1u;
}
int a = 0;
/* ------------------------------------------------------------------ */
/* 任务                                                                */
/* ------------------------------------------------------------------ */
static void UsbUpper_Task(void *argument)
{
    TickType_t last = xTaskGetTickCount();
    uint8_t    usb_inited = 0u;

    (void)argument;

    /* 上电默认未发车，尽快让上位机看到 start=0 */
    s_start_state = 0u;
    s_start_dirty = 1u;

    for (;;)
    {
        host_stat_poll();

        /* USB 枚举完成后才能创建接收任务，否则 USBD_CDC_ReceivePacket 会失败 */
        if ((usb_inited == 0u) &&
            (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED))
        {
            USB_CDC_Init(UsbUpper_OnUsbRx, NULL, NULL);
            usb_inited = 1u;
            s_start_inited = 1u;
        }

        /* 发车信号：调试期用 g_host_start_override，正式接入时换成 GPIO */
        UsbUpper_SetStartState((g_host_start_override != 0u) ? 1u : 0u);

        if (s_start_inited != 0u)
        {
            host_tx_poll();
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(HOST_TASK_PERIOD_MS));
    }
}

/* ------------------------------------------------------------------ */
/* 初始化                                                              */
/* ------------------------------------------------------------------ */
void UsbUpper_Init(void)
{
    g_host_vel.vx        = 0.0f;
    g_host_vel.vy        = 0.0f;
    g_host_vel.omega     = 0.0f;
    g_host_vel.sequence     = 0u;
    g_host_vel.last_rx_tick = 0u;
    g_host_vel.valid        = 0u;

    g_host_estop   = 0u;
    g_host_timeout = 0u;

    g_host_mech_id    = 0u;
    g_host_mech_state = 0u;
    g_host_mech_param = 0.0f;
    g_host_mech_seq   = 0u;

    s_frame_len     = 0u;
    s_start_state   = 0u;
    s_start_dirty   = 0u;
    s_start_inited  = 0u;
    s_txq_head      = 0u;
    s_txq_tail      = 0u;
    s_tx_busy       = 0u;
    s_tx_len        = 0u;
    s_last_tx       = 0u;
    s_last_start_tx = 0u;

    g_host_rx_rate    = 0u;
    g_host_max_gap_ms = 0u;
    s_rate_cnt        = 0u;
    s_gap_max         = 0u;
    s_rate_t0         = 0u;
    s_rx_started      = 0u;

    s_wd_last_seq  = 0u;
    s_wd_miss_cnt  = 0u;

    xTaskCreate(UsbUpper_Task, "usb_upper", 256, NULL, 3, NULL);
}
