/**
 * @file    usb_upper.c
 * @brief   上位机（ROS 2）USB CDC 协议实现
 *
 * 数据流：
 *   USB FS ISR -> CDC_Receive_FS -> CDC_RecvCplt_Handler -> 单包 mailbox
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
volatile uint32_t g_host_usb_init_fail;
volatile uint32_t g_host_usb_heap_free;
volatile uint8_t  g_host_usb_rx_ready;

/* 调试用：Watch 窗口里改成 1 即发车，改回 0 即未发车 */
volatile uint8_t  g_host_start_override = 0u;

/* 接收统计：每秒自动刷新一次，Watch 里直接读，不用手数 */
volatile uint32_t g_host_rx_rate;        /*!< 每秒有效速度帧数          */
volatile uint32_t g_host_max_gap_ms;     /*!< 统计周期内最大帧间隔 (ms) */
volatile uint32_t g_host_max_gap_all_ms; /*!< 上电以来最大帧间隔 (ms)，不按秒清零 */
volatile uint32_t g_host_last_rx_ms;     /*!< 距最近一次有效 0x01 的毫秒数，实时 */
volatile uint32_t g_host_stat_poll_cnt;  /*!< 1 s 统计窗口执行次数，验证秒窗口本身在跑 */
volatile uint32_t g_usb_out_pps;         /*!< 每秒 CDC OUT 包数（主机真正发出来的） */
volatile uint32_t g_usb_out_Bps;         /*!< 每秒 CDC 层字节数                        */
volatile uint32_t g_host_ok_pps;         /*!< 每秒成功解析的帧数（含心跳等）          */

static uint32_t   s_pkts_last;
static uint32_t   s_bytes_last;
static uint32_t   s_ok_last;

static uint32_t   s_rate_cnt;
static uint32_t   s_gap_max;
static uint32_t   s_rate_t0;
static uint8_t    s_rx_started;

/* HAL_Init()已启动TIM1；初始化完成前不能触发底盘安全输出。 */
static volatile uint8_t s_wd_inited;

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
static uint32_t     s_txq_generation;

static uint8_t      s_tx_pending;
static uint8_t      s_tx_is_start;
static TickType_t   s_last_start_tx;
static uint8_t      s_tx_buf[HOST_TX_ITEM_MAX];
static uint8_t      s_tx_len;
static uint32_t     s_tx_generation;
static uint32_t     s_rx_generation;
volatile uint32_t   g_host_tx_drop;

static void host_tx_wake(void)
{
    if (s_upper_task != NULL)
    {
        xTaskNotifyGive(s_upper_task);
    }
}

void UsbUpper_OnUsbTxComplete(void)
{
    BaseType_t woken = pdFALSE;

    if (s_upper_task != NULL)
    {
        vTaskNotifyGiveFromISR(s_upper_task, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

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
/* 看门狗：只有有效0x01发布才喂狗，HAL毫秒差不依赖底盘任务进度。        */
/* ------------------------------------------------------------------ */
static void host_publish_velocity(float vx, float vy, float om)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t now;

    __disable_irq();
    now = HAL_GetTick();
    if (s_rx_started != 0u)
    {
        uint32_t gap = now - g_host_vel.last_rx_tick;
        if (gap > s_gap_max) { s_gap_max = gap; }
        if (gap > g_host_max_gap_all_ms) { g_host_max_gap_all_ms = gap; }
        s_rate_cnt++;
    }
    else
    {
        s_rx_started = 1u;      /* 第一帧不计上电至今的间隔 */
        s_rate_cnt   = 1u;
    }

    g_host_vel.vx = vx;
    g_host_vel.vy = vy;
    g_host_vel.omega = om;
    g_host_vel.last_rx_tick = now;
    g_host_vel.sequence++;
    g_host_vel.valid = 1u;
    g_host_timeout = 0u;
    /* 延续协议：只有非零有效速度解除急停闭锁。 */
    if ((vx != 0.0f) || (vy != 0.0f) || (om != 0.0f))
    {
        g_host_estop = 0u;
    }
    __set_PRIMASK(primask);
}

/* 调用者已屏蔽中断，或正在TIM1 ISR中；无RTOS API和等待。 */
static uint8_t host_wd_update(uint32_t now)
{
    if (s_wd_inited == 0u) { return 0u; }
    if ((g_host_vel.valid != 0u) &&
        ((uint32_t)(now - g_host_vel.last_rx_tick) >= HOST_WD_TIMEOUT_MS))
    {
        if (g_host_timeout == 0u)
        {
            g_host_timeout = 1u;
            g_host_timeout_cnt++;
        }
    }
    return ((g_host_estop != 0u) ||
            ((g_host_vel.valid != 0u) && (g_host_timeout != 0u))) ? 1u : 0u;
}

/* 每1秒结算一次；与USB接收任务的累加/清零互斥，间隔单位恒为ms。 */
static void host_stat_poll(void)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t now;

    __disable_irq();
    now = HAL_GetTick();
    g_host_last_rx_ms = (g_host_vel.valid != 0u)
                      ? (uint32_t)(now - g_host_vel.last_rx_tick)
                      : 0xFFFFFFFFu;
    if ((uint32_t)(now - s_rate_t0) >= 1000u)
    {
        g_host_rx_rate    = s_rate_cnt;
        g_host_max_gap_ms = s_gap_max;
        g_host_ok_pps     = g_host_frame_ok - s_ok_last;
        g_usb_out_pps     = g_usb_out_pkts - s_pkts_last;
        g_usb_out_Bps     = g_usb_out_bytes - s_bytes_last;
        s_ok_last         = g_host_frame_ok;
        s_pkts_last       = g_usb_out_pkts;
        s_bytes_last      = g_usb_out_bytes;
        s_rate_cnt = 0u;
        s_gap_max  = 0u;
        s_rate_t0  = now;
        g_host_stat_poll_cnt++;
    }
    __set_PRIMASK(primask);
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

    /* 队列只包含当前 USB 会话的回执。 */
    taskENTER_CRITICAL();
    if (s_txq_generation != USB_CDC_GetGeneration())
    {
        s_txq_head = 0u;
        s_txq_tail = 0u;
        s_txq_generation = USB_CDC_GetGeneration();
    }
    next = (uint8_t)((s_txq_head + 1u) % HOST_TXQ_DEPTH);
    if (next == s_txq_tail)
    {
        g_host_tx_drop++;
        taskEXIT_CRITICAL();
        return 0u;
    }
    memcpy(s_txq[s_txq_head].buf, buf, len);
    s_txq[s_txq_head].len = len;
    s_txq_head = next;
    taskEXIT_CRITICAL();
    host_tx_wake();
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

            /* 原子发布速度、序号、毫秒时间戳和急停解除状态。 */
            host_publish_velocity(vx, vy, om);

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
            /* estop独立于速度valid，且不修改速度接收看门狗时间戳。 */
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
 * @brief 扫描 s_frame 已缓存的字节，消费所有能确认的帧
 * @note  只在缓冲区头部推进 pos；CRC/长度非法时按协议回退 1 字节重找 AA。
 *        正常路径每个字节只判定一次，且整包只做一次搬移。
 */
static void host_scan(void)
{
    uint16_t pos = 0u;
    uint16_t total;
    uint16_t crc_calc;
    uint16_t crc_recv;
    uint8_t  cmd;
    uint8_t  dlen;

    for (;;)
    {
        while ((pos < s_frame_len) && (s_frame[pos] != HOST_FRAME_HEAD0))
        {
            pos++;                          /* 不是帧头，逐字节丢弃 */
        }
        if ((uint16_t)(s_frame_len - pos) < 2u)
        {
            break;                          /* 连方向字节都还没到 */
        }
        if (s_frame[pos + 1u] != HOST_FRAME_DIR_DOWN)
        {
            pos++;                          /* 0x56 回显或其他方向：丢 1 字节 */
            continue;
        }
        if ((uint16_t)(s_frame_len - pos) < 4u)
        {
            break;                          /* 还没收到 cmd/len */
        }

        cmd  = s_frame[pos + 2u];
        dlen = s_frame[pos + 3u];

        if (dlen > HOST_DATA_MAX)
        {
            g_host_len_err++;
            pos++;                          /* 非法长度：丢 1 字节重新同步 */
            continue;
        }

        total = (uint16_t)(HOST_FRAME_OVERHEAD + dlen);
        if ((uint16_t)(s_frame_len - pos) < total)
        {
            break;                          /* 帧没收全，等下一字节 */
        }

        crc_calc = host_crc16(&s_frame[pos + 2u], (uint16_t)(2u + dlen));
        crc_recv = (uint16_t)s_frame[pos + total - 2u]
                 | (uint16_t)((uint16_t)s_frame[pos + total - 1u] << 8);

        if (crc_calc != crc_recv)
        {
            g_host_crc_err++;
            pos++;                          /* CRC 错误：回退 1 字节重新找 AA */
            continue;
        }

        host_dispatch(cmd, &s_frame[pos + 4u], dlen);
        g_host_frame_ok++;
        pos = (uint16_t)(pos + total);
    }

    /* 已确认无用的前缀一次性丢掉，避免逐帧 memmove */
    if (pos != 0u)
    {
        if (pos >= s_frame_len)
        {
            s_frame_len = 0u;
        }
        else
        {
            memmove(s_frame, &s_frame[pos], (size_t)(s_frame_len - pos));
            s_frame_len = (uint16_t)(s_frame_len - pos);
        }
    }
}

static void host_feed(uint8_t byte)
{
    /* 防御：正常流里 s_frame_len 到不了上限（最长帧 70 B 收全即被消费） */
    if (s_frame_len >= HOST_FRAME_MAX)
    {
        s_frame_len = 0u;
    }

    s_frame[s_frame_len++] = byte;
    g_host_rx_bytes++;
    host_scan();
}

void UsbUpper_OnUsbRx(uint8_t *src, uint16_t size)
{
    uint16_t i;
    uint32_t generation;

    if ((src == NULL) || (size == 0u))
    {
        return;
    }

    /* src 是 usb_trans 交付的私有拷贝，s_frame 也只被本任务访问，
       所以解析全程不需要临界区：只把原子的速度发布留在 host_publish_velocity。 */
    generation = USB_CDC_GetDeliveryGeneration();
    if (s_rx_generation != generation)
    {
        s_frame_len = 0u;                   /* 新会话：丢掉上一会话的半帧 */
        s_rx_generation = generation;
    }

    for (i = 0u; i < size; i++)
    {
        host_feed(src[i]);
    }

    /* 包所属会话若在解析期间失效，半帧不能带进新会话 */
    if (generation != USB_CDC_GetGeneration())
    {
        s_frame_len = 0u;
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
    uint32_t generation;

    /* 检查、复用缓冲及提交与 USB reset 中断串行。 */
    taskENTER_CRITICAL();
    generation = USB_CDC_GetGeneration();
    if (s_tx_generation != generation)
    {
        s_tx_generation = generation;
        s_tx_pending = 0u;
        if (s_txq_generation != generation)
        {
            s_txq_head = 0u;
            s_txq_tail = 0u;
            s_txq_generation = generation;
        }
        s_start_dirty = 1u;
    }

    if ((USB_CDC_IsConfigured() == 0u) || (CDC_TransmitBusy_FS() != 0u))
    {
        taskEXIT_CRITICAL();
        return;
    }

    if (s_tx_pending == 0u)
    {
        s_tx_is_start = 0u;
        if (host_txq_pop() == 0u)
        {
            if ((s_start_dirty == 0u) &&
                ((now - s_last_start_tx) < pdMS_TO_TICKS(HOST_START_TX_MS)))
            {
                taskEXIT_CRITICAL();
                return;
            }
            s_tx_len = host_build_start_frame(s_tx_buf);
            s_tx_is_start = 1u;
        }
        s_tx_pending = 1u;
    }

    /* BUSY/FAIL 保留该项；实际 TxState 清零前绝不改写发送缓冲。 */
    if (CDC_Transmit_FS(s_tx_buf, (uint16_t)s_tx_len) == USBD_OK)
    {
        s_tx_pending = 0u;
        if (s_tx_is_start != 0u)
        {
            s_last_start_tx = now;
            s_start_dirty = (s_tx_buf[4] != (s_start_state & 0x01u)) ? 1u : 0u;
        }
    }
    taskEXIT_CRITICAL();
}

void UsbUpper_SetStartState(uint8_t start)
{
    uint8_t v = (start != 0u) ? 1u : 0u;

    if (v != s_start_state)
    {
        s_start_state = v;
        s_start_dirty = 1u;
        host_tx_wake();
    }
}

uint8_t UsbUpper_GetStartState(void)
{
    return (uint8_t)s_start_state;
}

/** Copy the complete command and safety state as one interrupt-atomic snapshot. */
uint8_t UsbUpper_GetVelocitySnapshot(HostVelocitySnapshot_t *out)
{
    uint32_t primask;

    if (out == NULL) { return 0u; }
    primask = __get_PRIMASK();
    __disable_irq();
    out->velocity.vx = g_host_vel.vx;
    out->velocity.vy = g_host_vel.vy;
    out->velocity.omega = g_host_vel.omega;
    out->velocity.sequence = g_host_vel.sequence;
    out->velocity.last_rx_tick = g_host_vel.last_rx_tick;
    out->velocity.valid = g_host_vel.valid;
    out->estop = g_host_estop;
    out->timeout = g_host_timeout;
    __set_PRIMASK(primask);
    return 1u;
}

/** 1 ms hook called after HAL_IncTick from TIM1. Safe until UsbUpper_Init. */
uint8_t UsbUpper_WatchdogTick1ms(void)
{
    return host_wd_update(HAL_GetTick());
}

/** Compatibility query for callers outside the chassis loop. */
uint8_t UsbUpper_IsVelocityUsable(void)
{
    HostVelocitySnapshot_t snapshot;
    uint32_t now;

    if (UsbUpper_GetVelocitySnapshot(&snapshot) == 0u) { return 0u; }
    if (snapshot.estop != 0u) { return 0u; }
    if (snapshot.velocity.valid == 0u) { return 0u; }
    now = HAL_GetTick();
    return ((uint32_t)(now - snapshot.velocity.last_rx_tick) < HOST_WD_TIMEOUT_MS) ? 1u : 0u;
}
int a = 0;
/* ------------------------------------------------------------------ */
/* 任务                                                                */
/* ------------------------------------------------------------------ */
static void UsbUpper_Task(void *argument)
{
    uint8_t usb_inited = 0u;

    (void)argument;

    /* 上电默认未发车，尽快让上位机看到 start=0 */
//    s_start_state = 0u;
    s_start_dirty = 1u;

    for (;;)
    {
			if (a == 1)
{
g_host_start_override = 1;
}
        host_stat_poll();

        /* 任务创建失败时保留重试机会；首次 OUT 包由传输层暂存。 */
        if ((usb_inited == 0u) && (USB_CDC_IsConfigured() != 0u))
        {
            if (USB_CDC_Init(UsbUpper_OnUsbRx, NULL, NULL) == pdPASS)
            {
                usb_inited = 1u;
                g_host_usb_rx_ready = 1u;
            }
            else
            {
                g_host_usb_init_fail++;
            }
            g_host_usb_heap_free = (uint32_t)xPortGetFreeHeapSize();
        }

        /* 发车信号：调试期用 g_host_start_override，正式接入时换成 GPIO */
        UsbUpper_SetStartState((g_host_start_override != 0u) ? 1u : 0u);

        if (usb_inited != 0u)
        {
            host_tx_poll();
        }

        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(HOST_TASK_PERIOD_MS));
    }
}

/* ------------------------------------------------------------------ */
/* 初始化                                                              */
/* ------------------------------------------------------------------ */
void UsbUpper_Init(void)
{
    BaseType_t created;

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
    s_txq_head      = 0u;
    s_txq_tail      = 0u;
    s_txq_generation = USB_CDC_GetGeneration();
    s_tx_pending    = 0u;
    s_tx_is_start   = 0u;
    s_tx_len        = 0u;
    s_tx_generation = s_txq_generation;
    s_rx_generation = s_txq_generation;
    s_last_start_tx = 0u;
    g_host_tx_drop  = 0u;
    g_host_usb_init_fail = 0u;
    g_host_usb_heap_free = 0u;
    g_host_usb_rx_ready = 0u;

    g_host_rx_rate    = 0u;
    g_host_max_gap_ms = 0u;
    g_host_max_gap_all_ms = 0u;
    g_host_last_rx_ms = 0xFFFFFFFFu;
    g_host_stat_poll_cnt = 0u;
    g_usb_out_pps     = 0u;
    g_usb_out_Bps     = 0u;
    g_host_ok_pps     = 0u;
    s_pkts_last       = g_usb_out_pkts;
    s_bytes_last      = g_usb_out_bytes;
    s_ok_last         = 0u;
    s_rate_cnt        = 0u;
    s_gap_max         = 0u;
    s_rate_t0         = HAL_GetTick();
    s_rx_started      = 0u;
    s_wd_inited       = 1u;

    created = xTaskCreate(UsbUpper_Task, "usb_upper", 256, NULL, 3, &s_upper_task);
    configASSERT(created == pdPASS);
}
