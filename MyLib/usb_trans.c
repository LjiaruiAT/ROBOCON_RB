#include "usb_trans.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

static TaskHandle_t kUsbRecvTaskHandle;
static Recv_finished_cb_t Recv_Finished_Cb;
static RecvBufferOverflow_cb_t Recv_Overflow_cb;
static void *kUserData;

/* 每个完成的 OUT 包在 ISR 里立即拷进环形队列并马上重新挂载端点，
 * 解析耗时不再形成 NAK 背压（否则主机会因为写阻塞而抽稀发包频率）。
 * Init/DeInit 可能跑在调度器之前，只发布普通状态。 */
#define USB_RX_SLOTS   8u

typedef struct
{
    uint16_t len;
    uint32_t generation;
    uint8_t  buf[CDC_DATA_FS_MAX_PACKET_SIZE];
} UsbRxSlot_t;

static volatile uint32_t s_generation;
static uint32_t s_delivery_generation;
static volatile uint8_t s_cdc_active;
static volatile uint8_t s_rx_armed;
static volatile uint8_t s_rx_error;

static UsbRxSlot_t       s_rx_ring[USB_RX_SLOTS];
static volatile uint8_t  s_rx_head;   /*!< ISR 侧写入 */
static volatile uint8_t  s_rx_tail;   /*!< 任务侧读取 */

/* 链路诊断计数：区分"主机没发够"和"设备把包丢了"。Keil Watch 直接观察。 */
volatile uint32_t g_usb_out_pkts;      /*!< CDC OUT 回调次数（= 主机发出的包数） */
volatile uint32_t g_usb_out_bytes;     /*!< CDC 层累计收到字节数                 */
volatile uint32_t g_usb_deliver_pkts;  /*!< 真正交给上层解析的包数               */
volatile uint32_t g_usb_drop_pending;  /*!< 上一包未取走又来新包，被丢弃         */
volatile uint32_t g_usb_drop_invalid;  /*!< Buf/Len 非法，被丢弃                 */
volatile uint32_t g_usb_rearm_fail;    /*!< OUT 端点重新挂载失败                 */
volatile uint32_t g_usb_reset_cnt;     /*!< 重新枚举/复位次数                    */

static uint8_t USB_CDC_ConfiguredLocked(void)
{
    return (s_cdc_active != 0u) &&
           (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) &&
           (hUsbDeviceFS.pClassData != NULL) &&
           (hUsbDeviceFS.pClassDataCmsit[hUsbDeviceFS.classId] == hUsbDeviceFS.pClassData);
}

uint32_t USB_CDC_GetGeneration(void)
{
    return s_generation;
}

uint32_t USB_CDC_GetDeliveryGeneration(void)
{
    return s_delivery_generation;
}

uint8_t USB_CDC_IsConfigured(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t configured;
    __disable_irq();
    configured = USB_CDC_ConfiguredLocked();
    __set_PRIMASK(primask);
    return configured;
}

void USB_CDC_ClassInit(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    ++s_generation;
    g_usb_reset_cnt++;
    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_rx_error = 0u;
    s_cdc_active = 1u;
    /* ST's class Init automatically prepares the first OUT packet after this
     * interface callback returns. Do not submit it a second time in the task. */
    s_rx_armed = 1u;
    __DMB();
    __set_PRIMASK(primask);
}

void USB_CDC_ClassDeInit(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    ++s_generation;
    s_cdc_active = 0u;
    s_rx_armed = 0u;
    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_rx_error = 0u;
    __DMB();
    __set_PRIMASK(primask);
}

static void USB_RecvTask(void *param)
{
    uint8_t packet[CDC_DATA_FS_MAX_PACKET_SIZE];
    (void)param;

    for (;;)
    {
        uint32_t generation = 0u;
        uint16_t size = 0u;
        uint8_t error;
        uint8_t have = 0u;
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (s_rx_tail != s_rx_head)
        {
            size = s_rx_ring[s_rx_tail].len;
            generation = s_rx_ring[s_rx_tail].generation;
            /* A private copy also survives a reset/re-enumeration while the
             * application callback is running and ST rearms its first packet. */
            if (size != 0u)
                memcpy(packet, s_rx_ring[s_rx_tail].buf, size);
            s_rx_tail = (uint8_t)((s_rx_tail + 1u) % USB_RX_SLOTS);
            have = 1u;
        }
        error = s_rx_error;
        s_rx_error = 0u;
        __set_PRIMASK(primask);

        if ((error != 0u) && (Recv_Overflow_cb != NULL))
            Recv_Overflow_cb(kUserData);
        if ((have != 0u) && (size != 0u) && (generation == USB_CDC_GetGeneration()) &&
            (USB_CDC_IsConfigured() != 0u))
        {
            s_delivery_generation = generation;
            g_usb_deliver_pkts++;
            __DMB();
            Recv_Finished_Cb(packet, size);
        }

        /* OUT 端点由 ISR 立即重新挂载；这里只是断链/挂载失败时的兜底重试。 */
        primask = __get_PRIMASK();
        __disable_irq();
        if ((USB_CDC_ConfiguredLocked() != 0u) && (s_rx_armed == 0u))
        {
            if ((USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS) == USBD_OK) &&
                (USBD_CDC_ReceivePacket(&hUsbDeviceFS) == USBD_OK))
                s_rx_armed = 1u;
            else
                g_usb_rearm_fail++;
        }
        __set_PRIMASK(primask);

        /* 环形队列里还有包就立即继续，不要每包多等一个 tick。 */
        if (s_rx_tail != s_rx_head)
            continue;

        /* Notifications accelerate packets. The finite wait also retries a
         * failed rearm and observes lifecycle flags without RTOS calls in
         * Init/DeInit (including configuration before this task exists). */
        (void)ulTaskNotifyTake(pdTRUE, 1u);
    }
}

BaseType_t USB_CDC_Init(Recv_finished_cb_t recv_cb,
                      RecvBufferOverflow_cb_t recv_overflow_cb, void *user_data)
{
    if (kUsbRecvTaskHandle != NULL)
        return pdPASS;
    if (recv_cb == NULL)
        return pdFAIL;

    Recv_Finished_Cb = recv_cb;
    Recv_Overflow_cb = recv_overflow_cb;
    kUserData = user_data;
    /* No heap-allocated queue: the static ring is filled by the ISR even when
     * enumeration precedes task creation or task allocation fails. */
    if (xTaskCreate(USB_RecvTask, "usb_cdc_recv", 256, NULL, 4,
                    &kUsbRecvTaskHandle) != pdPASS)
    {
        kUsbRecvTaskHandle = NULL;
        return pdFAIL;
    }
    return pdPASS;
}

void CDC_RecvCplt_Handler(uint8_t *Buf, uint32_t *Len)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    s_rx_armed = 0u;
    if (s_cdc_active == 0u)
        return;

    g_usb_out_pkts++;

    if ((Buf == NULL) || (Len == NULL) || (Buf != UserRxBufferFS) ||
        (*Len > CDC_DATA_FS_MAX_PACKET_SIZE))
    {
        g_usb_drop_invalid++;
        s_rx_error = 1u;
    }
    else
    {
        uint8_t  next = (uint8_t)((s_rx_head + 1u) % USB_RX_SLOTS);
        uint16_t len  = (uint16_t)*Len;

        g_usb_out_bytes += len;
        if (next == s_rx_tail)
        {
            /* 环形队列满：丢这一包，但下面仍然立即重新挂载，不把主机卡住。 */
            g_usb_drop_pending++;
            s_rx_error = 1u;
        }
        else
        {
            if (len != 0u)
                memcpy(s_rx_ring[s_rx_head].buf, Buf, len);
            s_rx_ring[s_rx_head].len = len;
            s_rx_ring[s_rx_head].generation = s_generation;
            __DMB();
            s_rx_head = next;
        }
    }

    /* 数据已拷走，立刻重新挂载 OUT：主机不会再因为解析耗时而被 NAK 拖慢。 */
    if ((USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS) == USBD_OK) &&
        (USBD_CDC_ReceivePacket(&hUsbDeviceFS) == USBD_OK))
        s_rx_armed = 1u;
    else
        g_usb_rearm_fail++;

    if (kUsbRecvTaskHandle != NULL)
    {
        vTaskNotifyGiveFromISR(kUsbRecvTaskHandle, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}
