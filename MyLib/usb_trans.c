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

/* One completed OUT packet owns UserRxBufferFS until the task copies it.
 * The endpoint stays NAKed until parsing finishes; no short packet/ZLP needed.
 * Init/DeInit can run before the scheduler, so they only publish plain state. */
static volatile uint32_t s_generation;
static volatile uint32_t s_rx_generation;
static uint32_t s_delivery_generation;
static volatile uint16_t s_rx_size;
static volatile uint8_t s_cdc_active;
static volatile uint8_t s_rx_armed;
static volatile uint8_t s_rx_pending;
static volatile uint8_t s_rx_error;

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
    s_rx_pending = 0u;
    s_rx_error = 0u;
    s_rx_size = 0u;
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
    s_rx_pending = 0u;
    s_rx_error = 0u;
    s_rx_size = 0u;
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
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (s_rx_pending != 0u)
        {
            generation = s_rx_generation;
            size = s_rx_size;
            /* A private copy also survives a reset/re-enumeration while the
             * application callback is running and ST rearms its first packet. */
            if (size != 0u)
                memcpy(packet, UserRxBufferFS, size);
            s_rx_pending = 0u;
        }
        error = s_rx_error;
        s_rx_error = 0u;
        __set_PRIMASK(primask);

        if ((error != 0u) && (Recv_Overflow_cb != NULL))
            Recv_Overflow_cb(kUserData);
        if ((size != 0u) && (generation == USB_CDC_GetGeneration()) &&
            (USB_CDC_IsConfigured() != 0u))
        {
            s_delivery_generation = generation;
            __DMB();
            Recv_Finished_Cb(packet, size);
        }

        primask = __get_PRIMASK();
        __disable_irq();
        if ((USB_CDC_ConfiguredLocked() != 0u) &&
            (s_rx_armed == 0u) && (s_rx_pending == 0u))
        {
            if ((USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS) == USBD_OK) &&
                (USBD_CDC_ReceivePacket(&hUsbDeviceFS) == USBD_OK))
                s_rx_armed = 1u;
        }
        __set_PRIMASK(primask);

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
    /* No heap-allocated queue: the first packet is retained in the mailbox
     * even when enumeration precedes task creation or task allocation fails. */
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
    if ((Buf == NULL) || (Len == NULL) || (Buf != UserRxBufferFS) ||
        (*Len > CDC_DATA_FS_MAX_PACKET_SIZE) || (s_rx_pending != 0u))
    {
        /* Keep an existing packet intact; task reports an abnormal callback.
         * Normal bulk traffic cannot fill this mailbox because OUT is NAKed. */
        s_rx_error = 1u;
    }
    else
    {
        s_rx_size = (uint16_t)*Len;
        s_rx_generation = s_generation;
        __DMB();
        s_rx_pending = 1u;
    }

    if (kUsbRecvTaskHandle != NULL)
    {
        vTaskNotifyGiveFromISR(kUsbRecvTaskHandle, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}
