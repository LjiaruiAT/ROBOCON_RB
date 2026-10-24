#ifndef __USB_TRANS_H__
#define __USB_TRANS_H__

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

/* Task-context byte-stream callback: one nonempty USB FS packet at a time.
 * src is private to the receive task and valid only until the callback returns. */
typedef void (*Recv_finished_cb_t)(uint8_t *src, uint16_t size);
typedef void (*RecvBufferOverflow_cb_t)(void *user_data);

/* Call once from task context (before or after enumeration). pdFAIL leaves the
 * first received packet intact and permits a later retry; pdPASS is idempotent. */
BaseType_t USB_CDC_Init(Recv_finished_cb_t recv_cb,
                      RecvBufferOverflow_cb_t recv_overflow_cb, void *user_data);
uint32_t USB_CDC_GetGeneration(void);
/* Only in recv_cb: generation of the delivered snapshot, stable across reset.
 * Upper parser must compare this with GetGeneration before publishing data. */
uint32_t USB_CDC_GetDeliveryGeneration(void);
uint8_t USB_CDC_IsConfigured(void);

/* CDC interface lifecycle hooks: flags only, no allocation/application/RTOS
 * calls. Generation changes on both Init and DeInit, even between task polls. */
void USB_CDC_ClassInit(void);
void USB_CDC_ClassDeInit(void);
void CDC_RecvCplt_Handler(uint8_t *Buf, uint32_t *Len);

#endif
