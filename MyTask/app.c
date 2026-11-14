#include "app.h"
#include "swerve.h"
#include "swerve_config.h"
#include "usb_upper.h"
#include "JY61.h"
#include "yaw_hold.h"
#include <math.h>
#include "Gantry.h"
TaskHandle_t Remote_Handle;
TaskHandle_t Remote_Analysis_Handle;
float Vx = 0;
float Vy = 0;
float Wz = 0;


void Remote(void *pvParameters)
{
    g_comm_handle = Comm_Init(&huart5);
    RemoteCommInit(NULL);
    register_comm_recv_cb(recv_cb, 0x01, &recv_pack);

    portTickType xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {


        vTaskDelayUntil(&xLastWakeTime, 2);
    }
}
void Remote_Analysis_Task(void *pvParameters)
{

	while(1)
	{
		Remote_Analysis();
	}
}

RcCmd_t      rc_cmd;
TaskHandle_t Chassis_Task_Handle;

SwerveModule_t swerve[SWERVE_COUNT];

volatile uint32_t can_tx_skip_cnt = 0;  

volatile uint8_t  chassis_stage   = 0;
volatile uint32_t reset_loop_cnt  = 0;
volatile uint8_t  reset_done_mask = 0;
volatile uint32_t chassis_loop_cnt = 0;
volatile UBaseType_t chassis_stack_free = 0;

uint8_t s_host_unsafe;
uint8_t s_host_stop_pending;
volatile uint8_t s_drive_configured;

volatile uint8_t chassis_src_host = 0u;

void Chassis_ToggleCtrlSrc(void)
{
	uint32_t primask = __get_PRIMASK();

	chassis_src_host = (chassis_src_host == 0u) ? 1u : 0u;

	rc_cmd.vx = 0.0f;
	rc_cmd.vy = 0.0f;
	rc_cmd.wz = 0.0f;

	__disable_irq();
	s_host_unsafe = 0u;
	s_host_stop_pending = 0u;
	__set_PRIMASK(primask);
}
volatile uint8_t g_key_lvl[SWERVE_COUNT] = {0, 0, 0, 0};

static void Chassis_KeyMirror(void)
{
	for (uint8_t i = 0; i < SWERVE_COUNT; i++)
	{
		g_key_lvl[i] = Swerve_KeyLevel(&swerve[i]);
	}
}

static void Chassis_GetCmd(float *vx, float *vy, float *wz)
{
    HostVelocitySnapshot_t host;
    (void)UsbUpper_GetVelocitySnapshot(&host);

    if (chassis_src_host != 0u)
    {
        if ((host.velocity.valid != 0u) && (Chassis_HostUnsafe(&host, HAL_GetTick()) == 0u))
        {
            *vx = host.velocity.vx;
            *vy = host.velocity.vy;
            *wz = host.velocity.omega;
            return;
        }
        *vx = 0.0f;
        *vy = 0.0f;
        *wz = 0.0f;
        return;
    }

    *vy = rc_cmd.vx;
    *vx = rc_cmd.vy;
    *wz = rc_cmd.wz;
}

void Chassis_Task(void *pvParameters)
{
    TickType_t t0;
    TickType_t last_wake;
    int16_t    cur[SWERVE_COUNT];
    static uint8_t tx_phase = 0u;

    chassis_stage = 0;
    s_drive_configured = 0u;
    Swerve_Config(swerve);
    s_drive_configured = 1u;

    chassis_stage = 1;
    t0 = xTaskGetTickCount();
    while (!Swerve_AllReady(swerve) && ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(3000)))
    {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    chassis_stage = 2;
    t0 = xTaskGetTickCount();
    last_wake = t0;
    while (!Swerve_ResetStep(swerve, cur))
    {
        Chassis_KeyMirror();
        MotorSend(&hcan1, 0x200, cur);
        if ((xTaskGetTickCount() - t0) > pdMS_TO_TICKS(8000))
        {
            break;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CHASSIS_TASK_PERIOD_MS));
    }

    chassis_stage = 3;
    Swerve_OdomReset();
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        float vx;
        float vy;
        float wz;
        float wz_use;

        chassis_loop_cnt++;
        Chassis_KeyMirror();
        chassis_stack_free = uxTaskGetStackHighWaterMark(NULL);

        Chassis_GetCmd(&vx, &vy, &wz);

        wz_use = wz + WzLoop_Update(wz, sqrtf(vx * vx + vy * vy), g_chas_act_wz_now,
                                    CHASSIS_TASK_PERIOD_MS * 0.001f);

        Swerve_ControlStep(swerve, vx, vy, wz_use, cur);

        if (Swerve_OdomStep(swerve, vx, vy, wz_use, CHASSIS_TASK_PERIOD_MS * 0.001f) != 0u)
        {
            UsbUpper_UpdateChassisVel(g_chas_act_vx_now, g_chas_act_vy_now, g_chas_act_wz_now);
        }

        MotorSend(&hcan1, 0x200, cur);

        if (tx_phase == 0u)
        {
            Chassis_SendDriveCurrent(0u);
            Chassis_SendDriveCurrent(1u);
        }
        else
        {
            Chassis_SendDriveCurrent(2u);
            Chassis_SendDriveCurrent(3u);
        }
        tx_phase ^= 1u;

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CHASSIS_TASK_PERIOD_MS));
    }
}

uint32_t ID = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	 ID = CAN_Receive_DataFrame(hcan, Recv);

	Motor2006Recv(&swerve[0].steer, hcan, ID, Recv);
	Motor2006Recv(&swerve[1].steer, hcan, ID, Recv);
	Motor2006Recv(&swerve[2].steer, hcan, ID, Recv);
	Motor2006Recv(&swerve[3].steer, hcan, ID, Recv);
	Motor3508Recv(&g_gantry_3508, hcan, ID, Recv);
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	uint32_t ID_1 = CAN_Receive_DataFrame(hcan, Recv);



 VESC_ReceiveHandler(&swerve[0].drive, hcan, ID_1, Recv);
 VESC_ReceiveHandler(&swerve[1].drive, hcan, ID_1, Recv);
 VESC_ReceiveHandler(&swerve[2].drive, hcan, ID_1, Recv);
 VESC_ReceiveHandler(&swerve[3].drive, hcan, ID_1, Recv);
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	if (huart->Instance == UART5)
	{
		HAL_UART_DMAStop(&huart5);
		Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_dma_buff,size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_buff,sizeof(usart5_dma_buff));
   	__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	}
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	
	if (huart->Instance == UART4)
	{
		uint32_t isrflags;

		HAL_UART_DMAStop(&huart4);

		
		isrflags = READ_REG(huart4.Instance->SR);
		if (isrflags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE))
		{
			volatile uint32_t tmp_sr = READ_REG(huart4.Instance->SR);
			volatile uint32_t tmp_dr = READ_REG(huart4.Instance->DR);
			(void)tmp_sr;
			(void)tmp_dr;
		}

		huart4.ErrorCode = HAL_UART_ERROR_NONE;
		huart4.RxState   = HAL_UART_STATE_READY;
		huart4.gState    = HAL_UART_STATE_READY;

		__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart4, usart4_dma_buff, sizeof(usart4_dma_buff));
		__HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
		return;
	}

	if (huart->Instance == UART5)
	{
		HAL_UART_DMAStop(huart);
		
		huart->ErrorCode = HAL_UART_ERROR_NONE;
		huart->RxState = HAL_UART_STATE_READY;
		huart->gState = HAL_UART_STATE_READY;
		
		
		uint32_t isrflags = READ_REG(huart->Instance->SR);
		
		
		if (isrflags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) 
		{
				
				volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
				volatile uint32_t temp_dr = READ_REG(huart->Instance->DR); 

		if (isrflags & USART_SR_PE)
		{
				volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
		}
	}
		Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_dma_buff, 0);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_buff,sizeof(usart5_dma_buff));
		__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	}
}
