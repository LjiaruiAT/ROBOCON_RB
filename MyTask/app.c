#include "app.h"
#include "swerve.h"
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

static void Swerve_GpioInit(void)
{
	GPIO_InitTypeDef gpio = {0};

	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	gpio.Mode  = GPIO_MODE_INPUT;
	gpio.Pull  = GPIO_PULLUP;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;

	gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5;  
	HAL_GPIO_Init(GPIOC, &gpio);

	gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;  
	HAL_GPIO_Init(GPIOB, &gpio);
}

static void Swerve_Config(void)
{
	static const float pos_x[SWERVE_COUNT] = {  0.215f,  0.215f, -0.215f, -0.215f };
	static const float pos_y[SWERVE_COUNT] = {  0.225f, -0.225f,  0.225f, -0.225f };

	static const uint32_t steer_id[SWERVE_COUNT] = { 0x201, 0x202, 0x203, 0x204 };
	static const uint32_t drive_id[SWERVE_COUNT] = { 0x01, 0x02, 0x03, 0x04 };

	static GPIO_TypeDef * const key_port[SWERVE_COUNT] = { GPIOC, GPIOC, GPIOB, GPIOB };
	static const uint16_t        key_pin [SWERVE_COUNT] = { GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_0, GPIO_PIN_1 };

	for (uint8_t i = 0; i < SWERVE_COUNT; i++)
	{
		SwerveModule_t *m = &swerve[i];

		Swerve_ModuleInit(m);        

		m->steer.ID   = steer_id[i];
		m->steer.hcan = &hcan1;      
		m->drive.hcan     = &hcan2; 
		m->drive.motor_id = drive_id[i];

		m->key_port = key_port[i];    
		m->key_pin  = key_pin[i];

		m->x  = pos_x[i];
		m->y  = pos_y[i];
		m->steer_gear = 144.0f;       
		m->zero_deg   = 0.0f;         
		m->dir_steer  = 1.0f;        
		m->dir_drive  = 1.0f;          
		m->erpm_per_mps   = 663.1f;     
		m->steer_deadband = 3.0f;

		m->steer.pos_pid.Kp = 2.0f;    
		m->steer.pos_pid.Ki = 0.0f;
		m->steer.pos_pid.Kd = 0.0f;
		m->steer.pos_pid.limit = 6000.0f;
		m->steer.pos_pid.output_limit = 6000.0f;

		m->steer.vel_pid.Kp = 10.0f;   
		m->steer.vel_pid.Ki = 0.02f;
		m->steer.vel_pid.Kd = 0.0f;
		m->steer.vel_pid.limit = 5000.0f;
		m->steer.vel_pid.output_limit = 10000.0f;

		m->drive_pid.Kp = 0.0250f;     
		m->drive_pid.Ki = 0.0005f;
		m->drive_pid.Kd = 0.3f;
		m->drive_pid.limit = 500.0f;
		m->drive_pid.output_limit = 10.0f;
	}
}

void Chassis_Task(void *pvParameters)
{
	chassis_stage = 0;
	Swerve_GpioInit();    
	Swerve_Config();

	chassis_stage = 1;
	{
		TickType_t t0 = xTaskGetTickCount();

		while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(3000))
		{
			uint8_t all_ready = 1;
			for (uint8_t i = 0; i < SWERVE_COUNT; i++)
			{
				if (!swerve[i].steer.ready) { all_ready = 0; break; }
			}
			if (all_ready) { break; }
			vTaskDelay(pdMS_TO_TICKS(20));
		}
	}

	chassis_stage = 2;
	{
		TickType_t t_reset  = xTaskGetTickCount();
		TickType_t t_reset0 = xTaskGetTickCount();

		for (;;)
		{
			uint8_t all_done = 1;
			int16_t res_cur[SWERVE_COUNT];

			reset_loop_cnt++;

			for (uint8_t i = 0; i < SWERVE_COUNT; i++)
			{
				if (!Swerve_IsResetDone(&swerve[i]))
				{
					Swerve_Reset(&swerve[i]);     
					all_done = 0;
				}
				else
				{
					reset_done_mask |= (uint8_t)(1u << i);
				}
				res_cur[i] = (int16_t)Swerve_SteerLoop(&swerve[i], SWERVE_RESET_CUR_LIMIT);
			}

			MotorSend(&hcan1, 0x200, res_cur);

			if (all_done) { break; }
			if ((xTaskGetTickCount() - t_reset0) > pdMS_TO_TICKS(8000)) { break; }

			vTaskDelayUntil(&t_reset, pdMS_TO_TICKS(CHASSIS_TASK_PERIOD_MS));
		}
	}

	chassis_stage = 3;
	TickType_t last_wake = xTaskGetTickCount();

	for (;;)
	{
		chassis_loop_cnt++;
		if ((chassis_loop_cnt & 0x3Fu) == 0u)
		{
			chassis_stack_free = uxTaskGetStackHighWaterMark(NULL);
		}
		float vy = rc_cmd.vx;
		float vx = rc_cmd.vy;
		float wz = rc_cmd.wz;

		if ((xTaskGetTickCount() - rc_cmd.tick) > pdMS_TO_TICKS(RC_TIMEOUT_MS))
		{
			vx = 0.0f;
			vy = 0.0f;
			wz = 0.0f;
		}

		int16_t steer_cur[SWERVE_COUNT];

		for (uint8_t i = 0; i < SWERVE_COUNT; i++)
		{
			steer_cur[i] = (int16_t)Swerve_ModuleControl(&swerve[i], vx, vy, wz,
			                                             SWERVE_STEER_CUR_LIMIT,
			                                             SWERVE_VESC_CUR_LIMIT);
		}

		MotorSend(&hcan1, 0x200, steer_cur);
		for (uint8_t i = 0; i < SWERVE_COUNT; i++)
		{
			uint32_t t_wait = HAL_GetTick();

			while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0U)  
			{
				if ((HAL_GetTick() - t_wait) > 1U)  
				{
					can_tx_skip_cnt++;
					break;
				}
			}
			VESC_SetCurrent(&swerve[i].drive, swerve[i].drive_current);
		}

		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CHASSIS_TASK_PERIOD_MS));
	}
}
	uint32_t ID  = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	 ID = CAN_Receive_DataFrame(hcan, Recv);

	/* 转向 M2006 反馈（0x201~0x204，标准帧，走 CAN1） */
	Motor2006Recv(&swerve[0].steer, hcan, ID, Recv);
	Motor2006Recv(&swerve[1].steer, hcan, ID, Recv);
	Motor2006Recv(&swerve[2].steer, hcan, ID, Recv);
	Motor2006Recv(&swerve[3].steer, hcan, ID, Recv);
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
	if (huart->Instance == UART5)
	{
		HAL_UART_DMAStop(huart);
		// 重置HAL状态
		huart->ErrorCode = HAL_UART_ERROR_NONE;
		huart->RxState = HAL_UART_STATE_READY;
		huart->gState = HAL_UART_STATE_READY;
		
		// 然后清除错误标志 - 按照STM32F4参考手册要求的顺序
		uint32_t isrflags = READ_REG(huart->Instance->SR);
		
		// 按顺序处理各种错误标志，必须先读SR再读DR来清除错误
		if (isrflags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) 
		{
				// 对于ORE、NE、FE错误，需要先读SR再读DR
				volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
				volatile uint32_t temp_dr = READ_REG(huart->Instance->DR); // 这个读取会清除ORE、NE、FE        

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
