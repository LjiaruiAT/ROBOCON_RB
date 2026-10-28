#include "app.h"
#include "swerve.h"
#include "usb_upper.h"
#include <math.h>
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

static uint8_t s_host_unsafe;
static uint8_t s_host_stop_pending;
static volatile uint8_t s_drive_configured;

/* ------------------------------------------------------------------ */
/* 底盘实测速度：轮速反馈(ERPM) + 转向角反算车体 vx/vy/wz              */
/* 每秒结算一次，用于和指令速度对比（Watch 里直接看）                   */
/* ------------------------------------------------------------------ */
#define CHAS_ODOM_WINDOW_MS   1000u

volatile float g_chas_in_vx;        /*!< 上 1 s 平均：本轮实际下发的指令 vx (m/s) */
volatile float g_chas_in_vy;        /*!< 上 1 s 平均：本轮实际下发的指令 vy (m/s) */
volatile float g_chas_in_wz;        /*!< 上 1 s 平均：本轮实际下发的指令 wz (rad/s) */
volatile float g_chas_act_vx;       /*!< 上 1 s 平均：轮速反算的实测 vx (m/s) */
volatile float g_chas_act_vy;       /*!< 上 1 s 平均：轮速反算的实测 vy (m/s) */
volatile float g_chas_act_wz;       /*!< 上 1 s 平均：轮速反算的实测 wz (rad/s) */
volatile float g_chas_act_vx_now;   /*!< 本周期实测（瞬时，2 ms 刷新） */
volatile float g_chas_act_vy_now;
volatile float g_chas_act_wz_now;
volatile float g_chas_wheel_mps[SWERVE_COUNT];  /*!< 单轮实测线速度 (m/s，沿当前轮向) */
volatile float g_chas_wheel_deg[SWERVE_COUNT];  /*!< 单轮当前朝向 (deg，0=车头) */
volatile float g_chas_distance_m;               /*!< 上电后累计行驶距离 (m) */
volatile uint32_t g_chas_window_ms;             /*!< 上一个统计窗口实际长度 (ms) */
volatile uint32_t g_chas_window_n;              /*!< 窗口内样本数（2 ms 周期应≈500） */

static float    s_sum_in_vx;
static float    s_sum_in_vy;
static float    s_sum_in_wz;
static float    s_sum_act_vx;
static float    s_sum_act_vy;
static float    s_sum_act_wz;
static uint32_t s_odom_n;
static TickType_t s_odom_t0;

static uint8_t Chassis_HostUnsafe(const HostVelocitySnapshot_t *host, uint32_t now)
{
	return ((host->estop != 0u) ||
	        ((host->velocity.valid != 0u) &&
	         ((host->timeout != 0u) ||
	          ((uint32_t)(now - host->velocity.last_rx_tick) >= HOST_WD_TIMEOUT_MS)))) ? 1u : 0u;
}

/* TIM1调用；一次unsafe转换只调度一轮，邮箱满/未初始化逐tick重试。
   HAL_OK只代表提交到CAN邮箱，零电流不是制动、也不代表物理即时停车。 */
void Chassis_HostStopFromISR(uint8_t unsafe)
{
	uint8_t i;

	if (unsafe == 0u)
	{
		s_host_unsafe = 0u;
		s_host_stop_pending = 0u;
		return;                     /* 首次host接管前不干预遥控 */
	}
	if (s_host_unsafe == 0u)
	{
		s_host_unsafe = 1u;
		s_host_stop_pending = (uint8_t)((1u << SWERVE_COUNT) - 1u);
	}
	for (i = 0u; i < SWERVE_COUNT; i++)
	{
		uint8_t bit = (uint8_t)(1u << i);
		VESC_t *drive = &swerve[i].drive;
		if ((s_host_stop_pending & bit) == 0u) { continue; }
		if ((s_drive_configured == 0u) || (drive->hcan == NULL) || (drive->hcan != &hcan2) ||
		    (drive->motor_id == 0u) || (hcan2.Instance != CAN2) ||
		    (hcan2.State != HAL_CAN_STATE_LISTENING))
		{
			continue;
		}
		/* VESC_SetCurrent/HAL无等待；每tick最多四次HAL提交尝试。 */
		if (VESC_SetCurrent(drive, 0.0f) == HAL_OK)
		{
			s_host_stop_pending &= (uint8_t)~bit;
		}
		else
		{
			can_tx_skip_cnt++;
		}
	}
}

/* 仅包住一次快照检查和一次非阻塞提交，避免TIM1与任务抢同一CAN邮箱。
   保存/恢复PRIMASK允许安全嵌套；不在临界区执行PID或等待邮箱。 */
static void Chassis_SendDriveCurrent(uint8_t index)
{
	HostVelocitySnapshot_t host;
	uint32_t primask;
	VESC_t *drive = &swerve[index].drive;

	primask = __get_PRIMASK();
	__disable_irq();
	(void)UsbUpper_GetVelocitySnapshot(&host);
	if ((Chassis_HostUnsafe(&host, HAL_GetTick()) == 0u) &&
	    (s_drive_configured != 0u) && (drive->hcan == &hcan2) &&
	    (drive->motor_id != 0u) && (hcan2.Instance == CAN2) &&
	    (hcan2.State == HAL_CAN_STATE_LISTENING))
	{
		if (VESC_SetCurrent(drive, swerve[index].drive_current) != HAL_OK)
		{
			can_tx_skip_cnt++;
		}
	}
	__set_PRIMASK(primask);
}

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
	static const float pos_x[SWERVE_COUNT] = {  0.215f,  0.215f, -0.215f, -0.215f };//up_left is +
	static const float pos_y[SWERVE_COUNT] = {  -0.225f, -0.225f, -0.225f,  -0.225f };

	static const uint32_t steer_id[SWERVE_COUNT] = { 0x201, 0x202, 0x203, 0x204 };
	static const uint32_t drive_id[SWERVE_COUNT] = { 0x01, 0x02, 0x03, 0x04 };

	static GPIO_TypeDef * const key_port[SWERVE_COUNT] = { GPIOC, GPIOC, GPIOB, GPIOB };
	static const uint16_t        key_pin [SWERVE_COUNT] = { GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_0, GPIO_PIN_1 };

	static const float dir_drive_tab[SWERVE_COUNT] = {  1.0f, -1.0f, -1.0f,  1.0f };


	static const float zero_bias_tab[SWERVE_COUNT] = {
		 45.0f, -135.0f, -135.0f,  45.0f
	};

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
		m->zero_bias_deg = zero_bias_tab[i];
		m->dir_steer  = 1.0f;
		m->dir_drive  = dir_drive_tab[i];
		m->erpm_per_mps   = DRIVE_ERPM_PER_MPS;     
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

		m->drive_pid.Kp = 0.010f;     
		m->drive_pid.Ki = 0.0005f;
		m->drive_pid.Kd = 0.0f;
		m->drive_pid.limit = 1000.0f;
		m->drive_pid.output_limit = 30.0f;
	}
	s_drive_configured = 1u; /* 发布完整配置；此前TIM1只保留pending、不发送 */
}

void is_turn_down(void)
{
	uint8_t aligned = 1; 

	for (uint8_t i = 0; i < SWERVE_COUNT; i++)
	{
		if (!Swerve_IsAligned(&swerve[i])) { aligned = 0; break; }
	}

	is_turn_flag = aligned ? 0 : 1;

	if (is_turn_flag)
	{
		for (uint8_t i = 0; i < SWERVE_COUNT; i++)
		{
			swerve[i].drive_current = 0.0f;
		}
	}
}

/* 每秒结算窗口清零（进入正常控制前调用一次） */
static void Chassis_OdomReset(void)
{
	s_sum_in_vx  = 0.0f;
	s_sum_in_vy  = 0.0f;
	s_sum_in_wz  = 0.0f;
	s_sum_act_vx = 0.0f;
	s_sum_act_vy = 0.0f;
	s_sum_act_wz = 0.0f;
	s_odom_n     = 0u;
	s_odom_t0    = xTaskGetTickCount();
	g_chas_distance_m = 0.0f;
}

/* 每个控制周期（2 ms）调用一次，参数是本周期真正下发给底盘的指令速度。
   实测速度由“VESC ERPM + 转向编码器角度”逐轮合成为车体速度：
     vix = v_i·cosθ_i，viy = v_i·sinθ_i
   对称布局下 Σx = Σy = 0，所以直接求和就是最小二乘解。 */
static void Chassis_OdomStep(float in_vx, float in_vy, float in_wz)
{
	const float inv_n = 1.0f / (float)SWERVE_COUNT;
	const float dt_s  = (float)CHASSIS_TASK_PERIOD_MS * 0.001f;
	float sum_vx = 0.0f;
	float sum_vy = 0.0f;
	float num_wz = 0.0f;
	float den_wz = 0.0f;
	uint32_t elapsed;
	uint8_t i;

	for (i = 0u; i < SWERVE_COUNT; i++)
	{
		SwerveModule_t *m = &swerve[i];
		float mps = 0.0f;
		float rad;
		float vix;
		float viy;

		if (m->erpm_per_mps > 1.0f)
		{
			/* VESC 反馈电转速 → 轮子线速度 (m/s)；dir_drive 为 ±1 */
			mps = ((float)m->drive.epm / m->erpm_per_mps) * m->dir_drive;
		}
		g_chas_wheel_mps[i] = mps;
		g_chas_wheel_deg[i] = m->angle_deg;

		rad = m->angle_deg * (PI / 180.0f);
		vix = mps * cosf(rad);
		viy = mps * sinf(rad);

		sum_vx += vix;
		sum_vy += viy;
		num_wz += (m->x * viy) - (m->y * vix);
		den_wz += (m->x * m->x) + (m->y * m->y);
	}

	g_chas_act_vx_now = sum_vx * inv_n;
	g_chas_act_vy_now = sum_vy * inv_n;
	g_chas_act_wz_now = (den_wz > 1.0e-6f) ? (num_wz / den_wz) : 0.0f;

	g_chas_distance_m += sqrtf(g_chas_act_vx_now * g_chas_act_vx_now +
	                           g_chas_act_vy_now * g_chas_act_vy_now) * dt_s;

	s_sum_in_vx  += in_vx;
	s_sum_in_vy  += in_vy;
	s_sum_in_wz  += in_wz;
	s_sum_act_vx += g_chas_act_vx_now;
	s_sum_act_vy += g_chas_act_vy_now;
	s_sum_act_wz += g_chas_act_wz_now;
	s_odom_n++;

	/* 每秒结算一次：输出上 1 s 的指令/实测平均值 */
	elapsed = (uint32_t)(xTaskGetTickCount() - s_odom_t0);
	if (elapsed >= CHAS_ODOM_WINDOW_MS)
	{
		if (s_odom_n > 0u)
		{
			const float inv = 1.0f / (float)s_odom_n;

			g_chas_in_vx  = s_sum_in_vx  * inv;
			g_chas_in_vy  = s_sum_in_vy  * inv;
			g_chas_in_wz  = s_sum_in_wz  * inv;
			g_chas_act_vx = s_sum_act_vx * inv;
			g_chas_act_vy = s_sum_act_vy * inv;
			g_chas_act_wz = s_sum_act_wz * inv;
		}
		g_chas_window_ms = elapsed;
		g_chas_window_n  = s_odom_n;

		s_sum_in_vx  = 0.0f;
		s_sum_in_vy  = 0.0f;
		s_sum_in_wz  = 0.0f;
		s_sum_act_vx = 0.0f;
		s_sum_act_vy = 0.0f;
		s_sum_act_wz = 0.0f;
		s_odom_n     = 0u;
		s_odom_t0    = xTaskGetTickCount();
	}
}

void Chassis_Task(void *pvParameters)
{
	chassis_stage = 0;
	s_drive_configured = 0u;
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
	Chassis_OdomReset();
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


		HostVelocitySnapshot_t host;
		(void)UsbUpper_GetVelocitySnapshot(&host);

		/* 收到过上位机有效0x01后，控制权永久交给上位机。
		   掉线/急停只停车，不回退遥控器；急停独立于valid生效。 */
		if ((host.velocity.valid != 0u) || (host.estop != 0u))
		{
			if (Chassis_HostUnsafe(&host, HAL_GetTick()) == 0u)
			{
				vx = host.velocity.vx;
				vy = host.velocity.vy;
				wz = host.velocity.omega;
			}
			else
			{
				vx = 0.0f;
				vy = 0.0f;
				wz = 0.0f;
			}
		}

		int16_t steer_cur[SWERVE_COUNT];

		for (uint8_t i = 0; i < SWERVE_COUNT; i++)
		{
			steer_cur[i] = (int16_t)Swerve_ModuleControl(&swerve[i], vx, vy, wz,
			                                             SWERVE_STEER_CUR_LIMIT,
			                                             SWERVE_VESC_CUR_LIMIT);
		}

		/* 实测速度：以本周期真正下发到轮子的指令为对比基准 */
		Chassis_OdomStep(vx, vy, wz);

		is_turn_down();

		MotorSend(&hcan1, 0x200, steer_cur);



static uint8_t tx_phase = 0u;

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
	uint32_t ID  = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	 ID = CAN_Receive_DataFrame(hcan, Recv);

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
