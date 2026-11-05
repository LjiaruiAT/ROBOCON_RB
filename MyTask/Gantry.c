#include "Gantry.h"
#include "Task_Init.h"

Motor3508Ex_t g_gantry_3508 =
{
    .hcan = &hcan1,
    .ID   = 0x205,
};

int16_t motorCurrentBuf[4] = {0};

TaskHandle_t Gantry_Task_Handle;

volatile GantryState_t g_gantry_state = GANTRY_IDLE;
volatile uint8_t       g_gantry_armed = 0u;
volatile uint32_t      g_gantry_ms_in_state = 0u;

void Gantry_Task(void *pvParameters)
{
	TickType_t last_wake  = xTaskGetTickCount();
	TickType_t enter_tick = 0;
	uint8_t    pb10;
	uint8_t    pb11;

	(void)pvParameters;

	for (;;)
	{
		if (g_gantry_state == GANTRY_IDLE)
		{
			if (KEY_RISING_EDGE(Remote_Control.First, Remote_Control.Second, Left_Key_Up))
			{
				g_gantry_state = GANTRY_PB10;
			}
			else if (KEY_RISING_EDGE(Remote_Control.First, Remote_Control.Second, Left_Key_Down))
			{
				g_gantry_state = GANTRY_PB11;
			}
			else if (KEY_RISING_EDGE(Remote_Control.First, Remote_Control.Second, Left_Key_Right))
			{
				g_gantry_state = GANTRY_BOTH;
			}

			if (g_gantry_state != GANTRY_IDLE)
			{
				enter_tick     = xTaskGetTickCount();
				g_gantry_armed = 0u;
				g_gantry_3508.pos_pid.error_inter = 0.0f;
				g_gantry_3508.vel_pid.error_inter = 0.0f;
			}
		}
		else if (KEY_RISING_EDGE(Remote_Control.First, Remote_Control.Second, Right_Key_Left))
		{
			g_gantry_state = GANTRY_IDLE;
			g_gantry_armed = 0u;
			motorCurrentBuf[0] = 0;
			MotorSend(&hcan1, 0x1FF, motorCurrentBuf);
		}

		pb10 = ((g_gantry_state == GANTRY_PB10) ||
		        (g_gantry_state == GANTRY_BOTH)) ? 1u : 0u;
		pb11 = ((g_gantry_state == GANTRY_PB11) ||
		        (g_gantry_state == GANTRY_BOTH)) ? 1u : 0u;

		GPIOB->BSRR = (pb10 != 0u ? (uint32_t)GPIO_PIN_10 : ((uint32_t)GPIO_PIN_10 << 16)) |
		              (pb11 != 0u ? (uint32_t)GPIO_PIN_11 : ((uint32_t)GPIO_PIN_11 << 16));

		if (g_gantry_state == GANTRY_IDLE)
		{
			g_gantry_ms_in_state = 0u;
		}
		else
		{
			g_gantry_ms_in_state = (uint32_t)(xTaskGetTickCount() - enter_tick) *
			                       (uint32_t)portTICK_PERIOD_MS;

			if ((g_gantry_armed == 0u) &&
			    (g_gantry_ms_in_state >= GANTRY_ARM_DELAY_MS))
			{
				g_gantry_armed = 1u;
			}
		}

		if (g_gantry_armed != 0u)
		{
			PID_Control2(0.0f, 0.0f, &g_gantry_3508.pos_pid);
			PID_Control2(0.0f, 0.0f, &g_gantry_3508.vel_pid);

			motorCurrentBuf[0] =
				(int16_t)(g_gantry_3508.vel_pid.pid_out + GANTRY_CMD_CURRENT);
			MotorSend(&hcan1, 0x1FF, motorCurrentBuf);
		}

		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2));
	}
}
