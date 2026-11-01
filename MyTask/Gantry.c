#include "Gantry.h"

/* 3508 电机实例：绑定 CAN1，ID 见 Gantry.h */
Motor3508Ex_t g_gantry_3508 =
{
    .hcan = &hcan1,
    .ID   = 0x205,
};


TaskHandle_t Gantry_Task_Handle;
void Gantry_Task(void *pvParameters)
{
	TickType_t last_wake = xTaskGetTickCount();

for(;;)
    {


HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);    
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);  
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);   
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); 






		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2));
    }








}
