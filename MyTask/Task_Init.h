#ifndef _TASK_INIT_H_
#define _TASK_INIT_H_
#include "config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "queue.h"
#include "CANDrive.h"
#include "RobStride2.h"
#include "PID_old.h"
#include "math.h"
#include "motor.h"
#include "motorEx.h"
#include "usart.h"
#include "dataFrame.h"
#include "comm.h"
#include "comm_stm32_hal_middle.h"
#include "VESC.h"
#include "semphr.h"

typedef struct{
	uint8_t Left_Key_Up;         
	uint8_t Left_Key_Down;       
	uint8_t Left_Key_Left;       
	uint8_t Left_Key_Right;       
	uint8_t Left_Switch_Up;       
	uint8_t Left_Switch_Down;
	uint8_t Left_Broadside_Key;

	uint8_t Right_Key_Up;        
	uint8_t Right_Key_Down;      
	uint8_t Right_Key_Left;      
	uint8_t Right_Key_Right;     
	uint8_t Right_Switch_Up;      
	uint8_t Right_Switch_Down;      
	uint8_t Right_Broadside_Key;
} hw_key_t;
  
typedef struct {
    float Ex;
    float Ey;
    float Eomega;
    hw_key_t First,Second;
} Remote_Handle_t;
typedef struct
{
	PID2 PID;
	int dead_area;
	VESC_t steer;
}VESC_INIT;
extern SemaphoreHandle_t Remote_semaphore;
extern Remote_Handle_t   Remote_Control;
extern PackControl_t     recv_pack;
extern uint8_t           recv_buff[20];
extern VESC_INIT         vesc_1, vesc_2, vesc_3, vesc_4;
extern CommPackRecv_Cb   recv_cb;
void Task_Init(void);
void Remote(void *pvParameters);
extern TaskHandle_t Remote_Handle;
void Remote_Analysis_Task(void *pvParameters);
extern TaskHandle_t Remote_Analysis_Handle;
void Remote_Analysis();
extern uint8_t usart5_dma_buff[60];
extern uint8_t usart6_dma_buff[30];

#endif

