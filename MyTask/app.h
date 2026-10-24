#include "Task_Init.h"

/* ---------------- 舵轮底盘 ---------------- */

/* 遥控指令快照：Remote_Analysis() 写、Chassis_Task() 读 */
typedef struct
{
	float vx;       /* 前进速度 (m/s，前正) */
	float vy;       /* 横向速度 (m/s，左正) */
	float wz;       /* 自转速度 (rad/s，逆时针正) */
	uint32_t tick;  /* 最后一次遥控更新时刻（失联判据） */
} RcCmd_t;
extern RcCmd_t      rc_cmd;
extern TaskHandle_t Chassis_Task_Handle;
void Chassis_Task(void *pvParameters);

/* TIM1安全路径：仅发送有界VESC零电流重试，不等待CAN邮箱。 */
void Chassis_HostStopFromISR(uint8_t unsafe);
