#ifndef __GANTRY_H__
#define __GANTRY_H__

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motorEx.h"

#define GANTRY_ARM_DELAY_MS 5000u
#define GANTRY_CMD_CURRENT  0.0f
#define UP_CMD_ANGLE 5800.0f
typedef enum
{
    GANTRY_IDLE = 0,
    GANTRY_PB10,
    GANTRY_PB11,
    GANTRY_BOTH,
	  GANTRY_DOWN
} GantryState_t;

extern Motor3508Ex_t g_gantry_3508;
extern TaskHandle_t  Gantry_Task_Handle;

extern volatile GantryState_t g_gantry_state;
extern volatile uint8_t       g_gantry_armed;
extern volatile uint32_t      g_gantry_ms_in_state;

void Gantry_Task(void *pvParameters);

#endif
