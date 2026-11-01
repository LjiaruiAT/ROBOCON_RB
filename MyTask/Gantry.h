#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motorEx.h"

extern Motor3508Ex_t g_gantry_3508;
extern TaskHandle_t  Gantry_Task_Handle;

void Gantry_Task(void *pvParameters);
