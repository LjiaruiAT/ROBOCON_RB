
#include "Gantry.h"
#include "Task_Init.h"
#include <math.h>

/* 回零判断参数 */
#define GANTRY_ANGLE_TOLERANCE    2.0f
#define GANTRY_SPEED_TOLERANCE    5.0f
#define GANTRY_STABLE_COUNT      20u

Motor3508Ex_t g_gantry_3508 =
{
    .hcan = &hcan1,
    .ID   = 0x205,
};

int16_t motorCurrentBuf[4] = {0};

TaskHandle_t Gantry_Task_Handle;

int flag_init = 0;
int init_angle = 0;
int target_up_angle;

volatile GantryState_t g_gantry_state = GANTRY_IDLE;
volatile uint8_t g_gantry_armed = 0u;
volatile uint32_t g_gantry_ms_in_state = 0u;

/* 1：正在回零，0：正常控制 */
uint8_t gantry_return_flag = 0u;

int test = 0;
int back = 0;


void Gantry_Task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t enter_tick = 0;

    uint8_t pb10;
    uint8_t pb11;

    uint8_t return_stable_count = 0u;

    (void)pvParameters;

    g_gantry_3508.pos_pid.Kp = 22.1f;
    g_gantry_3508.pos_pid.Ki = 0.0f;
    g_gantry_3508.pos_pid.Kd = 0.0f;
    g_gantry_3508.pos_pid.limit = 6000.0f;
    g_gantry_3508.pos_pid.output_limit = 6000.0f;

    g_gantry_3508.vel_pid.Kp = 8.0f;
    g_gantry_3508.vel_pid.Ki = 0.0f;
    g_gantry_3508.vel_pid.Kd = 0.0f;
    g_gantry_3508.vel_pid.limit = 16384.0f;
    g_gantry_3508.vel_pid.output_limit = 10000.0f;

    TickType_t now;
    TickType_t state_start_tick = last_wake;
    for (;;)
    {
now = xTaskGetTickCount();
        if (flag_init == 0)
        {
            init_angle = g_gantry_3508.motor.Angle_DEG;
            target_up_angle = init_angle + UP_CMD_ANGLE;

            flag_init = 1;
        }
            if (KEY_RISING_EDGE(Remote_Control.First,Remote_Control.Second,Left_Key_Up) || test == 1)
            {
							    state_start_tick = now;

                g_gantry_state = GANTRY_PB10;
                test = 0;
            }
            else if (KEY_RISING_EDGE(Remote_Control.First,Remote_Control.Second,Left_Key_Down))
            {
							    state_start_tick = now;

                g_gantry_state = GANTRY_PB11;
            }
            else if (KEY_RISING_EDGE(Remote_Control.First,Remote_Control.Second,Left_Key_Right))
            {
							    state_start_tick = now;

                g_gantry_state = GANTRY_BOTH;
            }
            else if (KEY_RISING_EDGE(Remote_Control.First,Remote_Control.Second,Right_Key_Left) || back == 1)
            {
							    state_start_tick = now;
back = 0;
                g_gantry_state = GANTRY_DOWN;
            }
						switch (g_gantry_state)
{
    case GANTRY_IDLE:
        break;

    case GANTRY_PB10:
        if ((now - state_start_tick) < pdMS_TO_TICKS(1500))
        {
            PID_Control2(g_gantry_3508.motor.Angle_DEG,target_up_angle,&g_gantry_3508.pos_pid);
            PID_Control2(g_gantry_3508.motor.Speed,g_gantry_3508.pos_pid.pid_out,&g_gantry_3508.vel_pid);
            motorCurrentBuf[0] =(int16_t)g_gantry_3508.vel_pid.pid_out;
        }
        else
        {
            
            g_gantry_state = GANTRY_IDLE;
        }
        break;

    case GANTRY_PB11:
        if ((now - state_start_tick) < pdMS_TO_TICKS(1500))
        {
            PID_Control2(g_gantry_3508.motor.Angle_DEG,target_up_angle,&g_gantry_3508.pos_pid);
            PID_Control2(g_gantry_3508.motor.Speed,g_gantry_3508.pos_pid.pid_out,&g_gantry_3508.vel_pid);
            motorCurrentBuf[0] =(int16_t)g_gantry_3508.vel_pid.pid_out;
        }
        else
        {
            g_gantry_state = GANTRY_IDLE;
        }
        break;

    case GANTRY_BOTH:
        if ((now - state_start_tick) < pdMS_TO_TICKS(1500))
        {
            PID_Control2(g_gantry_3508.motor.Angle_DEG,target_up_angle,&g_gantry_3508.pos_pid);
            PID_Control2(g_gantry_3508.motor.Speed,g_gantry_3508.pos_pid.pid_out,&g_gantry_3508.vel_pid);
            motorCurrentBuf[0] =(int16_t)g_gantry_3508.vel_pid.pid_out;
        }
        else
        {    
            g_gantry_state = GANTRY_IDLE;
        }
        break;

    case GANTRY_DOWN:
			        if ((now - state_start_tick) < pdMS_TO_TICKS(1500))
        {
            PID_Control2(g_gantry_3508.motor.Angle_DEG,init_angle,&g_gantry_3508.pos_pid);
            PID_Control2(g_gantry_3508.motor.Speed,g_gantry_3508.pos_pid.pid_out,&g_gantry_3508.vel_pid);
            motorCurrentBuf[0] =(int16_t)g_gantry_3508.vel_pid.pid_out;
        }
        else
        {    motorCurrentBuf[0] = 0;
            g_gantry_state = GANTRY_IDLE;
        }
        break;
}
        MotorSend(&hcan1, 0x1FF, motorCurrentBuf);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2));
    }
}
