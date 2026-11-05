#include "Task_Init.h"
#include "app.h"
#include "usb_upper.h"
#include "Gantry.h"
uint8_t usart5_dma_buff[60];
uint8_t usart6_dma_buff[64];
uint8_t usart4_dma_buff[64];   /* IMU（WT901）：PC11 */
 bool is_turn_flag = 0;

void Task_Init()
{

		/* IMU（WT901）：UART4，PC10=TX / PC11=RX */
		__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart4, usart4_dma_buff, sizeof(usart4_dma_buff));
		__HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);

		__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_buff, sizeof(usart5_dma_buff));
		__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);

    	xTaskCreate(Remote,
         "Remote",
          256,
          NULL,
          4,
          &Remote_Handle); 
		xTaskCreate(Remote_Analysis_Task,
		 "Remote_Analysis_Task", 
		  400, NULL, 
		  4, 
		  &Remote_Analysis_Handle);

		xTaskCreate(Chassis_Task,
		 "Chassis_Task",
		  512, NULL,          
		    4,
		  &Chassis_Task_Handle);

          		xTaskCreate(Gantry_Task,
		 "Gantry_Task",
		  256, NULL,          
		    4,
		  &Gantry_Task_Handle);

		UsbUpper_Init();

}
static void Key_Parse(uint32_t key, hw_key_t *out)
{
    out->Right_Switch_Up     = (key & KEY_Right_Switch_Up)     ? 1 : 0;
    out->Right_Switch_Down   = (key & KEY_Right_Switch_Down)   ? 1 : 0;

    out->Right_Key_Up        = (key & KEY_Right_Key_Up)        ? 1 : 0;
    out->Right_Key_Down      = (key & KEY_Right_Key_Down)      ? 1 : 0;
    out->Right_Key_Left      = (key & KEY_Right_Key_Left)      ? 1 : 0;
    out->Right_Key_Right     = (key & KEY_Right_Key_Right)     ? 1 : 0;

    out->Right_Broadside_Key = (key & KEY_Right_Broadside_Key) ? 1 : 0;

    out->Left_Switch_Up      = (key & KEY_Left_Switch_Up)      ? 1 : 0;
    out->Left_Switch_Down    = (key & KEY_Left_Switch_Down)    ? 1 : 0;

    out->Left_Key_Up         = (key & KEY_Left_Key_Up)         ? 1 : 0;
    out->Left_Key_Down       = (key & KEY_Left_Key_Down)       ? 1 : 0;
    out->Left_Key_Left       = (key & KEY_Left_Key_Left)       ? 1 : 0;
    out->Left_Key_Right      = (key & KEY_Left_Key_Right)      ? 1 : 0;

    out->Left_Broadside_Key  = (key & KEY_Left_Broadside_Key)  ? 1 : 0;
}
Remote_Handle_t Remote_Control;
PackControl_t   recv_pack;
uint8_t         recv_buff[20] = {0};

VESC_INIT vesc_1 = { .steer.motor_id = 0x01, .steer.hcan = &hcan2 };
VESC_INIT vesc_2 = { .steer.motor_id = 0x02, .steer.hcan = &hcan2 };
VESC_INIT vesc_3 = { .steer.motor_id = 0x03, .steer.hcan = &hcan2 };
VESC_INIT vesc_4 = { .steer.motor_id = 0x04, .steer.hcan = &hcan2 };
float remote_Ex = 0, remote_Ey = 0, remote_Eomega = 0;
float vx, vy, vz = 0;
void Remote_Analysis(void)
{
    if (xSemaphoreTake(Remote_semaphore, 0) == pdTRUE) {
        Remote_Control.Second = Remote_Control.First;
        Key_Parse(recv_pack.Key, &Remote_Control.First);
        remote_Ex     = -recv_pack.rocker[0] / REMOTE_FIGER * MAX_ROBOT_VEL;
        remote_Ey     =  recv_pack.rocker[1] / REMOTE_FIGER * MAX_ROBOT_VEL;
        remote_Eomega = -recv_pack.rocker[2] / REMOTE_FIGER * MAX_ROBOT_OMEGA;

        float r_x = recv_pack.rocker[0];
        float r_y = recv_pack.rocker[1];
        float r_w = recv_pack.rocker[2];

        if (fabsf(r_x) < RC_DEADZONE) { r_x = 0.0f; }
        if (fabsf(r_y) < RC_DEADZONE) { r_y = 0.0f; }
        if (fabsf(r_w) < RC_DEADZONE) { r_w = 0.0f; }

        rc_cmd.vx = -r_x / RC_FULL_SCALE * CHASSIS_VEL_MAX;
        rc_cmd.vy =  r_y / RC_FULL_SCALE * CHASSIS_VEL_MAX;
        rc_cmd.wz = -r_w / RC_FULL_SCALE * CHASSIS_OMEGA_MAX;
        rc_cmd.tick = xTaskGetTickCount();

        if (KEY_RISING_EDGE(Remote_Control.First, Remote_Control.Second, Left_Key_Left)) {
            Chassis_ToggleCtrlSrc();
        }
    }

    Remote_Control.Ex     = remote_Ex;
    Remote_Control.Ey     = remote_Ey;
    Remote_Control.Eomega = remote_Eomega;

    vTaskDelay(pdMS_TO_TICKS(10));
}

void MyRecvCallback(uint8_t *src, uint16_t size, void *user_data)
{
    memcpy(&recv_buff, src, size);
    memcpy(&recv_pack, recv_buff, sizeof(recv_pack));
    xSemaphoreGive(Remote_semaphore);
}
CommPackRecv_Cb  recv_cb = MyRecvCallback;
