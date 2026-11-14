#include "Task_Init.h"
#include "swerve.h"

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

extern volatile uint8_t chassis_src_host;
void Chassis_ToggleCtrlSrc(void);

extern SwerveModule_t    swerve[SWERVE_COUNT];
extern volatile uint32_t can_tx_skip_cnt;
extern volatile uint8_t  s_drive_configured;
extern uint8_t           s_host_unsafe;
extern uint8_t           s_host_stop_pending;

/* TIM1安全路径：仅发送有界VESC零电流重试，不等待CAN邮箱。 */
void Chassis_HostStopFromISR(uint8_t unsafe);

/* ---------------- 底盘实测速度（轮速反馈反算，每秒结算） ---------------- */
/* 对比方式：g_chas_in_* 是本秒实际下发的指令均值，g_chas_act_* 是轮速反算的实测均值。
   两者都在车体系（前正 / 左正 / 逆时针正），单位 m/s、rad/s。 */
extern volatile float    g_chas_in_vx;
extern volatile float    g_chas_in_vy;
extern volatile float    g_chas_in_wz;
extern volatile float    g_chas_act_vx;
extern volatile float    g_chas_act_vy;
extern volatile float    g_chas_act_wz;
extern volatile float    g_chas_act_vx_now;   /*!< 瞬时值（2 ms 刷新） */
extern volatile float    g_chas_act_vy_now;
extern volatile float    g_chas_act_wz_now;
extern volatile float    g_chas_distance_m;    /*!< 累计行驶距离 (m) */
extern volatile uint32_t g_chas_window_ms;     /*!< 统计窗口实际长度 (ms) */
extern volatile uint32_t g_chas_window_n;      /*!< 窗口内样本数（2 ms 周期应≈500） */
