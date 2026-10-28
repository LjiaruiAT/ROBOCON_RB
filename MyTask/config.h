#define PI                 3.14159265359f
#define REMOTE_FIGER       1647.0f
#define MAX_ROBOT_VEL      4.0f               /* 最大线速度 (m/s) */
#define MAX_ROBOT_OMEGA    (PI * 1.8f)        /* 最大角速度 (rad/s) */
#define LENGTH             0.457f             /* 轮心到中心的距离 (m) */
#define WHEEL_RADIUS       0.048f             /* 轮子半径 (m)：直径 96mm */
#define ANGLE2RAD(x)       ((x) * PI / 180.0f)
#define KEY_RISING_EDGE(cur, last, field)  ((cur.field == 1) && (last.field == 0))
#define VESC_POLE_PAIRS    7                  /* 电机极对数（磁铁片数 / 2） */
#define VESC_POLE_MAGNETS  14                 /* 磁铁片数：VESC Tool 里 Motor Poles 填这个 */

/* ---------------- 行走电机换算（5055 + 行星减速 + 96mm 轮） ---------------- */
#define DRIVE_GEAR_RATIO   3.3333f            /* 行星减速比：电机转 3.333 圈 = 轮子转 1 圈 */
#define DRIVE_WHEEL_DIA    0.096f             /* 轮子直径 (m) */

/* ERPM/(m/s) = 极对数 × 减速比 × 60 / (π × 轮径) ≈ 4641.9 */
#define DRIVE_ERPM_PER_MPS ((VESC_POLE_PAIRS * DRIVE_GEAR_RATIO * 60.0f / (PI * DRIVE_WHEEL_DIA)))*2
#define UP_VOLLEY_MOTOR_MS      1000.0f

/* ---------------- 遥控器（TODO：按实车标定后填值） ---------------- */
#define RC_FULL_SCALE       1647.0f    /* 摇杆满量程（推满看 recv_pack.rocker[] 实测） */
#define RC_DEADZONE         50.0f      /* 摇杆死区（原始值，回中偏置之内） */
#define RC_TIMEOUT_MS       200        /* 遥控失联判定 (ms) */
#define CHASSIS_VEL_MAX     2.0f       /* 遥控最大线速度 (m/s) */
#define CHASSIS_OMEGA_MAX   9.0f       /* 遥控最大自转速度 (rad/s) */

/* ---------------- 舵轮底盘（TODO：按实车标定后填值） ---------------- */
#define CHASSIS_TASK_PERIOD_MS    2         /* 舵轮控制周期 (ms) */
#define SWERVE_STEER_CUR_LIMIT    10000.0f  /* M2006 转向电流限幅 */
#define SWERVE_VESC_CUR_LIMIT     20.0f     /* VESC 行走电流限幅 (A) */
#define SWERVE_RESET_CUR_LIMIT    2000.0f   /* 上电复位扫描时的转向电流限幅（慢速找零位） */

/* ---------------- 上位机 USB 链路（protocol.md v1.4） ---------------- */
#define HOST_VX_LIMIT       3.0f    /* 协议硬闸：vx 前向 (m/s) */
#define HOST_VY_LIMIT       3.0f    /* 协议硬闸：vy 左向 (m/s) */
#define HOST_OMEGA_LIMIT    5.0f    /* 协议硬闸：omega 逆时针 (rad/s) */
#define HOST_WD_TIMEOUT_MS  600u    /* 300 ms 未收到有效 0x01 即停车 */
#define HOST_START_TX_MS    100u    /* 0x12 发车状态重发周期 = 10 Hz */
#define HOST_TASK_PERIOD_MS 10u     /* usb_upper 任务周期 (ms) */
