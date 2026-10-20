#define PI                 3.14159265359f
#define REMOTE_FIGER       1647.0f
#define MAX_ROBOT_VEL      4.0f               /* 最大线速度 (m/s) */
#define MAX_ROBOT_OMEGA    (PI * 0.3f)        /* 最大角速度 (rad/s) */
#define LENGTH             0.457f             /* 轮心到中心的距离 (m) */
#define WHEEL_RADIUS       0.048f             /* 轮子半径 (m)：直径 96mm */
#define ANGLE2RAD(x)       ((x) * PI / 180.0f)
#define KEY_RISING_EDGE(cur, last, field)  ((cur.field == 1) && (last.field == 0))
#define VESC_POLE_PAIRS    7                  /* 电机极对数 */
#define UP_VOLLEY_MOTOR_MS      1000.0f

/* ---------------- 遥控器（TODO：按实车标定后填值） ---------------- */
#define RC_FULL_SCALE       1647.0f    /* 摇杆满量程（推满看 recv_pack.rocker[] 实测） */
#define RC_DEADZONE         50.0f      /* 摇杆死区（原始值，回中偏置之内） */
#define RC_TIMEOUT_MS       200        /* 遥控失联判定 (ms) */
#define CHASSIS_VEL_MAX     12.0f       /* 遥控最大线速度 (m/s) */
#define CHASSIS_OMEGA_MAX   3.0f       /* 遥控最大自转速度 (rad/s) */

/* ---------------- 舵轮底盘（TODO：按实车标定后填值） ---------------- */
#define CHASSIS_TASK_PERIOD_MS    2         /* 舵轮控制周期 (ms) */
#define SWERVE_STEER_CUR_LIMIT    10000.0f  /* M2006 转向电流限幅 */
#define SWERVE_VESC_CUR_LIMIT     20.0f     /* VESC 行走电流限幅 (A) */
#define SWERVE_RESET_CUR_LIMIT    2000.0f   /* 上电复位扫描时的转向电流限幅（慢速找零位） */
