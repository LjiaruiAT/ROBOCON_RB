# BR 四舵轮底盘固件

STM32F407 + FreeRTOS，四舵轮全向底盘。支持遥控与上位机（ROS 2）双通道下发速度指令。

## 硬件

| 项 | 说明 |
|---|---|
| 主控 | STM32F407VETx，168 MHz |
| 转向 | M2006 + C610 电调 + 光电门找零，CAN1，电流帧 `0x200`，反馈 `0x201~0x204` |
| 行走 | VESC（5055 + 3.333 行星减速 + 96 mm 轮），CAN2，电流控制 |
| 姿态 | WT901，UART4（PC10 TX / PC11 RX），115200，DMA + IDLE |
| 遥控 | 自定义协议，UART5（PC12 / PD2） |
| 上位机 | USB CDC，协议 v1.4，50 Hz 下发速度 |

底盘几何：轮心 `±0.215 m`（前后）× `±0.225 m`（左右），轮半径 `0.048 m`。
模块索引：`0=左前 1=右前 2=右后 3=左后`。

## 目录

```
MyLib/          驱动与算法：逆解(swerve) VESC 电机 CAN 收发 PID
MyTask/         业务层：Chassis_Task / 遥控 / USB 协议 / 角速度内环(yaw_hold)
Remote_control/ 遥控协议栈
Core/           CubeMX 生成
```

## 控制链路

```
遥控 / 上位机 ──► vx, vy, wz
                    │
              WT901 ──► 角速度内环 (yaw_hold.c)  wz + corr
                    │
                    ▼
              逆解 vix = vx − wz·y
                   viy = vy + wz·x
                    │
        90° 最优角翻转 + cos³ 余弦补偿
                    │
              转向环 / 轮速环（1 ms 周期）
                    │
            CAN1 M2006 电流 / CAN2 VESC 电流
                    │
                  车体 ──► WT901（闭环）
```

三个闭环嵌套：轮速环（单轮）、转向环（轮向）、角速度环（整车偏航）。
**角速度环为内环，闭环 ω 而非航向角**，因此不与上位机的航向闭环冲突。

**周期说明**：底盘任务按 `CHASSIS_TASK_PERIOD_MS = 1` 调度。四路 M2006 转向电流每周期全量下发（共用 `0x200` 帧）；四路 VESC 行走电流分两相位交替、每周期发 2 路，因此**单轮行走电流更新周期为 2 ms**。

## 关键参数（`MyTask/config.h`）

| 宏 | 值 | 含义 |
|---|---|---|
| `CHASSIS_TASK_PERIOD_MS` | 1 | 控制周期 (ms) |
| `CHASSIS_VEL_MAX` / `CHASSIS_OMEGA_MAX` | 2 m/s / 4.5 rad/s | 遥控限幅 |
| `HOST_VX/VY/OMEGA_LIMIT` | 3 / 3 / 5 | 上位机协议硬闸 |
| `SWERVE_STEER_CUR_LIMIT` | 10000 | M2006 电流限幅 |
| `SWERVE_VESC_CUR_LIMIT` | 20 A | VESC 电流限幅 |
| `CHASSIS_USE_TURN_GATE` | 0 | 转向未到位硬门控（已由余弦补偿替代）|

## 上位机协议（v1.4）

帧格式：`AA 55 | cmd | len | data | crc16`

| cmd | 方向 | 内容 |
|---|---|---|
| `0x01` | 下行 | 速度 `vx(4) vy(4) ω(4)` |
| `0x02` | 下行 | 急停 |
| `0x03` | 下行 | 机构 |
| `0x04` | 下行 | 心跳 |
| `0x12` | 上行 | 发车状态 |

掉线 600 ms 自动停车；急停闭锁，需显式解除。

## 编译

Keil MDK（ARMCC V5），工程 `MDK-ARM/BR.uvprojx`。
新增源文件需同时加入工程分组，否则不参与编译。

## 调试变量（Watch）

| 变量 | 含义 |
|---|---|
| `g_gyro_rx_cnt` | IMU 有效帧计数，每秒应涨 ≈ 输出频率 |
| `g_wz_meas` / `g_wz_err` / `g_wz_corr` | 实测角速度 / 误差 / 修正量（rad/s） |
| `g_wz_ok` | 1 = IMU 正常，内环在工作 |
| `g_chas_wheel_mps[4]` | 单轮实测线速度，直行时应基本一致 |
| `g_chas_act_wz` | 轮速反算的角速度，可对比 IMU |

## 注意事项

- WT901 出厂 10 Hz，**必须改到 100 Hz**，否则内环响应跟不上会振荡。
- 建议关闭磁力计（6 轴模式），避免电机磁场使 `Angle.Yaw` 跳变。
- `yaw_hold.h` 的 `WZLOOP_GYRO_SIGN` 需按实际安装方向验证，符号反会变成正反馈。
- 转向零位依赖上电找零，找零失败（光电门未触发）时应禁止发车。
