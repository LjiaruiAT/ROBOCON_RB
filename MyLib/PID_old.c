/**
 * @file    PID.c
 * @author  yao
 * @date    1-May-2020
 * @brief   PID模块
 */

#include "PID_old.h"
#include <stddef.h>


/**
 * @brief 标准位置式PID（积分/微分周期由调用者给定）
 * @param dt_s 本环真实执行周期 (s)。必须和调用该函数的任务周期一致，
 *             否则 Ki 的物理含义会整体偏离（dt 写大 → 积分增益被放大）。
 */
void PID_ControlDt(float current, float expected, PID2 *parameter, float dt_s)
{
    if (parameter == NULL)
    {
        return;
    }
    if (dt_s <= 0.0f)
    {
        dt_s = 0.002f;
    }

    parameter->error_last = parameter->error_now;
    parameter->error_now = expected - current;

    parameter->error_inter += parameter->error_now * dt_s;

    if (parameter->error_inter > parameter->limit)
        parameter->error_inter = parameter->limit;
    else if (parameter->error_inter < -parameter->limit)
        parameter->error_inter = -parameter->limit;

    parameter->pid_out =
        parameter->Kp * parameter->error_now +
        parameter->Ki * parameter->error_inter +
        parameter->Kd * (parameter->error_now - parameter->error_last) / dt_s;

    if (parameter->pid_out > parameter->output_limit)
        parameter->pid_out = parameter->output_limit;
    else if (parameter->pid_out < -parameter->output_limit)
        parameter->pid_out = -parameter->output_limit;
}

/* 兼容旧调用：Gantry 等 2 ms 任务继续用这个入口 */
void PID_Control2(float current, float expected, PID2 *parameter)
{
    PID_ControlDt(current, expected, parameter, 0.002f);
}


float PID_Increment2(float current, float expect, PID_ADD2 *parameter) {
    parameter->error_now = expect - current;

    parameter->increament =
            parameter->Kp * (parameter->error_now - parameter->error_next) + parameter->Ki * (parameter->error_now) +
            parameter->Kd * (parameter->error_now - 2 * parameter->error_next + parameter->error_last);

    parameter->error_last = parameter->error_next;
    parameter->error_next = parameter->error_now;

    return parameter->increament;
}


