/**
 * @file    PID.c
 * @author  yao
 * @date    1-May-2020
 * @brief   PIDÄ£¿é
 */

#include "PID_old.h"


void PID_Control2(float current, float expected, PID2 *parameter)
{
    const float dt = 0.002f;  // PIDÖÜÆÚ£º2ms

    parameter->error_last = parameter->error_now;
    parameter->error_now = expected - current;

    parameter->error_inter += parameter->error_now * dt;

    if (parameter->error_inter > parameter->limit)
        parameter->error_inter = parameter->limit;
    else if (parameter->error_inter < -parameter->limit)
        parameter->error_inter = -parameter->limit;

    parameter->pid_out =
        parameter->Kp * parameter->error_now +
        parameter->Ki * parameter->error_inter +
        parameter->Kd * (parameter->error_now - parameter->error_last) / dt;

    if (parameter->pid_out > parameter->output_limit)
        parameter->pid_out = parameter->output_limit;
    else if (parameter->pid_out < -parameter->output_limit)
        parameter->pid_out = -parameter->output_limit;
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


