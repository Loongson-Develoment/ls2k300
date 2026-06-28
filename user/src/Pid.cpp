#include "Pid.h"

Pid::Pid(float Kp, float Ki, float Kd, float max_output)
    : Kp(Kp), 
    Ki(Ki), 
    Kd(Kd), 
    integral(0.0f), 
    last_error(0.0f),
    last_last_error(0.0f),
    output(0.0f),
    max_output(max_output)
{
}

float Pid::calculate(float setpoint, float measured_value)
{
    float error = setpoint - measured_value;
    integral += error;
    output = Kp * error + Ki * integral + Kd * (error - last_error);
    last_error = error;
    if (output > max_output) {
        output = max_output;
    } else if (output < -max_output) {
        output = -max_output;
    }
    return output;
}

float Pid::incremental_calculate(float setpoint, float measured_value)
{
    float error = setpoint - measured_value;
    float delta_error = error - last_error;
    float delta_last_error = last_error - last_last_error;
    float delta_output = Kp * delta_error + Kd * (delta_error - delta_last_error);

    if (!((output >= max_output && error > 0.0f) || (output <= -max_output && error < 0.0f))) {
        delta_output += Ki * error;
    }

    output += delta_output;

    last_last_error = last_error;
    last_error = error;
    if (output > max_output) {
        output = max_output;
    } else if (output < -max_output) {
        output = -max_output;
    }
    return output;
}


void Pid::Set_config(float set_Kp, float set_Ki, float set_Kd)
{
    Kp = set_Kp;
    Ki = set_Ki;
    Kd = set_Kd;
    integral = 0.0;
    last_error = 0.0;
    last_last_error = 0.0;
    output = 0.0;
}

void Pid::Set_output(float set_output)
{
    if (set_output > max_output) {
        output = max_output;
    } else if (set_output < -max_output) {
        output = -max_output;
    } else {
        output = set_output;
    }

    integral = 0.0f;
    last_error = 0.0f;
    last_last_error = 0.0f;
}
