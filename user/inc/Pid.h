#ifndef _PID_H_
#define _PID_H_

#define MAX_OUTPUT 100.0f
class   Pid
{

public:
    Pid(float Kp, float Ki, float Kd, float max_output);
    float calculate(float setpoint, float measured_value);
    float incremental_calculate(float setpoint, float measured_value);
    void Set_config(float set_Kp, float set_Ki, float set_Kd);
    void Set_output(float set_output);


private:
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float last_error;
    float last_last_error;
    float output;
    float max_output;

};



#endif /* _PID_H_ */
