#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "Encoder.h"
#include "Pid.h"
#include <cstdint>
#include "LS2K0300_DRV_INC.h"

#define MOTOR_EN_PIN     PIN_48
#define MOTOR_PWM_PIN    GTIM_PWM2_PIN89
#define MOTOR_IO_PIN     PIN_88

class Motor {

public:
    Motor();
    ~Motor();
    void Motor_output(int duty);


private:
    ls2k0300_gpio_t DIR, EN;
    ls2k0300_gtim_pwm_t VAL;
    bool dir_ready;
    bool en_ready;
    bool pwm_ready;

};





#endif /* _MOTOR_H_ */
