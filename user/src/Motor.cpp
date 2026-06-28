#include "Motor.h"


Motor::Motor()
{
    ls2k0300_gpio_init(&DIR, MOTOR_IO_PIN, GPIO_MODE_OUT, GPIO_MUX_GPIO);
    ls2k0300_gpio_level_set(&DIR, GPIO_LOW);
    ls2k0300_gtim_pwm_init(&VAL, MOTOR_PWM_PIN, 15000, 0, GTIM_PWM_POL_INV);

    ls2k0300_gpio_init(&EN, MOTOR_EN_PIN, GPIO_MODE_OUT, GPIO_MUX_GPIO);
    ls2k0300_gpio_level_set(&EN, GPIO_HIGH);
    sleep(1);

}

Motor::~Motor()
{
    ls2k0300_gtim_pwm_set_duty(&VAL, 0);
    ls2k0300_gpio_level_set(&EN, GPIO_LOW);

    ls2k0300_gpio_deinit(&EN);
    ls2k0300_gpio_deinit(&DIR);
    ls2k0300_gtim_pwm_deinit(&VAL);

}

void Motor::Motor_output(int duty)
{
    if(duty >= 0)
    {
        ls2k0300_gpio_level_set(&DIR, GPIO_LOW);
        ls2k0300_gtim_pwm_set_duty(&VAL, duty);

    }
    else
    {
        ls2k0300_gpio_level_set(&DIR, GPIO_HIGH);
        ls2k0300_gtim_pwm_set_duty(&VAL, -duty);

    }

}

