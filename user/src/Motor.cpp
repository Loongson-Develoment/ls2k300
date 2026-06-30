#include "Motor.h"


Motor::Motor()
    : DIR()
    , EN()
    , VAL()
    , dir_ready(false)
    , en_ready(false)
    , pwm_ready(false)
{
    if (ls2k0300_gpio_init(&DIR, MOTOR_IO_PIN, GPIO_MODE_OUT, GPIO_MUX_GPIO) == 0) {
        dir_ready = true;
        ls2k0300_gpio_level_set(&DIR, GPIO_LOW);
    }
    if (ls2k0300_gtim_pwm_init(&VAL, MOTOR_PWM_PIN, 15000, 0, GTIM_PWM_POL_INV) == 0) {
        pwm_ready = true;
    }

    if (ls2k0300_gpio_init(&EN, MOTOR_EN_PIN, GPIO_MODE_OUT, GPIO_MUX_GPIO) == 0) {
        en_ready = true;
        ls2k0300_gpio_level_set(&EN, GPIO_HIGH);
    }
    sleep(1);

}

Motor::~Motor()
{
    if (pwm_ready) {
        ls2k0300_gtim_pwm_set_duty(&VAL, 0);
    }
    if (en_ready) {
        ls2k0300_gpio_level_set(&EN, GPIO_LOW);
    }

    if (pwm_ready) {
        ls2k0300_gtim_pwm_deinit(&VAL);
        pwm_ready = false;
    }
    if (en_ready) {
        ls2k0300_gpio_deinit(&EN);
        en_ready = false;
    }
    if (dir_ready) {
        ls2k0300_gpio_deinit(&DIR);
        dir_ready = false;
    }

}

void Motor::Motor_output(int duty)
{
    if (!dir_ready || !pwm_ready) {
        return;
    }

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
