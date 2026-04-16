#define _POSIX_C_SOURCE 200809L

#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "LS2K0300_DRV_INC.h"

typedef enum example_result {
    EXAMPLE_PASS = 0,
    EXAMPLE_SKIP = 1,
    EXAMPLE_FAIL = -1
} example_result_t;

static volatile sig_atomic_t g_sigint_cb_hit = 0;
static volatile int g_timer_ticks = 0;

static void example_signal_exit_cb(void)
{
    g_sigint_cb_hit = 1;
}

static void example_timer_cb(void *user_data)
{
    (void)user_data;
    g_timer_ticks++;
}

static void example_sleep_ms(uint32_t ms)
{
    struct timespec req;

    req.tv_sec = (time_t)(ms / 1000U);
    req.tv_nsec = (long)((ms % 1000U) * 1000000UL);
    (void)nanosleep(&req, NULL);
}

example_result_t ls2k0300_example_signal(void)
{
    ls2k0300_signal_set_exit_cb(example_signal_exit_cb);
    return EXAMPLE_PASS;
}

example_result_t ls2k0300_example_gpio(void)
{
    ls2k0300_gpio_t gpio;

    if (ls2k0300_gpio_init(&gpio, PIN_64, GPIO_MODE_OUT, GPIO_MUX_GPIO) != 0) {
        return EXAMPLE_FAIL;
    }

    ls2k0300_gpio_level_set(&gpio, GPIO_HIGH);
    ls2k0300_gpio_level_set(&gpio, GPIO_LOW);
    ls2k0300_gpio_deinit(&gpio);
    return EXAMPLE_PASS;
}

example_result_t ls2k0300_example_adc(void)
{
    int raw;
    float voltage;

    if (ls2k0300_adc_init() != 0) {
        return EXAMPLE_FAIL;
    }

    raw = ls2k0300_adc_read_raw(LS_ADC_CH0);
    voltage = ls2k0300_adc_read_voltage(LS_ADC_CH0);
    ls2k0300_adc_deinit();

    if (raw < 0 || voltage < 0.0f) {
        return EXAMPLE_FAIL;
    }
    return EXAMPLE_PASS;
}

example_result_t ls2k0300_example_pwm(void)
{
    ls2k0300_pwm_t pwm;

    if (ls2k0300_pwm_init(&pwm, PWM0_PIN64, 1000U, 5000U, PWM_POL_NORMAL) != 0) {
        return EXAMPLE_FAIL;
    }

    ls2k0300_pwm_set_duty(&pwm, 2500U);
    ls2k0300_pwm_deinit(&pwm);
    return EXAMPLE_PASS;
}

example_result_t ls2k0300_example_uart(void)
{
    ls2k0300_uart_t uart;
    static const uint8_t msg[] = "LS2K0300 uart example\r\n";
    ssize_t written;

    if (ls2k0300_uart_init(&uart, UART1, B115200, LS_UART_STOP1, LS_UART_DATA8, LS_UART_PARITY_NONE) != 0) {
        return EXAMPLE_FAIL;
    }

    written = ls2k0300_uart_write(&uart, msg, sizeof(msg) - 1U);
    ls2k0300_uart_deinit(&uart);
    if (written < 0) {
        return EXAMPLE_FAIL;
    }

    return EXAMPLE_PASS;
}

example_result_t ls2k0300_example_i2c(void)
{
    ls2k0300_i2c_t i2c;

    if (ls2k0300_i2c_init(&i2c, PIN_85, PIN_86, 0x50U) != 0) {
        return EXAMPLE_FAIL;
    }

    ls2k0300_i2c_deinit(&i2c);
    return EXAMPLE_PASS;
}

example_result_t ls2k0300_example_spi(void)
{
    ls2k0300_spi_t spi;
    uint8_t tx[2] = {0xA5U, 0x5AU};
    uint8_t rx[2] = {0U, 0U};
    ssize_t ret;

    if (ls2k0300_spi_init(&spi, LS_SPI2, 1000000U, LS_SPI_MODE_0) != 0) {
        return EXAMPLE_FAIL;
    }

    ret = ls2k0300_spi_transfer(&spi, tx, rx, sizeof(tx));
    ls2k0300_spi_deinit(&spi);

    if (ret != 0) {
        return EXAMPLE_FAIL;
    }
    return EXAMPLE_PASS;
}

example_result_t ls2k0300_example_soft_spi(void)
{
    ls2k0300_soft_spi_t soft_spi;

    if (ls2k0300_soft_spi_init(&soft_spi, PIN_64, PIN_65, PIN_66, PIN_67, LS_SOFT_SPI_MODE_0) != 0) {
        return EXAMPLE_FAIL;
    }

    (void)ls2k0300_soft_spi_read_byte(&soft_spi, 0x00U);
    ls2k0300_soft_spi_deinit(&soft_spi);
    return EXAMPLE_PASS;
}

example_result_t ls2k0300_example_atim_pwm(void)
{
    ls2k0300_atim_pwm_t atim_pwm;

    if (ls2k0300_atim_pwm_init(&atim_pwm, ATIM_PWM0_PIN81, 1000U, 5000U, ATIM_PWM_POL_NORMAL) != 0) {
        return EXAMPLE_FAIL;
    }

    ls2k0300_atim_pwm_deinit(&atim_pwm);
    return EXAMPLE_PASS;
}

example_result_t ls2k0300_example_gtim_pwm(void)
{
    ls2k0300_gtim_pwm_t gtim_pwm;

    if (ls2k0300_gtim_pwm_init(&gtim_pwm, GTIM_PWM0_PIN87, 1000U, 5000U, GTIM_PWM_POL_NORMAL) != 0) {
        return EXAMPLE_FAIL;
    }

    ls2k0300_gtim_pwm_deinit(&gtim_pwm);
    return EXAMPLE_PASS;
}

example_result_t ls2k0300_example_encoder(void)
{
    ls2k0300_pwm_encoder_t enc;

    if (ls2k0300_pwm_encoder_init(&enc, ENC_PWM0_PIN64, PIN_72) != 0) {
        return EXAMPLE_FAIL;
    }

    (void)ls2k0300_pwm_encoder_get_count(&enc);
    ls2k0300_pwm_encoder_deinit(&enc);
    return EXAMPLE_PASS;
}

example_result_t ls2k0300_example_timer(void)
{
    ls2k0300_timer_t timer;
    int ret;

    g_timer_ticks = 0;

    if (ls2k0300_timer_init(&timer) != 0) {
        return EXAMPLE_FAIL;
    }

    ret = ls2k0300_timer_set_seconds_ms(&timer, 20U, example_timer_cb, NULL);
    example_sleep_ms(120U);
    (void)ls2k0300_timer_stop(&timer);
    ls2k0300_timer_deinit(&timer);

    if (ret != 0 || g_timer_ticks <= 0) {
        return EXAMPLE_FAIL;
    }
    return EXAMPLE_PASS;
}

example_result_t ls2k0300_example_canfd(void)
{
    ls2k0300_canfd_t canfd;
    ls2k0300_canfd_frame_t frame;
    int ret;

    if (geteuid() != 0) {
        return EXAMPLE_SKIP;
    }

    if (ls2k0300_canfd_init(&canfd, CAN0, CANFD_MODE_THREAD, NULL, NULL) != 0) {
        return EXAMPLE_FAIL;
    }

    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x123U;
    frame.len = 8U;
    frame.data[0] = 1U;
    frame.data[1] = 2U;
    frame.data[2] = 3U;
    frame.data[3] = 4U;
    frame.data[4] = 5U;
    frame.data[5] = 6U;
    frame.data[6] = 7U;
    frame.data[7] = 8U;

    ret = ls2k0300_canfd_write_frame(&canfd, &frame);
    ls2k0300_canfd_deinit(&canfd);

    if (ret < 0) {
        return EXAMPLE_FAIL;
    }
    return EXAMPLE_PASS;
}

#ifdef LS2K0300_EXAMPLE_RUN_ALL
typedef struct example_entry {
    const char *name;
    example_result_t (*fn)(void);
} example_entry_t;

static void run_one(const example_entry_t *entry, int *pass_cnt, int *fail_cnt, int *skip_cnt)
{
    example_result_t ret = entry->fn();

    if (ret == EXAMPLE_PASS) {
        (*pass_cnt)++;
        printf("[PASS] %s\n", entry->name);
    } else if (ret == EXAMPLE_SKIP) {
        (*skip_cnt)++;
        printf("[SKIP] %s\n", entry->name);
    } else {
        (*fail_cnt)++;
        printf("[FAIL] %s\n", entry->name);
    }
}

int main(void)
{
    int pass_cnt = 0;
    int fail_cnt = 0;
    int skip_cnt = 0;
    size_t i;
    static const example_entry_t cases[] = {
        {"signal", ls2k0300_example_signal},
        {"gpio", ls2k0300_example_gpio},
        {"adc", ls2k0300_example_adc},
        {"pwm", ls2k0300_example_pwm},
        {"uart", ls2k0300_example_uart},
        {"i2c", ls2k0300_example_i2c},
        {"spi", ls2k0300_example_spi},
        {"soft_spi", ls2k0300_example_soft_spi},
        {"atim_pwm", ls2k0300_example_atim_pwm},
        {"gtim_pwm", ls2k0300_example_gtim_pwm},
        {"encoder", ls2k0300_example_encoder},
        {"timer", ls2k0300_example_timer},
        {"canfd", ls2k0300_example_canfd},
    };

    printf("LS2K0300 example suite start\n");
    for (i = 0U; i < (sizeof(cases) / sizeof(cases[0])); i++) {
        run_one(&cases[i], &pass_cnt, &fail_cnt, &skip_cnt);
    }
    printf("Summary: pass=%d fail=%d skip=%d sigint_cb_hit=%d\n",
           pass_cnt,
           fail_cnt,
           skip_cnt,
           (int)g_sigint_cb_hit);

    return (fail_cnt == 0) ? 0 : 1;
}
#endif
