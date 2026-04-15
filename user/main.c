#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "LS2K0300_DRV_INC.h"

/* 统一的测试结果状态，用于最终汇总统计 */
typedef enum result_state {
    RESULT_PASS = 0,
    RESULT_FAIL = 1,
    RESULT_SKIP = 2
} result_state_t;

/* SIGINT 回调命中标志（当前用于演示信号注册成功） */
static volatile sig_atomic_t g_sigint_cb_hit = 0;
/* 定时器回调计数，验证 timer 线程是否正常触发 */
static volatile int g_timer_ticks = 0;

/* Ctrl+C 退出回调：只做最小工作，避免复杂逻辑 */
static void app_signal_exit_cb(void)
{
    g_sigint_cb_hit = 1;
}

/* 周期定时器回调：每次触发累加计数 */
static void app_timer_cb(void *user_data)
{
    (void)user_data;
    g_timer_ticks++;
}

/* 统一打印测试结果并更新 pass/fail/skip 计数 */
static void report_result(const char *name,
                          result_state_t state,
                          const char *detail,
                          int *pass_cnt,
                          int *fail_cnt,
                          int *skip_cnt)
{
    if (state == RESULT_PASS) {
        (*pass_cnt)++;
        printf("[PASS] %s - %s\n", name, detail);
    } else if (state == RESULT_SKIP) {
        (*skip_cnt)++;
        printf("[SKIP] %s - %s\n", name, detail);
    } else {
        (*fail_cnt)++;
        printf("[FAIL] %s - %s\n", name, detail);
    }
}

int main(void)
{
    int pass_cnt = 0;
    int fail_cnt = 0;
    int skip_cnt = 0;

    /* 启动信息 */
    printf("LS2K0300 simple smoke test start\n");

    /* signal: 验证 SIGINT 退出回调注册 */
    ls2k0300_signal_set_exit_cb(app_signal_exit_cb);
    report_result("signal", RESULT_PASS, "SIGINT callback registered", &pass_cnt, &fail_cnt, &skip_cnt);

    /* gpio: 验证 GPIO 初始化、输出控制和释放 */
    {
        ls2k0300_gpio_t gpio;
        if (ls2k0300_gpio_init(&gpio, PIN_64, GPIO_MODE_OUT, GPIO_MUX_GPIO) != 0) {
            report_result("gpio", RESULT_FAIL, "init failed", &pass_cnt, &fail_cnt, &skip_cnt);
        } else {
            ls2k0300_gpio_level_set(&gpio, GPIO_HIGH);
            ls2k0300_gpio_level_set(&gpio, GPIO_LOW);
            ls2k0300_gpio_deinit(&gpio);
            report_result("gpio", RESULT_PASS, "init/set/deinit ok", &pass_cnt, &fail_cnt, &skip_cnt);
        }
    }

    /* adc: 验证 ADC 初始化、原始值读取、电压读取和释放 */
    {
        if (ls2k0300_adc_init() != 0) {
            report_result("adc", RESULT_FAIL, "init failed", &pass_cnt, &fail_cnt, &skip_cnt);
        } else {
            int raw = ls2k0300_adc_read_raw(LS_ADC_CH0);
            float voltage = ls2k0300_adc_read_voltage(LS_ADC_CH0);
            ls2k0300_adc_deinit();

            if (raw < 0 || voltage < 0.0f) {
                report_result("adc", RESULT_FAIL, "read failed", &pass_cnt, &fail_cnt, &skip_cnt);
            } else {
                report_result("adc", RESULT_PASS, "read ok", &pass_cnt, &fail_cnt, &skip_cnt);
            }
        }
    }

    /* pwm: 验证基础 PWM 初始化、占空比设置和释放 */
    {
        ls2k0300_pwm_t pwm;
        if (ls2k0300_pwm_init(&pwm, PWM0_PIN64, 1000U, 5000U, PWM_POL_NORMAL) != 0) {
            report_result("pwm", RESULT_FAIL, "init failed", &pass_cnt, &fail_cnt, &skip_cnt);
        } else {
            ls2k0300_pwm_set_duty(&pwm, 2500U);
            ls2k0300_pwm_deinit(&pwm);
            report_result("pwm", RESULT_PASS, "init/config/deinit ok", &pass_cnt, &fail_cnt, &skip_cnt);
        }
    }

    /* uart: 验证 UART 初始化、发送与释放 */
    {
        ls2k0300_uart_t uart;
        const uint8_t msg[] = "LS2K0300 UART smoke\n";
        if (ls2k0300_uart_init(&uart, UART1, B115200, LS_UART_STOP1, LS_UART_DATA8, LS_UART_PARITY_NONE) != 0) {
            report_result("uart", RESULT_FAIL, "init failed", &pass_cnt, &fail_cnt, &skip_cnt);
        } else {
            ssize_t written = ls2k0300_uart_write(&uart, msg, sizeof(msg) - 1U);
            ls2k0300_uart_deinit(&uart);
            if (written < 0) {
                report_result("uart", RESULT_FAIL, "write failed", &pass_cnt, &fail_cnt, &skip_cnt);
            } else {
                report_result("uart", RESULT_PASS, "init/write/deinit ok", &pass_cnt, &fail_cnt, &skip_cnt);
            }
        }
    }

    /* i2c: 验证软件 I2C 初始化与释放（不依赖从设备） */
    {
        ls2k0300_i2c_t i2c;
        if (ls2k0300_i2c_init(&i2c, PIN_85, PIN_86, 0x50U) != 0) {
            report_result("i2c", RESULT_FAIL, "init failed", &pass_cnt, &fail_cnt, &skip_cnt);
        } else {
            ls2k0300_i2c_deinit(&i2c);
            report_result("i2c", RESULT_PASS, "init/deinit ok", &pass_cnt, &fail_cnt, &skip_cnt);
        }
    }

    /* spi: 验证硬件 SPI 初始化、一次收发和释放 */
    {
        ls2k0300_spi_t spi;
        uint8_t tx[2] = {0xA5U, 0x5AU};
        uint8_t rx[2] = {0U, 0U};
        if (ls2k0300_spi_init(&spi, LS_SPI2, 1000000U, LS_SPI_MODE_0) != 0) {
            report_result("spi", RESULT_FAIL, "init failed", &pass_cnt, &fail_cnt, &skip_cnt);
        } else {
            ssize_t ret = ls2k0300_spi_transfer(&spi, tx, rx, sizeof(tx));
            ls2k0300_spi_deinit(&spi);
            if (ret != 0) {
                report_result("spi", RESULT_FAIL, "transfer failed", &pass_cnt, &fail_cnt, &skip_cnt);
            } else {
                report_result("spi", RESULT_PASS, "init/transfer/deinit ok", &pass_cnt, &fail_cnt, &skip_cnt);
            }
        }
    }

    /* soft_spi: 验证软件 SPI 初始化、读取与释放 */
    {
        ls2k0300_soft_spi_t soft_spi;
        if (ls2k0300_soft_spi_init(&soft_spi, PIN_64, PIN_65, PIN_66, PIN_67, LS_SOFT_SPI_MODE_0) != 0) {
            report_result("soft_spi", RESULT_FAIL, "init failed", &pass_cnt, &fail_cnt, &skip_cnt);
        } else {
            (void)ls2k0300_soft_spi_read_byte(&soft_spi, 0x00U);
            ls2k0300_soft_spi_deinit(&soft_spi);
            report_result("soft_spi", RESULT_PASS, "init/read/deinit ok", &pass_cnt, &fail_cnt, &skip_cnt);
        }
    }

    /* atim_pwm: 验证高级定时器 PWM 初始化与释放 */
    {
        ls2k0300_atim_pwm_t atim_pwm;
        if (ls2k0300_atim_pwm_init(&atim_pwm, ATIM_PWM0_PIN81, 1000U, 5000U, ATIM_PWM_POL_NORMAL) != 0) {
            report_result("atim_pwm", RESULT_FAIL, "init failed", &pass_cnt, &fail_cnt, &skip_cnt);
        } else {
            ls2k0300_atim_pwm_deinit(&atim_pwm);
            report_result("atim_pwm", RESULT_PASS, "init/deinit ok", &pass_cnt, &fail_cnt, &skip_cnt);
        }
    }

    /* gtim_pwm: 验证通用定时器 PWM 初始化与释放 */
    {
        ls2k0300_gtim_pwm_t gtim_pwm;
        if (ls2k0300_gtim_pwm_init(&gtim_pwm, GTIM_PWM0_PIN87, 1000U, 5000U, GTIM_PWM_POL_NORMAL) != 0) {
            report_result("gtim_pwm", RESULT_FAIL, "init failed", &pass_cnt, &fail_cnt, &skip_cnt);
        } else {
            ls2k0300_gtim_pwm_deinit(&gtim_pwm);
            report_result("gtim_pwm", RESULT_PASS, "init/deinit ok", &pass_cnt, &fail_cnt, &skip_cnt);
        }
    }

    /* encoder: 验证编码器输入初始化、计数读取和释放 */
    {
        ls2k0300_pwm_encoder_t enc;
        if (ls2k0300_pwm_encoder_init(&enc, ENC_PWM0_PIN64, PIN_72) != 0) {
            report_result("encoder", RESULT_FAIL, "init failed", &pass_cnt, &fail_cnt, &skip_cnt);
        } else {
            (void)ls2k0300_pwm_encoder_get_count(&enc);
            ls2k0300_pwm_encoder_deinit(&enc);
            report_result("encoder", RESULT_PASS, "init/read/deinit ok", &pass_cnt, &fail_cnt, &skip_cnt);
        }
    }

    /* timer: 验证软件定时器回调是否周期触发 */
    {
        ls2k0300_timer_t timer;
        g_timer_ticks = 0;
        if (ls2k0300_timer_init(&timer) != 0) {
            report_result("timer", RESULT_FAIL, "init failed", &pass_cnt, &fail_cnt, &skip_cnt);
        } else {
            int ret = ls2k0300_timer_set_seconds_ms(&timer, 20U, app_timer_cb, NULL);
            usleep(120U * 1000U);
            (void)ls2k0300_timer_stop(&timer);
            ls2k0300_timer_deinit(&timer);

            if (ret != 0 || g_timer_ticks <= 0) {
                report_result("timer", RESULT_FAIL, "callback not invoked", &pass_cnt, &fail_cnt, &skip_cnt);
            } else {
                report_result("timer", RESULT_PASS, "callback invoked", &pass_cnt, &fail_cnt, &skip_cnt);
            }
        }
    }

    /* canfd: 需要 root 权限，验证初始化、发送和释放 */
    {
        if (geteuid() != 0) {
            report_result("canfd", RESULT_SKIP, "requires root", &pass_cnt, &fail_cnt, &skip_cnt);
        } else {
            ls2k0300_canfd_t canfd;
            if (ls2k0300_canfd_init(&canfd, CAN0, CANFD_MODE_THREAD, NULL, NULL) != 0) {
                report_result("canfd", RESULT_FAIL, "init failed", &pass_cnt, &fail_cnt, &skip_cnt);
            } else {
                ls2k0300_canfd_frame_t frame = {0x123U, 8U, {1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U}};
                int ret = ls2k0300_canfd_write_frame(&canfd, &frame);
                ls2k0300_canfd_deinit(&canfd);

                if (ret < 0) {
                    report_result("canfd", RESULT_FAIL, "write failed", &pass_cnt, &fail_cnt, &skip_cnt);
                } else {
                    report_result("canfd", RESULT_PASS, "init/write/deinit ok", &pass_cnt, &fail_cnt, &skip_cnt);
                }
            }
        }
    }

    /* 打印总结果并返回进程退出码 */
    printf("Summary: pass=%d, fail=%d, skip=%d\n", pass_cnt, fail_cnt, skip_cnt);
    return (fail_cnt == 0) ? 0 : 1;
}
