#include <iostream>
#include <unistd.h>
#include "LS2K0300_ADC.h"
#include "LS2K0300_CANFD.h"
#include "LS2K0300_GPIO.h"
#include "LS2K0300_I2C.h"
#include "LS2K0300_MAP.h"
#include "LS2K0300_PWM.h"
#include "LS2K0300_SPI.h"
#include "LS2K0300_UART.h"

void can_callback(const ls2k0300_canfd_frame_t* frame, void* user_data) {
    std::cout << "CAN frame received ID: " << frame->can_id << std::endl;
}

int main() {
    std::cout << "Starting LS2K0300 test..." << std::endl;
    
    // GPIO
    ls2k0300_gpio_direction_set(PIN_64, GPIO_MODE_OUT);
    ls2k0300_gpio_level_set(PIN_64, GPIO_HIGH);

    // ADC
    ls2k0300_adc_init();
    float voltage = ls2k0300_adc_read_voltage(LS_ADC_CH0);
    std::cout << "ADC CH0: " << voltage << "V" << std::endl;
    ls2k0300_adc_deinit();

    // UART
    ls2k0300_uart_t uart;
    ls2k0300_uart_init(&uart, UART_2, UART_BAUDRATE_115200);
    ls2k0300_uart_write(&uart, (const uint8_t*)"Hello", 5);
    ls2k0300_uart_deinit(&uart);

    // CANFD
    ls2k0300_canfd_t can;
    ls2k0300_canfd_init(&can, "can0", CANFD_MODE_THREAD, can_callback, nullptr);
    ls2k0300_canfd_frame_t frame = {0x123, 8, {1, 2, 3, 4, 5, 6, 7, 8}};
    ls2k0300_canfd_write_frame(&can, &frame);
    ls2k0300_canfd_deinit(&can);

    // PWM
    ls2k0300_pwm_t pwm;
    ls2k0300_pwm_init(&pwm, PWM0_PIN64, 50, 2000, PWM_POL_NORMAL);
    ls2k0300_pwm_deinit(&pwm);

    // I2C
    ls2k0300_i2c_t i2c;
    ls2k0300_i2c_init(&i2c, PIN_85, PIN_86, 0x50);
    uint8_t data;
    ls2k0300_i2c_read_byte(&i2c, 0x00, &data);

    // SPI
    ls2k0300_spi_t spi;
    ls2k0300_spi_init(&spi, LS_SPI2, 1000000, LS_SPI_MODE_0);
    uint8_t tx_buf[2] = {0x00, 0xFF};
    uint8_t rx_buf[2];
    ls2k0300_spi_transfer(&spi, tx_buf, rx_buf, 2);
    ls2k0300_spi_deinit(&spi);

    std::cout << "Test completed successfully." << std::endl;
    return 0;
}
