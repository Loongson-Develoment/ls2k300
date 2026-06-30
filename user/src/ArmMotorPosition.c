#include "ArmMotorPosition.h"

#include "X42_V2.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int64_t monotonic_time_ms(void)
{
    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        return 0;
    }

    return (int64_t)now.tv_sec * 1000LL +
           (int64_t)now.tv_nsec / 1000000LL;
}

static ssize_t read_byte_timeout(ls2k0300_uart_t *uart, uint8_t *byte,
                                 int64_t deadline_ms)
{
    if (uart == NULL || byte == NULL) {
        return -1;
    }

    while (monotonic_time_ms() < deadline_ms) {
        int64_t now_ms = monotonic_time_ms();
        ssize_t n;

        (void)now_ms;
        n = ls2k0300_uart_read(uart, byte, 1);
        if (n == 1) {
            return 1;
        }

        if (n < 0 && errno != EINTR && errno != EAGAIN &&
            errno != EWOULDBLOCK) {
            return -1;
        }

        usleep(1000);
    }

    return 0;
}

static void print_bytes(const char *prefix, uint8_t addr,
                        const uint8_t *buffer, size_t size)
{
    printf("%s addr=%u size=%zu data:", prefix, (unsigned int)addr, size);
    for (size_t i = 0U; i < size; ++i) {
        printf(" %02X", buffer[i]);
    }
    printf("\n");
}

static ssize_t read_position_response_timeout(ls2k0300_uart_t *uart,
                                              uint8_t addr,
                                              uint8_t response[ARM_MOTOR_POSITION_RESPONSE_SIZE],
                                              int timeout_ms)
{
    uint8_t debug_buffer[64];
    size_t debug_size = 0U;
    size_t frame_pos = 0U;
    int64_t deadline_ms;

    if (uart == NULL || response == NULL || timeout_ms <= 0) {
        return -1;
    }

    deadline_ms = monotonic_time_ms() + (int64_t)timeout_ms;
    while (monotonic_time_ms() < deadline_ms) {
        uint8_t byte = 0U;
        ssize_t n = read_byte_timeout(uart, &byte, deadline_ms);

        if (n < 0) {
            return -1;
        }
        if (n == 0) {
            break;
        }

        if (debug_size < sizeof(debug_buffer)) {
            debug_buffer[debug_size++] = byte;
        }

        if (frame_pos == 0U) {
            if (byte != addr) {
                continue;
            }
            response[frame_pos++] = byte;
            continue;
        }

        if (frame_pos == 1U) {
            if (byte != 0x36U) {
                frame_pos = (byte == addr) ? 1U : 0U;
                if (frame_pos == 1U) {
                    response[0] = byte;
                }
                continue;
            }
            response[frame_pos++] = byte;
            continue;
        }

        response[frame_pos++] = byte;
        if (frame_pos == ARM_MOTOR_POSITION_RESPONSE_SIZE) {
            if (response[7] == 0x6BU) {
                return (ssize_t)frame_pos;
            }

            frame_pos = 0U;
        }
    }

    print_bytes("invalid motor position rx", addr, debug_buffer, debug_size);
    return 0;
}

static double x_firmware_position_to_degrees(uint32_t raw_position,
                                             uint8_t sign)
{
    double degrees = (double)raw_position / 10.0;

    if (sign != 0U) {
        degrees = -degrees;
    }

    return degrees;
}

int arm_read_motor_position_degrees(ls2k0300_uart_t *uart, uint8_t addr,
                                    double *motor_degrees,
                                    int timeout_ms)
{
    uint8_t response[ARM_MOTOR_POSITION_RESPONSE_SIZE] = {0};
    ssize_t response_size;
    uint32_t raw_position;

    if (uart == NULL || motor_degrees == NULL) {
        return 0;
    }

    if (ls2k0300_uart_flush(uart) != 0) {
        return 0;
    }

    ZDT_X42_V2_Read_Sys_Params(uart, addr, S_CPOS);

    response_size = read_position_response_timeout(uart, addr, response,
                                                  timeout_ms);
    if (response_size != (ssize_t)sizeof(response) ||
        response[0] != addr ||
        response[1] != 0x36U ||
        response[7] != 0x6BU) {
        return 0;
    }

    raw_position = ((uint32_t)response[3] << 24) |
                   ((uint32_t)response[4] << 16) |
                   ((uint32_t)response[5] << 8) |
                   ((uint32_t)response[6] << 0);
    *motor_degrees = x_firmware_position_to_degrees(raw_position,
                                                    response[2]);
    return 1;
}

int arm_read_joint_angles_rad(ls2k0300_uart_t *uart,
                              const uint8_t addrs[3],
                              const double gear_ratios[3],
                              const double joint_directions[3],
                              const double joint_offsets_rad[3],
                              double theta_rad[3],
                              int timeout_ms)
{
    for (int i = 0; i < 3; ++i) {
        double motor_degrees;
        double motor_rad;

        if (fabs(gear_ratios[i]) < 1e-9) {
            return 0;
        }

        if (!arm_read_motor_position_degrees(uart, addrs[i],
                                             &motor_degrees, timeout_ms)) {
            return 0;
        }

        motor_rad = motor_degrees * M_PI / 180.0;
        theta_rad[i] = joint_offsets_rad[i] +
                       joint_directions[i] * motor_rad / gear_ratios[i];
    }

    return 1;
}
