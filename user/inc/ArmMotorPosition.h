#ifndef ARM_MOTOR_POSITION_H
#define ARM_MOTOR_POSITION_H

#include "LS2K0300_UART.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ARM_MOTOR_POSITION_RESPONSE_SIZE 8U
#define ARM_MOTOR_POSITION_TIMEOUT_MS 200

int arm_read_motor_position_degrees(ls2k0300_uart_t *uart, uint8_t addr,
                                    double *motor_degrees,
                                    int timeout_ms);

int arm_read_joint_angles_rad(ls2k0300_uart_t *uart,
                              const uint8_t addrs[3],
                              const double gear_ratios[3],
                              const double joint_directions[3],
                              const double joint_offsets_rad[3],
                              double theta_rad[3],
                              int timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
