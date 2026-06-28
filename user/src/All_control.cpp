#include "All_control.h"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

All_control::All_control()
    : setspeed(0.0f) //ml/s  [-0.14,0.14]
    , setposition(0.0f) //ml
    , origin_count(0)
    , position_pid(2.0, 0.0, 1.0, MAX_SPEED)
    , speed_pid(0.0, 0.0, 0.0, MAX_DUTY)
    , period_count(0)
    , zero_speed_count(0)
    , speed_sample_count(0)
    , position_error_ready(false)
    , position_cross_stopped(false)
    , position_arrived(true)
    , homing_active(false)
    , homing_finished(true)
    , zero_start_preloaded(false)
    , speed_filter_ready(false)
    , motion_direction(1)
    , last_position_error(0.0f)
    , filtered_speed(0.0f)
    , speed_pid_segment(SPEED_PID_SEG_INVALID)
    , motor()
    , uart_injection()
    , rec_buf{0}
    , sample_current{0}
    , current_average_window{0.0f}
    , current_average_sum(0.0f)
    , current_average_value(0.0f)
    , current_baseline(0.0f)
    , current_baseline_delta(0.0f)
    , current_average_index(0)
    , current_average_count(0)
    , current_error_count(0)
    , current_sample_index(0UL)
    , current_filter_ready(false)
    , current_baseline_ready(false)
    , contact(false)
    , inject_position_arrived(false)
    , injection_uart_ready(false)
    , inject_running_flag(NULL)
{
    All_control::Motor_control_init();



}

All_control::~All_control()
{
    All_control::Motor_control_deinit();
    All_control::Inject_control_deinit();



}

void All_control::Motor_control_init()
{
    Init_encoder_data(&encoder_data);
    encoder_data.current_count = Read_encoder();
    encoder_data.last_count = encoder_data.current_count;
    origin_count = encoder_data.current_count;
    setposition = 0.0f;
    position_arrived = true;
    homing_active = false;
    homing_finished = true;

    clock_gettime(CLOCK_MONOTONIC, &last_time);


}

void All_control::Set_target_delta(float delta_ml)
{
    long long current_count;
    float current_position_ml;

    if (delta_ml > POSITION_LIMIT_ML) {
        delta_ml = POSITION_LIMIT_ML;
    } else if (delta_ml < -POSITION_LIMIT_ML) {
        delta_ml = -POSITION_LIMIT_ML;
    }

    current_count = Read_encoder();
    current_position_ml = (float)((double)(current_count - origin_count) / COUNT_PER_1ML);
    setposition = current_position_ml + delta_ml;
    motion_direction = (delta_ml >= 0.0f) ? 1 : -1;

    setspeed = 0.0f;
    period_count = POSITION_LOOP_DIV;
    position_error_ready = false;
    position_cross_stopped = false;
    position_arrived = (std::fabs(delta_ml) <= DEAD_POS);
    homing_active = false;
    homing_finished = true;
    zero_start_preloaded = false;
    speed_pid_segment = SPEED_PID_SEG_INVALID;
    position_pid.Set_output(0.0f);
    speed_pid.Set_output(0.0f);
}

void All_control::Trigger_home(void)
{
    long long current_count = Read_encoder();
    long long saved_count = Load_encoder();
    long long deadzone_count = (long long)(DEAD_POS * COUNT_PER_1ML);
    long long abs_saved_count;
    float current_position_ml;

    if (current_count == -1) {
        return;
    }

    abs_saved_count = (saved_count >= 0) ? saved_count : -saved_count;

    if (saved_count != -1 && abs_saved_count > deadzone_count) {
        origin_count = current_count - saved_count;
        current_position_ml = (float)((double)saved_count / COUNT_PER_1ML);
    } else {
        origin_count = current_count;
        current_position_ml = 0.0f;
    }

    setposition = 0.0f;
    motion_direction = (current_position_ml <= 0.0f) ? 1 : -1;

    setspeed = 0.0f;
    period_count = POSITION_LOOP_DIV;
    position_error_ready = false;
    position_cross_stopped = false;
    position_arrived = (std::fabs(current_position_ml) <= DEAD_POS);
    homing_active = !position_arrived;
    homing_finished = position_arrived;
    zero_start_preloaded = false;
    speed_pid_segment = SPEED_PID_SEG_INVALID;
    position_pid.Set_output(0.0f);
    speed_pid.Set_output(0.0f);
}

void All_control::Save_current_position(void)
{
    long long current_count = Read_encoder();

    if (current_count != -1) {
        Save_encoder(current_count - origin_count);
    }
}

void All_control::Set_current_as_zero(void)
{
    motor.Motor_output(0);
    setspeed = 0.0f;
    setposition = 0.0f;
    origin_count = 0;
    period_count = POSITION_LOOP_DIV;
    position_error_ready = false;
    position_cross_stopped = false;
    position_arrived = true;
    homing_active = false;
    homing_finished = true;
    zero_start_preloaded = false;
    speed_pid_segment = SPEED_PID_SEG_INVALID;
    filtered_speed = 0.0f;
    speed_filter_ready = false;
    position_pid.Set_output(0.0f);
    speed_pid.Set_output(0.0f);

    if (Reset_encoder() == 0) {
        encoder_data.current_count = 0;
        encoder_data.last_count = 0;
        encoder_data.diff_count = 0;
    } else {
        encoder_data.current_count = Read_encoder();
        encoder_data.last_count = encoder_data.current_count;
        encoder_data.diff_count = 0;
    }

    Save_encoder(0);
}

bool All_control::Is_arrived(void) const
{
    return position_arrived;
}

bool All_control::Is_home_finished(void) const
{
    return homing_finished;
}

bool All_control::Is_contact(void) const
{
    return contact;
}

bool All_control::Is_inject_position_arrived(void) const
{
    return inject_position_arrived;
}

void All_control::Inject_set_running_flag(const volatile sig_atomic_t *running_flag)
{
    inject_running_flag = running_flag;
}

void All_control::Motor_control(void)
{
    timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double dt = timespec_diff_sec(&now, &last_time);
    last_time = now;
    if (dt > 0.0) {
        Get_encoder_data(&encoder_data);

        float current_position_ml =
            (float)((double)(encoder_data.current_count - origin_count) / COUNT_PER_1ML);
        float raw_speed = (float)((double)encoder_data.diff_count / COUNT_PER_1ML / dt);

        if (std::fabs(raw_speed) <= ZERO_SPEED_EPS) {
            if (zero_speed_count < ZERO_SPEED_CYCLES) {
                zero_speed_count++;
            }
        } else {
            zero_speed_count = 0;
            zero_start_preloaded = false;
        }

        sample_speed[2] = sample_speed[1];
        sample_speed[1] = sample_speed[0];
        sample_speed[0] = raw_speed;
        if (speed_sample_count < 3) {
            speed_sample_count++;
        }

        float median_speed = (speed_sample_count < 3) ? raw_speed :
                             median3(sample_speed[0], sample_speed[1], sample_speed[2]);
        if (!speed_filter_ready) {
            filtered_speed = median_speed;
            speed_filter_ready = true;
        } else {
            filtered_speed = low_pass_speed(median_speed, filtered_speed, LOW_PASS);
        }

        bool motor_idle = false;
        float raw_target_speed = setspeed;

        /* 位置环低频运行，输出作为速度环的目标速度。 */
        if (++period_count >= POSITION_LOOP_DIV) {
            period_count = 0;
            raw_target_speed = position_pid.calculate(setposition, current_position_ml);

            /* 位置环输出限幅，保证速度目标不超过机械允许速度。 */
            if (raw_target_speed > MAX_SPEED) {
                raw_target_speed = MAX_SPEED;
            } else if (raw_target_speed < -MAX_SPEED) {
                raw_target_speed = -MAX_SPEED;
            }

            /* 非零小速度提升到最小可控速度，避免落在电机静摩擦死区里。 */
            if (std::fabs(raw_target_speed) > 0.0f &&
                std::fabs(raw_target_speed) < MIN_TARGET_SPEED) {
                raw_target_speed = (raw_target_speed > 0.0f) ?
                                   MIN_TARGET_SPEED : -MIN_TARGET_SPEED;
            }
        }

        /* 处理目标前死区、越过目标停止，以及禁止反向运动。 */
        motor_idle = Handle_deadzone_and_direction(current_position_ml,
                                                   raw_target_speed);
        if (homing_active && position_arrived) {
            homing_active = false;
            homing_finished = true;
        }

        /* 目标速度接近 0 时不进入速度环，避免速度环主动反向刹车。 */
        if (std::fabs(setspeed) <= TARGET_SPEED_EPS) {
            motor_idle = true;
        }

        int res_duty = 0;

        if (motor_idle) {
            /* 停机状态下清掉速度环内部输出和低速预置标志。 */
            speed_pid.Set_output(0.0f);
            zero_start_preloaded = false;
        } else {
            /* 根据目标速度大小切换速度环 PID 参数。 */
            Set_speed_pid_segment(setspeed);

            /* 低速且连续静止时，预置一个启动占空比帮助克服静摩擦。 */
            if (std::fabs(setspeed) > TARGET_SPEED_EPS &&
                std::fabs(setspeed) < LOW_SPEED_PRELOAD_LIMIT &&
                zero_speed_count >= ZERO_SPEED_CYCLES &&
                !zero_start_preloaded) {
                int preload_sign = (setspeed >= 0.0f) ? 1 : -1;
                speed_pid.Set_output(preload_sign * LOW_SPEED_PRELOAD_DUTY);
                zero_start_preloaded = true;
            }

            res_duty = (int)(speed_pid.incremental_calculate(setspeed, filtered_speed));
            /* 最终输出前再做一次方向保护，禁止速度环给反向占空比。 */
            if (std::fabs((float)res_duty) > TARGET_SPEED_EPS &&
                res_duty * motion_direction < 0) {
                res_duty = 0;
                speed_pid.Set_output(0.0f);
                zero_start_preloaded = false;
                motor_idle = true;
            }
        }

        // printf("pos %.4f/%.4f speed %.4f/%.4f duty=%d\n",
        //        current_position_ml, setposition, filtered_speed, setspeed, res_duty);

        motor.Motor_output(motor_idle ? 0 : res_duty);


    }

}

void All_control::Motor_control_deinit()
{
    motor.Motor_output(0);

}

double All_control::timespec_diff_sec(const timespec *now, const timespec *last)
{
    return (double)(now->tv_sec - last->tv_sec) +
           (double)(now->tv_nsec - last->tv_nsec) / 1e9;
}


float All_control::median3(float a, float b, float c)
{
    if (a > b) {
        float t = a; a = b; b = t;
    }
    if (b > c) {
        float t = b; b = c; c = t;
    }
    if (a > b) {
        float t = a; a = b; b = t;
    }
    return b;
}


float All_control::low_pass_speed(float input, float last_output, float delta)
{

    if (delta < 0.0f) {
        delta = 0.0f;
    } else if (delta > 1.0f) {
        delta = 1.0f;
    }

    return last_output + delta * (input - last_output);
}

bool All_control::Handle_deadzone_and_direction(float current_position_ml,
                                                float raw_target_speed)
{
    float position_error = setposition - current_position_ml;

    if (!position_error_ready) {
        last_position_error = position_error;
        motion_direction = (position_error >= 0.0f) ? 1 : -1;
        position_error_ready = true;
    }

    bool passed_target = position_error * (float)motion_direction < -POSITION_ERROR_EPS;
    bool crossed_target = std::fabs(last_position_error) > POSITION_ERROR_EPS &&
                          std::fabs(position_error) > POSITION_ERROR_EPS &&
                          last_position_error * position_error < 0.0f;
    bool priority_deadzone = position_error * (float)motion_direction > 0.0f &&
                             std::fabs(position_error) < DEAD_POS;
    bool position_reached = std::fabs(position_error) <= POSITION_ERROR_EPS;
    bool reverse_speed = std::fabs(raw_target_speed) > TARGET_SPEED_EPS &&
                         raw_target_speed * (float)motion_direction < 0.0f;

    if (position_reached || priority_deadzone || position_cross_stopped ||
        passed_target || crossed_target || reverse_speed) {
        setspeed = 0.0f;
        position_pid.Set_output(0.0f);
        if (position_reached || priority_deadzone ||
            passed_target || crossed_target || position_cross_stopped) {
            position_arrived = true;
        }
        if (passed_target || crossed_target) {
            position_cross_stopped = true;
        }
        last_position_error = position_error;
        return true;
    }

    setspeed = raw_target_speed;
    last_position_error = position_error;
    return false;
}

void All_control::Set_speed_pid_segment(float target_speed)
{
    speed_pid_segment_t next_segment;
    float abs_speed = std::fabs(target_speed);
    float kp;
    float ki;

    if (abs_speed < SPEED_PID_LOW_LIMIT) {
        next_segment = SPEED_PID_SEG_LOW;
    } else if (abs_speed < SPEED_PID_MID_LIMIT) {
        next_segment = SPEED_PID_SEG_MID;
    } else {
        next_segment = SPEED_PID_SEG_HIGH;
    }

    if (next_segment == speed_pid_segment) {
        return;
    }

    switch (next_segment) {
    case SPEED_PID_SEG_LOW:
        kp = SPEED_PID_LOW_KP;
        ki = SPEED_PID_LOW_KI;
        break;
    case SPEED_PID_SEG_MID:
        kp = SPEED_PID_MID_KP;
        ki = SPEED_PID_DEFAULT_KI;
        break;
    case SPEED_PID_SEG_HIGH:
    default:
        kp = SPEED_PID_HIGH_KP;
        ki = SPEED_PID_DEFAULT_KI;
        break;
    }

    speed_pid.Set_config(kp, ki, 0.0f);
    speed_pid_segment = next_segment;
}


bool All_control::Ensure_inject_uart_ready(void)
{
    if (injection_uart_ready) {
        return true;
    }

    return Inject_control_init();
}

bool All_control::Inject_control_init(void)
{
    if (ls2k0300_uart_block_init(&uart_injection, UART_INJECTION_PIN, B115200,
                                 LS_UART_STOP1, LS_UART_DATA8,
                                 LS_UART_PARITY_NONE, 1U) != 0) {
        injection_uart_ready = false;
        return false;
    }

    memset(&rec_buf, 0, sizeof(rec_buf));
    Inject_reset_contact_state();
    injection_uart_ready = true;

    return true;

}

void All_control::Inject_control_deinit(void)
{
    if (injection_uart_ready) {
        ZDT_X42_V2_Velocity_Control(&uart_injection, INJECTION_MOTOR_ADD,
                                    0U, INJECT_VELOCITY_RAMP_RPM_PER_S,
                                    0.0f, 0U);
        ls2k0300_uart_deinit(&uart_injection);
        injection_uart_ready = false;
    }


}

void All_control::Inject_sleep_ms(long milliseconds) const
{
    timespec delay;

    delay.tv_sec = milliseconds / 1000L;
    delay.tv_nsec = (milliseconds % 1000L) * 1000000L;

    while (Inject_should_continue() &&
           nanosleep(&delay, &delay) != 0 &&
           errno == EINTR) {
    }
}

bool All_control::Inject_should_continue(void) const
{
    return inject_running_flag == NULL || *inject_running_flag != 0;
}

int64_t All_control::Inject_monotonic_time_ms(void) const
{
    timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        return 0;
    }

    return (int64_t)now.tv_sec * 1000LL +
           (int64_t)now.tv_nsec / 1000000LL;
}

void All_control::Inject_print_bytes(const char *prefix, const uint8_t *data,
                                     ssize_t size) const
{
    printf("%s size=%zd, data:", prefix, size);
    if (size > 0) {
        for (ssize_t i = 0; i < size; ++i) {
            printf(" %02X", data[i]);
        }
    }
    printf("\n");
}

ssize_t All_control::Inject_read_exact_timeout(uint8_t *buffer, size_t size,
                                               int timeout_ms)
{
    size_t received = 0U;
    int64_t deadline_ms = Inject_monotonic_time_ms() + (int64_t)timeout_ms;
    pollfd pfd;

    if (!injection_uart_ready) {
        return -1;
    }

    pfd.fd = uart_injection.fd;
    pfd.events = POLLIN;
    pfd.revents = 0;

    while (Inject_should_continue() && received < size) {
        int remaining_ms = (int)(deadline_ms - Inject_monotonic_time_ms());
        int poll_result;
        ssize_t read_size;

        if (remaining_ms <= 0) {
            break;
        }

        pfd.revents = 0;
        poll_result = poll(&pfd, 1, remaining_ms);
        if (poll_result == 0) {
            break;
        }
        if (poll_result < 0) {
            if (errno == EINTR) {
                if (!Inject_should_continue()) {
                    return -1;
                }
                continue;
            }
            perror("poll injection uart");
            return -1;
        }
        if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            return (received > 0U) ? (ssize_t)received : -1;
        }
        if ((pfd.revents & POLLIN) == 0) {
            continue;
        }

        read_size = ls2k0300_uart_read(&uart_injection, buffer + received,
                                       size - received);
        if (read_size > 0) {
            received += (size_t)read_size;
            continue;
        }
        if (read_size == 0 || errno == EINTR) {
            if (!Inject_should_continue()) {
                return -1;
            }
            continue;
        }

        perror("injection uart read");
        return -1;
    }

    return Inject_should_continue() ? (ssize_t)received : -1;
}

void All_control::Inject_reset_contact_state(void)
{
    memset(&sample_current, 0, sizeof(sample_current));
    memset(&current_average_window, 0, sizeof(current_average_window));
    current_average_sum = 0.0f;
    current_average_value = 0.0f;
    current_baseline = 0.0f;
    current_baseline_delta = 0.0f;
    current_average_index = 0;
    current_average_count = 0;
    current_error_count = 0;
    current_sample_index = 0UL;
    current_filter_ready = false;
    current_baseline_ready = false;
    contact = false;
}

float All_control::Inject_update_moving_average(float sample)
{
    if (current_average_count < INJECT_MOVING_AVERAGE_SIZE) {
        current_average_window[current_average_index] = sample;
        current_average_sum += sample;
        current_average_count++;
    } else {
        current_average_sum -= current_average_window[current_average_index];
        current_average_window[current_average_index] = sample;
        current_average_sum += sample;
    }

    current_average_index =
        (uint16_t)((current_average_index + 1U) % INJECT_MOVING_AVERAGE_SIZE);
    current_average_value =
        current_average_sum / (float)current_average_count;

    return current_average_value;
}

bool All_control::Inject_origin_status_done(uint8_t status) const
{
    return ((status & INJECT_ORIGIN_STATE_MASK) == INJECT_ORIGIN_STATE_DONE) &&
           ((status & INJECT_ORIGIN_STATUS_READY_MASK) ==
            INJECT_ORIGIN_STATUS_READY);
}

bool All_control::Inject_read_origin_status(uint8_t *status)
{
    uint8_t response[INJECT_COMMAND_RESPONSE_SIZE] = {0};
    ssize_t response_size;

    if (!Ensure_inject_uart_ready()) {
        return false;
    }
    if (tcflush(uart_injection.fd, TCIFLUSH) != 0) {
        perror("tcflush injection input");
        return false;
    }

    ZDT_X42_V2_Read_Sys_Params(&uart_injection, INJECTION_MOTOR_ADD, S_OFLAG);
    response_size = Inject_read_exact_timeout(response, sizeof(response),
                                              INJECT_STATUS_READ_TIMEOUT_MS);
    if (response_size != (ssize_t)sizeof(response) ||
        response[0] != INJECTION_MOTOR_ADD ||
        response[1] != INJECT_ORIGIN_STATUS_CODE ||
        response[3] != INJECT_CHECK_BYTE) {
        Inject_print_bytes("invalid origin status rx", response,
                           response_size);
        return false;
    }

    *status = response[2];
    return true;
}

bool All_control::Inject_read_bus_current(uint16_t *current)
{
    uint8_t response[INJECT_CURRENT_RESPONSE_SIZE] = {0};
    ssize_t response_size;

    if (!Ensure_inject_uart_ready()) {
        return false;
    }
    if (tcflush(uart_injection.fd, TCIFLUSH) != 0) {
        perror("tcflush injection input");
        return false;
    }

    ZDT_X42_V2_Read_Sys_Params(&uart_injection, INJECTION_MOTOR_ADD, S_CBUS);
    response_size = Inject_read_exact_timeout(response, sizeof(response),
                                              INJECT_CURRENT_READ_TIMEOUT_MS);
    if (response_size != (ssize_t)sizeof(response)) {
        if (response_size > 0) {
            Inject_print_bytes("short current rx", response, response_size);
        }
        return false;
    }

    if (response[0] != INJECTION_MOTOR_ADD ||
        response[1] != INJECT_READ_BUS_CURRENT_CODE ||
        response[4] != INJECT_CHECK_BYTE) {
        Inject_print_bytes("invalid current rx", response, response_size);
        return false;
    }

    *current = (uint16_t)(((uint16_t)response[2] << 8) | response[3]);
    return true;
}

bool All_control::Inject_read_motor_status(uint8_t *status)
{
    uint8_t response[INJECT_COMMAND_RESPONSE_SIZE] = {0};
    ssize_t response_size;

    if (!Ensure_inject_uart_ready()) {
        return false;
    }
    if (tcflush(uart_injection.fd, TCIFLUSH) != 0) {
        perror("tcflush injection input");
        return false;
    }

    ZDT_X42_V2_Read_Sys_Params(&uart_injection, INJECTION_MOTOR_ADD, S_SFLAG);
    response_size = Inject_read_exact_timeout(response, sizeof(response),
                                              INJECT_STATUS_READ_TIMEOUT_MS);
    if (response_size != (ssize_t)sizeof(response) ||
        response[0] != INJECTION_MOTOR_ADD ||
        response[1] != INJECT_MOTOR_STATUS_CODE ||
        response[3] != INJECT_CHECK_BYTE) {
        Inject_print_bytes("invalid motor status rx", response,
                           response_size);
        return false;
    }

    *status = response[2];
    return true;
}

bool All_control::Inject_wait_position_ack(uint8_t *command_status)
{
    uint8_t response[INJECT_COMMAND_RESPONSE_SIZE] = {0};
    ssize_t response_size;

    *command_status = 0U;
    response_size = Inject_read_exact_timeout(response, sizeof(response),
                                              INJECT_POSITION_RESPONSE_TIMEOUT_MS);
    if (response_size != (ssize_t)sizeof(response)) {
        return true;
    }

    Inject_print_bytes("position rx", response, response_size);
    if (response[0] != INJECTION_MOTOR_ADD ||
        response[1] != INJECT_POSITION_CODE ||
        response[3] != INJECT_CHECK_BYTE) {
        return true;
    }

    *command_status = response[2];
    if (response[2] == 0xE2U || response[2] == 0xEEU) {
        printf("position command failed, status=0x%02X\n", response[2]);
        return false;
    }

    return true;
}

bool All_control::Inject_wait_position_arrived(void)
{
    uint8_t stable_arrived_count = 0U;
    int64_t start_ms = Inject_monotonic_time_ms();

    while (Inject_should_continue()) {
        uint8_t status;

        if (Inject_monotonic_time_ms() - start_ms >
            INJECT_POSITION_WAIT_TIMEOUT_MS) {
            printf("position arrive timeout\n");
            return false;
        }

        if (!Inject_read_motor_status(&status)) {
            Inject_sleep_ms(INJECT_POSITION_STATUS_POLL_MS);
            continue;
        }

        printf("motor status=0x%02X\n", status);
        if ((status & INJECT_POSITION_ARRIVED_MASK) != 0U) {
            stable_arrived_count++;
            if (stable_arrived_count >=
                INJECT_POSITION_STABLE_DONE_POLLS) {
                inject_position_arrived = true;
                printf("inject position arrived\n");
                return true;
            }
        } else {
            stable_arrived_count = 0U;
        }

        Inject_sleep_ms(INJECT_POSITION_STATUS_POLL_MS);
    }

    return false;
}

bool All_control::Inject_set_velocity(uint8_t dir, float speed_rpm)
{
    if (!Ensure_inject_uart_ready()) {
        printf("error for init injection uart\n");
        return false;
    }

    ZDT_X42_V2_Velocity_Control(&uart_injection, INJECTION_MOTOR_ADD, dir,
                                INJECT_VELOCITY_RAMP_RPM_PER_S, speed_rpm,
                                0U);
    Inject_sleep_ms(200L);

    if (tcflush(uart_injection.fd, TCIFLUSH) != 0) {
        perror("tcflush injection input");
        return false;
    }

    return true;
}


void All_control::Inject_control(void)
{
    uint16_t current;
    float filtered_current;
    float moving_average;

    if (!Ensure_inject_uart_ready()) {
        printf("error for init injection uart\n");
        contact = false;
        return;
    }

    if (!Inject_read_bus_current(&current)) {
        current_error_count++;
        if (current_error_count >= INJECT_CURRENT_MAX_CONSECUTIVE_ERRORS) {
            contact = false;
        }
        return;
    }
    current_error_count = 0U;

    if (current_filter_ready) {
        filtered_current = 0.5f * (float)current +
                           0.5f * (float)sample_current[1];
    } else {
        filtered_current = (float)current;
        current_filter_ready = true;
    }
    sample_current[1] = current;

    moving_average = Inject_update_moving_average(filtered_current);
    if (!current_baseline_ready &&
        current_average_count >= INJECT_MOVING_AVERAGE_SIZE) {
        current_baseline = moving_average;
        current_baseline_ready = true;
    }

    if (current_baseline_ready) {
        current_baseline_delta = moving_average - current_baseline;
        contact = current_baseline_delta > INJECT_CONTACT_DELTA_THRESHOLD;
    }
}


void All_control::Inject_set_speed(uint8_t dir, uint16_t speed)
{
    (void)Inject_set_velocity(dir, (float)speed);
}

bool All_control::Inject_home_to_zero(void)
{
    uint8_t stable_done_count = 0U;
    int64_t start_ms;

    if (!Ensure_inject_uart_ready()) {
        printf("error for init injection uart\n");
        return false;
    }
    if (tcflush(uart_injection.fd, TCIFLUSH) != 0) {
        perror("tcflush injection input");
        return false;
    }

    printf("home: send 9A mode 04\n");
    ZDT_X42_V2_Origin_Trigger_Return(&uart_injection, INJECTION_MOTOR_ADD,
                                     INJECT_ORIGIN_MODE_ABSOLUTE_ZERO, false);

    Inject_sleep_ms(INJECT_ORIGIN_FIRST_POLL_DELAY_MS);
    start_ms = Inject_monotonic_time_ms();

    while (Inject_should_continue()) {
        uint8_t status;
        uint8_t origin_state;

        if (Inject_monotonic_time_ms() - start_ms >
            INJECT_ORIGIN_WAIT_TIMEOUT_MS) {
            printf("home timeout\n");
            return false;
        }

        if (!Inject_read_origin_status(&status)) {
            Inject_sleep_ms(INJECT_ORIGIN_STATUS_POLL_MS);
            continue;
        }

        origin_state = (uint8_t)(status & INJECT_ORIGIN_STATE_MASK);
        printf("origin status=0x%02X\n", status);

        if (origin_state == INJECT_ORIGIN_STATE_BUSY) {
            stable_done_count = 0U;
        } else if (origin_state == INJECT_ORIGIN_STATE_FAILED) {
            printf("home failed, origin status=0x%02X\n", status);
            return false;
        } else if (Inject_origin_status_done(status)) {
            stable_done_count++;
            if (stable_done_count >= INJECT_ORIGIN_STABLE_DONE_POLLS) {
                printf("home done\n");
                return true;
            }
        } else {
            stable_done_count = 0U;
        }

        Inject_sleep_ms(INJECT_ORIGIN_STATUS_POLL_MS);
    }

    return false;
}

bool All_control::Inject_run_until_contact(uint8_t dir, float speed_rpm)
{
    Inject_reset_contact_state();

    if (!Inject_set_velocity(dir, speed_rpm)) {
        return false;
    }

    while (Inject_should_continue()) {
        uint16_t current;
        float filtered_current;
        float moving_average;

        if (!Inject_read_bus_current(&current)) {
            current_error_count++;
            if (current_error_count >= INJECT_CURRENT_MAX_CONSECUTIVE_ERRORS) {
                printf("too many current read errors\n");
                Inject_stop(dir);
                return false;
            }
            Inject_sleep_ms(INJECT_QUERY_PERIOD_MS);
            continue;
        }
        current_error_count = 0U;

        if (current_filter_ready) {
            filtered_current = 0.5f * (float)current +
                               0.5f * (float)sample_current[1];
        } else {
            filtered_current = (float)current;
            current_filter_ready = true;
        }
        sample_current[1] = current;

        moving_average = Inject_update_moving_average(filtered_current);
        if (!current_baseline_ready &&
            current_average_count >= INJECT_MOVING_AVERAGE_SIZE) {
            current_baseline = moving_average;
            current_baseline_ready = true;
            printf("baseline current=%.3f\n", current_baseline);
        }
        if (current_baseline_ready) {
            current_baseline_delta = moving_average - current_baseline;
        }

        if (current_baseline_ready &&
            current_baseline_delta > INJECT_CONTACT_DELTA_THRESHOLD) {
            contact = true;
            printf("contact detected: baseline_delta=%.3f > %.3f\n",
                   current_baseline_delta, INJECT_CONTACT_DELTA_THRESHOLD);
            Inject_stop(dir);
            return true;
        }

        if ((current_sample_index % INJECT_STATUS_PRINT_INTERVAL) == 0UL) {
            printf("sample[%lu]: raw=%u, filtered=%.1f, average=%.3f, "
                   "baseline_delta=%.3f, contact=%u%s\n",
                   current_sample_index, (unsigned int)current,
                   filtered_current, moving_average, current_baseline_delta,
                   contact ? 1U : 0U,
                   current_baseline_ready ? "" : " (baseline pending)");
        }

        current_sample_index++;
        Inject_sleep_ms(INJECT_QUERY_PERIOD_MS);
    }

    Inject_stop(dir);
    return false;
}

bool All_control::Inject_move_relative(int32_t relative_units,
                                       uint8_t default_direction,
                                       float speed_rpm)
{
    uint8_t direction = default_direction;
    uint8_t command_status = 0U;
    int64_t signed_units = (int64_t)relative_units;
    uint32_t abs_units;
    float position_degrees;

    if (!Ensure_inject_uart_ready()) {
        printf("error for init injection uart\n");
        return false;
    }

    if (signed_units < 0) {
        direction = (uint8_t)(default_direction == 0U ? 1U : 0U);
        abs_units = (uint32_t)(-signed_units);
    } else {
        abs_units = (uint32_t)signed_units;
    }

    inject_position_arrived = false;
    position_degrees = (float)abs_units / 10.0f;

    if (tcflush(uart_injection.fd, TCIFLUSH) != 0) {
        perror("tcflush injection input");
        return false;
    }

    printf("relative position: units=%ld, dir=%u\n",
           (long)relative_units, (unsigned int)direction);
    ZDT_X42_V2_Bypass_Position_LV_Control(&uart_injection,
                                          INJECTION_MOTOR_ADD,
                                          direction,
                                          speed_rpm,
                                          position_degrees,
                                          INJECT_POSITION_MODE_REL_CURRENT,
                                          0U);

    if (!Inject_wait_position_ack(&command_status)) {
        return false;
    }
    if (command_status == 0x9FU) {
        inject_position_arrived = true;
        printf("inject position arrived\n");
        return true;
    }

    return Inject_wait_position_arrived();
}

void All_control::Inject_stop(uint8_t dir)
{
    if (!Ensure_inject_uart_ready()) {
        return;
    }

    ZDT_X42_V2_Velocity_Control(&uart_injection, INJECTION_MOTOR_ADD, dir,
                                INJECT_VELOCITY_RAMP_RPM_PER_S, 0.0f, 0U);
    Inject_sleep_ms(50L);
    (void)tcflush(uart_injection.fd, TCIFLUSH);
}

float All_control::Inject_current_average(void) const
{
    return current_average_value;
}

float All_control::Inject_current_baseline_delta(void) const
{
    return current_baseline_delta;
}
