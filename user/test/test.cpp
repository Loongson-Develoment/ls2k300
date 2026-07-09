#define USE_ALL_CONTROL_TEST 1

#if USE_ALL_CONTROL_TEST

#include "All_control.h"

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <poll.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>

#define CONTROL_PERIOD_NS (33 * 1000 * 1000)

static volatile sig_atomic_t running = 1;
static const timespec period = {0, CONTROL_PERIOD_NS};

static void signal_handler(int signum)
{
    (void)signum;
    running = 0;
}

static void *control_thread(void *arg)
{
    All_control *control = (All_control *)arg;

    while (running) {
        control->Motor_control();
        nanosleep(&period, NULL);
    }

    return NULL;
}

static void handle_position_delta_input(pollfd *input_fd,
                                        All_control *control,
                                        bool position_input_enabled)
{
    char line[64];
    float position_delta;
    int poll_ret;

    if (input_fd->fd < 0) {
        return;
    }

    poll_ret = poll(input_fd, 1, 0);
    if (poll_ret <= 0) {
        if (poll_ret < 0 && errno != EINTR) {
            perror("poll stdin");
            input_fd->fd = -1;
        }
        return;
    }

    if (input_fd->revents & (POLLERR | POLLNVAL)) {
        input_fd->fd = -1;
        return;
    }

    if (!(input_fd->revents & POLLIN)) {
        return;
    }

    if (fgets(line, sizeof(line), stdin) == NULL) {
        input_fd->fd = -1;
        return;
    }

    if (line[0] == 'q' || line[0] == 'Q') {
        running = 0;
        return;
    }

    if (sscanf(line, "%f", &position_delta) == 1) {
        if (!position_input_enabled) {
            printf("homing, ignore position delta=%.4f ml\n", position_delta);
            printf("wait home finished, then enter position delta in ml, q to quit\n> ");
            fflush(stdout);
            return;
        }

        control->Set_target_delta(position_delta);
        printf("position delta=%.4f ml\n", position_delta);
        printf("> ");
        fflush(stdout);
    }
}

int main()
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    All_control control;
    control.Trigger_home();
    printf("homing...\n");

    pthread_t ctrl_thread;
    if (pthread_create(&ctrl_thread, NULL, control_thread, &control) != 0) {
        perror("pthread_create control_thread");
        control.Save_current_position();
        return 1;
    }

    pollfd input_fd;
    input_fd.fd = STDIN_FILENO;
    input_fd.events = POLLIN;
    input_fd.revents = 0;

    bool home_zeroed = false;

    while (running) {
        handle_position_delta_input(&input_fd, &control, home_zeroed);

        if (!home_zeroed && control.Is_home_finished()) {
            control.Set_current_as_zero();
            home_zeroed = true;
            printf("home finished, set current as zero\n");
            printf("enter position delta in ml, q to quit\n> ");
            fflush(stdout);
        }

        nanosleep(&period, NULL);
    }

    pthread_join(ctrl_thread, NULL);
    control.Save_current_position();
    return 0;
}

#else

#include "Encoder.h"
#include "Pid.h"
#include "LS2K0300_DRV_INC.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <netinet/in.h>
#include <poll.h>
#include <pthread.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define COUNT_PER_1ML             70000.0f
#define DEFAULT_POSITION_DELTA_ML 1.0f
#define POSITION_PID_KP           2.0f
#define POSITION_PID_KD           1.0f
#define MAX_SPEED_MLPS            0.14f
#define MAX_DUTY                  10000.0f
#define CONTROL_PERIOD_NS         (33 * 1000 * 1000)
#define RETARGET_HOLD_CYCLES      5U

#define UDP_SERVER_IP             "192.168.214.246"
#define UDP_SERVER_PORT           5005

#define SPEED_FILTER_TAU_SEC      0.06f
#define SPEED_PID_LOW_LIMIT       0.01f
#define SPEED_PID_MID_LIMIT       0.10f
#define SPEED_PID_LOW_KP          68000.0f
#define SPEED_PID_MID_KP          62000.0f
#define SPEED_PID_HIGH_KP         38000.0f
#define SPEED_PID_LOW_KI          7000.0f
#define SPEED_PID_DEFAULT_KI      7000.0f

#define MIN_TARGET_SPEED          0.005f
#define LOW_SPEED_PRELOAD_LIMIT   0.05f
#define LOW_SPEED_PRELOAD_DUTY    4000.0f
#define POSITION_PRIORITY_DEADZONE_ML 0.0015f
#define POSITION_ERROR_EPS        0.0001f
#define TARGET_SPEED_EPS          0.0001f
#define ZERO_SPEED_EPS            0.0001f
#define ZERO_SPEED_CYCLES         3U

static volatile sig_atomic_t running = 1;
static const timespec period = {0, CONTROL_PERIOD_NS};

static ls2k0300_gpio_t IN1, IN2;
static ls2k0300_gtim_pwm_t ENA;

static Pid position_pid(POSITION_PID_KP, 0.0f, POSITION_PID_KD, MAX_SPEED_MLPS);
static Pid speed_pid(0.0f, 0.0f, 0.0f, MAX_DUTY);

static pthread_mutex_t position_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t position_state_mutex = PTHREAD_MUTEX_INITIALIZER;
static long long origin_count = 0;
static long long saved_position_count = 0;
static bool saved_position_valid = false;
static bool position_control_enabled = false;
static float target_position_ml = 0.0f;
static float target_position_delta_ml = DEFAULT_POSITION_DELTA_ML;
static uint8_t retarget_hold_cycles = 0;
static bool retarget_after_hold = false;
static float udp_target_position_ml = 0.0f;
static float udp_current_position_ml = 0.0f;
static bool udp_position_valid = false;

typedef enum speed_pid_segment {
    SPEED_PID_SEG_LOW = 0,
    SPEED_PID_SEG_MID,
    SPEED_PID_SEG_HIGH,
    SPEED_PID_SEG_INVALID,
} speed_pid_segment_t;

static double timespec_diff_sec(const timespec *now, const timespec *last)
{
    return (double)(now->tv_sec - last->tv_sec) +
           (double)(now->tv_nsec - last->tv_nsec) / 1e9;
}

static float median3(float a, float b, float c)
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

static float low_pass_speed(float input, float last_output, float dt)
{
    float alpha = dt / (SPEED_FILTER_TAU_SEC + dt);

    if (alpha < 0.0f) {
        alpha = 0.0f;
    } else if (alpha > 1.0f) {
        alpha = 1.0f;
    }

    return last_output + alpha * (input - last_output);
}

static float clamp_speed(float speed)
{
    float abs_speed;

    if (speed > MAX_SPEED_MLPS) {
        return MAX_SPEED_MLPS;
    }
    if (speed < -MAX_SPEED_MLPS) {
        return -MAX_SPEED_MLPS;
    }

    abs_speed = std::fabs(speed);
    if (abs_speed > 0.0f && abs_speed < MIN_TARGET_SPEED) {
        return (speed > 0.0f) ? MIN_TARGET_SPEED : -MIN_TARGET_SPEED;
    }

    return speed;
}

static speed_pid_segment_t select_speed_pid_segment(float speed)
{
    float abs_speed = std::fabs(speed);

    if (abs_speed < SPEED_PID_LOW_LIMIT) {
        return SPEED_PID_SEG_LOW;
    }
    if (abs_speed < SPEED_PID_MID_LIMIT) {
        return SPEED_PID_SEG_MID;
    }
    return SPEED_PID_SEG_HIGH;
}

static void set_speed_pid_segment(float speed, speed_pid_segment_t *current_segment)
{
    float kp;
    float ki;
    speed_pid_segment_t next_segment = select_speed_pid_segment(speed);

    if (next_segment == *current_segment) {
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
    *current_segment = next_segment;
}

static int sign_from_float(float value)
{
    return (value >= 0.0f) ? 1 : -1;
}

static bool is_reverse_motion(float value, int motion_direction)
{
    return motion_direction != 0 &&
           std::fabs(value) > TARGET_SPEED_EPS &&
           sign_from_float(value) != motion_direction;
}

static void motor_output_duty(int duty)
{
    if (duty >= 0) {
        ls2k0300_gpio_level_set(&IN1, GPIO_HIGH);
        ls2k0300_gpio_level_set(&IN2, GPIO_LOW);
        ls2k0300_gtim_pwm_set_duty(&ENA, (uint32_t)duty);
    } else {
        ls2k0300_gpio_level_set(&IN1, GPIO_LOW);
        ls2k0300_gpio_level_set(&IN2, GPIO_HIGH);
        ls2k0300_gtim_pwm_set_duty(&ENA, (uint32_t)(-duty));
    }
}

static void motor_stop()
{
    ls2k0300_gpio_level_set(&IN1, GPIO_LOW);
    ls2k0300_gpio_level_set(&IN2, GPIO_LOW);
    ls2k0300_gtim_pwm_set_duty(&ENA, 0);
}

static void init_saved_position_count()
{
    long long saved_count = Load_encoder();

    if (saved_count != -1) {
        saved_position_count = saved_count;
        saved_position_valid = true;
        printf("load saved position count=%lld\n", saved_position_count);
        return;
    }

    saved_position_count = 0;
    saved_position_valid = false;
    printf("no saved position count\n");
}

static void save_current_position_count()
{
    long long current_count = Read_encoder();

    if (current_count != -1 && Save_encoder(current_count - origin_count) == 0) {
        printf("save current position count=%lld\n", current_count - origin_count);
    }
}

static void update_position_state(float target_ml, float current_ml)
{
    pthread_mutex_lock(&position_state_mutex);
    udp_target_position_ml = target_ml;
    udp_current_position_ml = current_ml;
    udp_position_valid = true;
    pthread_mutex_unlock(&position_state_mutex);
}

static bool get_position_state(float *target_ml, float *current_ml)
{
    bool valid;

    pthread_mutex_lock(&position_state_mutex);
    *target_ml = udp_target_position_ml;
    *current_ml = udp_current_position_ml;
    valid = udp_position_valid;
    pthread_mutex_unlock(&position_state_mutex);

    return valid;
}

static int udp_client_init(sockaddr_in *server_addr)
{
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("socket udp");
        return -1;
    }

    memset(server_addr, 0, sizeof(*server_addr));
    server_addr->sin_family = AF_INET;
    server_addr->sin_port = htons(UDP_SERVER_PORT);
    if (inet_pton(AF_INET, UDP_SERVER_IP, &server_addr->sin_addr) != 1) {
        perror("inet_pton udp server");
        close(sockfd);
        return -1;
    }

    return sockfd;
}

static void udp_send_position(int sockfd, const sockaddr_in *server_addr)
{
    float target_ml;
    float current_ml;
    char buffer[64];

    if (!get_position_state(&target_ml, &current_ml)) {
        return;
    }

    int len = snprintf(buffer, sizeof(buffer), "%.6f,%.6f\n", target_ml, current_ml);
    if (len <= 0 || len >= (int)sizeof(buffer)) {
        return;
    }

    sendto(sockfd, buffer, (size_t)len, 0,
           (const sockaddr *)server_addr, sizeof(*server_addr));
}

static void handle_position_delta_input(pollfd *input_fd)
{
    char line[64];
    float position_delta;
    int poll_ret;

    if (input_fd->fd < 0) {
        return;
    }

    poll_ret = poll(input_fd, 1, 0);
    if (poll_ret <= 0) {
        if (poll_ret < 0 && errno != EINTR) {
            perror("poll stdin");
            input_fd->fd = -1;
        }
        return;
    }

    if (input_fd->revents & (POLLERR | POLLNVAL)) {
        input_fd->fd = -1;
        return;
    }

    if (!(input_fd->revents & POLLIN)) {
        return;
    }

    if (fgets(line, sizeof(line), stdin) == NULL) {
        input_fd->fd = -1;
        return;
    }

    if (sscanf(line, "%f", &position_delta) == 1) {
        pthread_mutex_lock(&position_mutex);
        target_position_delta_ml = position_delta;
        position_pid.Set_output(0.0f);
        retarget_hold_cycles = RETARGET_HOLD_CYCLES;
        retarget_after_hold = true;
        position_control_enabled = true;
        pthread_mutex_unlock(&position_mutex);
        printf("position delta=%.4f ml, retarget after hold\n", position_delta);
    }
}

static void signal_handler(int signum)
{
    (void)signum;
    running = 0;
}

static void *ctrl_function(void *arg)
{
    (void)arg;

    EncoderData encoder_data;
    Init_encoder_data(&encoder_data);
    encoder_data.current_count = Read_encoder();
    encoder_data.last_count = encoder_data.current_count;
    long long deadzone_count = (long long)(POSITION_PRIORITY_DEADZONE_ML * COUNT_PER_1ML);
    long long abs_saved_count = (saved_position_count >= 0) ?
                                saved_position_count : -saved_position_count;
    long long local_origin_count;

    if (saved_position_valid && abs_saved_count > deadzone_count) {
        local_origin_count = encoder_data.current_count - saved_position_count;
        position_control_enabled = true;
        printf("startup homing by saved position count=%lld\n",
               saved_position_count);
    } else {
        local_origin_count = encoder_data.current_count;
        position_control_enabled = false;
        printf("startup position is in zero deadzone, skip homing\n");
    }

    origin_count = local_origin_count;
    printf("home to zero origin count=%lld, current count=%lld, saved count=%lld\n",
           local_origin_count, encoder_data.current_count, saved_position_count);

    timespec last_time;
    clock_gettime(CLOCK_MONOTONIC, &last_time);

    float speed_samples[3] = {0.0f, 0.0f, 0.0f};
    uint8_t speed_sample_count = 0;
    float filtered_speed = 0.0f;
    bool speed_filter_ready = false;

    uint8_t zero_speed_count = 0;
    bool zero_start_preloaded = false;
    speed_pid_segment_t speed_pid_segment = SPEED_PID_SEG_INVALID;
    float last_position_error = 0.0f;
    bool position_error_ready = false;
    bool position_cross_stopped = false;
    int motion_direction = 1;

    while (running) {
        nanosleep(&period, NULL);
        if (!running) {
            break;
        }

        timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        double dt = timespec_diff_sec(&now, &last_time);
        last_time = now;
        if (dt <= 0.0) {
            continue;
        }

        Get_encoder_data(&encoder_data);
        float current_position_ml =
            (float)((double)(encoder_data.current_count - local_origin_count) / COUNT_PER_1ML);
        float raw_speed = (float)((double)encoder_data.diff_count / COUNT_PER_1ML / dt);

        if (std::fabs(raw_speed) <= ZERO_SPEED_EPS) {
            if (zero_speed_count < ZERO_SPEED_CYCLES) {
                zero_speed_count++;
            }
        } else {
            zero_speed_count = 0;
            zero_start_preloaded = false;
        }

        speed_samples[2] = speed_samples[1];
        speed_samples[1] = speed_samples[0];
        speed_samples[0] = raw_speed;
        if (speed_sample_count < 3) {
            speed_sample_count++;
        }

        float median_speed = (speed_sample_count < 3) ? raw_speed :
                             median3(speed_samples[0], speed_samples[1], speed_samples[2]);
        if (!speed_filter_ready) {
            filtered_speed = median_speed;
            speed_filter_ready = true;
        } else {
            filtered_speed = low_pass_speed(median_speed, filtered_speed, (float)dt);
        }

        float target_position;
        float target_speed;
        bool motor_idle = false;

        pthread_mutex_lock(&position_mutex);
        if (!position_control_enabled) {
            target_speed = 0.0f;
            speed_pid.Set_output(0.0f);
            motor_idle = true;
        } else if (retarget_hold_cycles > 0) {
            retarget_hold_cycles--;
            target_speed = 0.0f;
            speed_pid.Set_output(0.0f);
            motor_idle = true;
            if (retarget_hold_cycles == 0 && retarget_after_hold) {
                target_position_ml = current_position_ml + target_position_delta_ml;
                position_pid.Set_output(0.0f);
                position_error_ready = false;
                position_cross_stopped = false;
                motion_direction = sign_from_float(target_position_ml - current_position_ml);
                retarget_after_hold = false;
            }
        } else {
            float position_error = target_position_ml - current_position_ml;
            float raw_target_speed = clamp_speed(position_pid.calculate(target_position_ml,
                                                                        current_position_ml));

            if (!position_error_ready) {
                last_position_error = position_error;
                motion_direction = sign_from_float(position_error);
                position_error_ready = true;
            }

            bool crossed_target = std::fabs(last_position_error) > POSITION_ERROR_EPS &&
                                  std::fabs(position_error) > POSITION_ERROR_EPS &&
                                  sign_from_float(position_error) != sign_from_float(last_position_error);
            bool reverse_after_cross = crossed_target &&
                                       std::fabs(raw_target_speed) > TARGET_SPEED_EPS &&
                                       sign_from_float(raw_target_speed) !=
                                           sign_from_float(last_position_error);
            bool priority_deadzone =
                (motion_direction > 0 &&
                 position_error > 0.0f &&
                 position_error < POSITION_PRIORITY_DEADZONE_ML) ||
                (motion_direction < 0 &&
                 position_error < 0.0f &&
                 position_error > -POSITION_PRIORITY_DEADZONE_ML);
            bool reverse_motion_command = is_reverse_motion(raw_target_speed,
                                                            motion_direction);

            if (priority_deadzone) {
                target_speed = 0.0f;
                position_pid.Set_output(0.0f);
                motor_idle = true;
            } else if (position_cross_stopped || reverse_after_cross) {
                target_speed = 0.0f;
                position_pid.Set_output(0.0f);
                position_cross_stopped = true;
                motor_idle = true;
            } else if (reverse_motion_command) {
                target_speed = 0.0f;
                position_pid.Set_output(0.0f);
                motor_idle = true;
            } else {
                target_speed = raw_target_speed;
            }

            last_position_error = position_error;
        }
        target_position = target_position_ml;
        pthread_mutex_unlock(&position_mutex);

        int duty = 0;

        if (std::fabs(target_speed) <= TARGET_SPEED_EPS) {
            motor_idle = true;
        }

        if (motor_idle) {
            speed_pid.Set_output(0.0f);
            zero_start_preloaded = false;
        } else {
            set_speed_pid_segment(target_speed, &speed_pid_segment);

            if (std::fabs(target_speed) == 0.0f) {
                speed_pid.Set_output(0.0f);
            }

            if (std::fabs(target_speed) > TARGET_SPEED_EPS &&
                std::fabs(target_speed) < LOW_SPEED_PRELOAD_LIMIT &&
                zero_speed_count >= ZERO_SPEED_CYCLES &&
                !zero_start_preloaded) {
                int preload_sign = sign_from_float(target_speed);
                speed_pid.Set_output(preload_sign * LOW_SPEED_PRELOAD_DUTY);
                zero_start_preloaded = true;
            }

            duty = (int)speed_pid.incremental_calculate(target_speed, filtered_speed);
            if (is_reverse_motion((float)duty, motion_direction)) {
                duty = 0;
                speed_pid.Set_output(0.0f);
                zero_start_preloaded = false;
                motor_idle = true;
            }
        }
        update_position_state(target_position, current_position_ml);
        printf("pos %.4f/%.4f speed %.4f/%.4f duty=%d\n",
               current_position_ml, target_position, filtered_speed, target_speed, duty);
        if (motor_idle) {
            motor_stop();
        } else {
            motor_output_duty(duty);
        }
    }

    motor_stop();

    return NULL;
}

int main()
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    ls2k0300_gpio_init(&IN1, PIN_89, GPIO_MODE_OUT, GPIO_MUX_GPIO);
    ls2k0300_gpio_init(&IN2, PIN_73, GPIO_MODE_OUT, GPIO_MUX_GPIO);
    ls2k0300_gtim_pwm_init(&ENA, GTIM_PWM1_PIN88, 20000, 0, GTIM_PWM_POL_INV);
    init_saved_position_count();

    sockaddr_in udp_server_addr;
    int udp_sockfd = udp_client_init(&udp_server_addr);
    if (udp_sockfd < 0) {
        motor_stop();
        ls2k0300_gpio_deinit(&IN1);
        ls2k0300_gpio_deinit(&IN2);
        ls2k0300_gtim_pwm_deinit(&ENA);
        return 1;
    }

    pthread_t ctrl_thread;
    if (pthread_create(&ctrl_thread, NULL, ctrl_function, NULL) != 0) {
        close(udp_sockfd);
        motor_stop();
        ls2k0300_gpio_deinit(&IN1);
        ls2k0300_gpio_deinit(&IN2);
        ls2k0300_gtim_pwm_deinit(&ENA);
        return 1;
    }

    pollfd input_fd;
    input_fd.fd = STDIN_FILENO;
    input_fd.events = POLLIN;
    input_fd.revents = 0;

    while (running) {
        handle_position_delta_input(&input_fd);
        udp_send_position(udp_sockfd, &udp_server_addr);
        nanosleep(&period, NULL);
    }

    close(udp_sockfd);
    pthread_join(ctrl_thread, NULL);
    save_current_position_count();
    ls2k0300_gpio_deinit(&IN1);
    ls2k0300_gpio_deinit(&IN2);
    ls2k0300_gtim_pwm_deinit(&ENA);
    return 0;
}

#endif
