#include "ArmKinematics.h"
#include "ArmMotorPosition.h"
#include "LS2K0300_DRV_INC.h"
#include "X42_V2.h"

#include "opencv2/opencv.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <dirent.h>
#include <fcntl.h>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#define ARM_MOTOR_UART UART4
#define ARM_MOTOR_BAUD B921600

#define ARM_MOTOR_ADDR_1 1U
#define ARM_MOTOR_ADDR_2 2U
#define ARM_MOTOR_ADDR_3 3U

#define ARM_GEAR_RATIO_1 (32.0 / 9.0)
#define ARM_GEAR_RATIO_2 (32.0 / 9.0)
#define ARM_GEAR_RATIO_3 (32.0 / 9.0)

#define ARM_INITIAL_THETA1_RAD 0.0
#define ARM_INITIAL_THETA2_RAD (90.0 * M_PI / 180.0)
#define ARM_INITIAL_THETA3_RAD (180.0 * M_PI / 180.0)
#define ARM_STARTUP_TARGET_THETA1_RAD 0.0
#define ARM_STARTUP_TARGET_THETA2_RAD 0.0
#define ARM_STARTUP_TARGET_THETA3_RAD (120.0 * M_PI / 180.0)

#define ARM_JOINT_DIRECTION_1 1.0
#define ARM_JOINT_DIRECTION_2 -1.0
#define ARM_JOINT_DIRECTION_3 1.0

#define ARM_ENDPOINT_X_DIRECTION 1.0
#define ARM_PATROL_X_SPEED_MMPS 10.0
#define ARM_SINE_Z_AMPLITUDE_MM 25.0
#define ARM_SINE_HALF_PERIOD_MS 4000LL
#define ARM_CORRECTION_PERIOD_MS 30LL
#define ARM_MAX_MOTOR_RPM 30.0f
#define ARM_VELOCITY_RAMP_RPMPS 20U
#define ARM_JOINT_ERROR_GAIN 1.0
#define ARM_MAX_CORRECTION_RADPS 3.0

#define ARM_LIMIT_LOW_Z_MIN_MM -110.0
#define ARM_LIMIT_LOW_Z_MAX_MM -30.0
#define ARM_LIMIT_LOW_Z_MIN_X_MM 110.0
#define ARM_LIMIT_Z90_MM 90.0
#define ARM_LIMIT_Z90_MAX_X_MM 145.0
#define ARM_LIMIT_Z100_MM 100.0
#define ARM_LIMIT_Z100_MAX_X_MM 125.0
#define ARM_LIMIT_MAX_X_MM 200.0
#define ARM_LIMIT_MAX_Z_MM 200.0
#define ARM_PATROL_MIN_X_MM 100.0
#define ARM_PATROL_MAX_X_MM 200.0
#define ARM_PATROL_X_TOLERANCE_MM 4.0

#define ARM_ORIGIN_MODE_NEAREST_ZERO 0x00U
#define ARM_ORIGIN_COMMAND_CODE 0x9AU
#define ARM_ORIGIN_STATUS_CODE 0x3BU
#define ARM_MOTOR_STATUS_CODE 0x3AU
#define ARM_POSITION_COMMAND_CODE 0xFBU
#define ARM_X42_CHECK_BYTE 0x6BU
#define ARM_COMMAND_STATUS_ALREADY_DONE 0x12U
#define ARM_COMMAND_STATUS_ACTION_DONE 0x9FU
#define ARM_COMMAND_STATUS_PARAM_ERROR 0xE2U
#define ARM_COMMAND_STATUS_FORMAT_ERROR 0xEEU
#define ARM_ORIGIN_READY_MASK 0x03U
#define ARM_ORIGIN_READY_VALUE 0x03U
#define ARM_ORIGIN_STATE_MASK 0x0CU
#define ARM_ORIGIN_STATE_BUSY 0x04U
#define ARM_ORIGIN_STATE_FAILED 0x08U
#define ARM_ORIGIN_STATE_DONE 0x00U
#define ARM_ORIGIN_NEAREST_DONE_STATUS 0x0BU
#define ARM_ORIGIN_TRIGGER_RESPONSE_TIMEOUT_MS 200
#define ARM_ORIGIN_STATUS_RESPONSE_TIMEOUT_MS 200
#define ARM_ORIGIN_FIRST_POLL_DELAY_MS 1L
#define ARM_ORIGIN_STATUS_POLL_MS 20L
#define ARM_ORIGIN_WAIT_TIMEOUT_MS 15000LL
#define ARM_ORIGIN_STABLE_DONE_POLLS 3U
#define ARM_POSITION_MODE_REL_CURRENT 0x02U
#define ARM_POSITION_RESPONSE_TIMEOUT_MS 200
#define ARM_POSITION_STATUS_RESPONSE_TIMEOUT_MS 200
#define ARM_POSITION_STATUS_POLL_MS 20L
#define ARM_POSITION_WAIT_TIMEOUT_MS 15000LL
#define ARM_POSITION_STABLE_DONE_POLLS 3U
#define ARM_POSITION_ARRIVED_MASK 0x02U
#define ARM_POSITION_MOVE_SPEED_RPM 10.0f
#define ARM_POSITION_SKIP_EPS_DEG 0.2
#define ARM_STARTUP_JOINT_ARRIVE_EPS_DEG 2.0
#define ARM_SHUTDOWN_TARGET_THETA1_RAD ARM_INITIAL_THETA1_RAD
#define ARM_SHUTDOWN_TARGET_THETA2_RAD ARM_INITIAL_THETA2_RAD
#define ARM_SHUTDOWN_TARGET_THETA3_RAD ARM_INITIAL_THETA3_RAD
#define ARM_SHUTDOWN_JOINT_ARRIVE_EPS_DEG 2.0
#define ARM_POSITION_READ_MAX_TRIES 2
#define ARM_POSITION_READ_RETRY_DELAY_MS 5L
#define ARM_PATROL_HALF_CYCLE_TIMEOUT_MARGIN_MS 5000LL

#define CAMERA_WIDTH 640
#define CAMERA_HEIGHT 480
#define CAMERA_FPS 30
#define CAMERA_BUFFERS 1

/*
 * 训练数据沿用 user/module 里已验证的采集格式：摄像头原始尺寸 BGR/MJPEG。
 * 后续 YOLO/ncnn 训练阶段再压缩；LCD 叠字只用于预览，不写入视频文件。
 */
#define VIDEO_WIDTH CAMERA_WIDTH
#define VIDEO_HEIGHT CAMERA_HEIGHT

#define LCD_RST_PIN PIN_51
#define LCD_DC_PIN PIN_50
#define LCD_BL_PIN PIN_74

#define VIDEO_SAVE_DIR "/home/root/video"
#define VIDEO_HOLD_BEFORE_RECORD_MS 2000LL
#define CAMERA_DEVICE "/dev/video0"

static volatile sig_atomic_t running = 1;

typedef struct {
    double theta1;
    double theta2;
    double theta3;
    double motor2_zero_degrees;
    double motor3_zero_degrees;
    double target_theta2;
    double target_theta3;
    int64_t last_correction_us;
    bool motor_zero_valid;
    bool joint_target_valid;
    JointMotorSpeeds last_speeds;
} ArmRuntime;

static void signal_handler(int signum)
{
    (void)signum;
    running = 0;
}

static int install_signal_handlers(void)
{
    struct sigaction action;

    std::memset(&action, 0, sizeof(action));
    action.sa_handler = signal_handler;
    sigemptyset(&action.sa_mask);

    if (sigaction(SIGINT, &action, NULL) != 0 ||
        sigaction(SIGTERM, &action, NULL) != 0) {
        perror("sigaction");
        return -1;
    }

    return 0;
}

static int64_t monotonic_time_ms(void)
{
    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        return 0;
    }

    return (int64_t)now.tv_sec * 1000LL +
           (int64_t)now.tv_nsec / 1000000LL;
}

static int64_t monotonic_time_us(void)
{
    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        return 0;
    }

    return (int64_t)now.tv_sec * 1000000LL +
           (int64_t)now.tv_nsec / 1000LL;
}

static void sleep_ms(long milliseconds)
{
    struct timespec delay;

    delay.tv_sec = milliseconds / 1000L;
    delay.tv_nsec = (milliseconds % 1000L) * 1000000L;
    while (running && nanosleep(&delay, &delay) != 0) {
    }
}

static void command_gap_ms(long milliseconds)
{
    struct timespec delay;

    delay.tv_sec = milliseconds / 1000L;
    delay.tv_nsec = (milliseconds % 1000L) * 1000000L;
    while (nanosleep(&delay, &delay) != 0 && errno == EINTR) {
    }
}

static double clamp_double(double value, double min_value, double max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static void zero_speeds(JointMotorSpeeds *speeds)
{
    if (speeds == NULL) {
        return;
    }

    std::memset(speeds, 0, sizeof(*speeds));
    speeds->valid = 1;
}

static void zero_motor_speed(MotorSpeed *speed)
{
    if (speed == NULL) {
        return;
    }

    speed->rpm = 0.0f;
    speed->rpm_abs = 0.0f;
    speed->dir = 0U;
}

static bool ensure_save_dir(void)
{
    if (mkdir(VIDEO_SAVE_DIR, 0755) == 0 || errno == EEXIST) {
        return true;
    }

    perror("mkdir video dir");
    return false;
}

static std::vector<int> make_y_scan_degrees(void)
{
    std::vector<int> degrees;

    degrees.push_back(0);
    for (int degree = 1; degree <= 15; ++degree) {
        degrees.push_back(degree);
    }
    for (int degree = -1; degree >= -15; --degree) {
        degrees.push_back(degree);
    }

    return degrees;
}

static bool parse_video_sequence(const char *filename, int *sequence)
{
    const char *dot;
    const char *underscore;
    int value = 0;

    if (filename == NULL || sequence == NULL) {
        return false;
    }

    dot = std::strrchr(filename, '.');
    if (dot == NULL || dot == filename) {
        return false;
    }

    underscore = dot;
    while (underscore > filename && *underscore != '_') {
        --underscore;
    }
    if (*underscore != '_' || underscore + 1 >= dot) {
        return false;
    }

    for (const char *p = underscore + 1; p < dot; ++p) {
        if (*p < '0' || *p > '9') {
            return false;
        }
        value = value * 10 + (*p - '0');
    }

    *sequence = value;
    return true;
}

static int find_latest_video_sequence(const char *dir_path)
{
    DIR *dir;
    struct dirent *entry;
    int latest = 0;

    if (dir_path == NULL) {
        return 0;
    }

    dir = opendir(dir_path);
    if (dir == NULL) {
        perror("opendir video dir");
        return 0;
    }

    while ((entry = readdir(dir)) != NULL) {
        int sequence;
        if (parse_video_sequence(entry->d_name, &sequence) &&
            sequence > latest) {
            latest = sequence;
        }
    }

    closedir(dir);
    return latest;
}

static std::string make_video_path(int y_degree, int sequence)
{
    char path[256];
    std::snprintf(path,
                  sizeof(path),
                  "%s/%+d_%05d.avi",
                  VIDEO_SAVE_DIR,
                  y_degree,
                  sequence);
    return std::string(path);
}

static void remove_partial_video(const std::string& path)
{
    if (!path.empty()) {
        (void)std::remove(path.c_str());
    }
}

static void draw_overlay(cv::Mat *image,
                         const std::string& line1,
                         const std::string& line2)
{
    if (image == NULL || image->empty()) {
        return;
    }

    cv::rectangle(*image,
                  cv::Rect(0, 0, image->cols, 54),
                  cv::Scalar(0, 0, 0),
                  cv::FILLED);
    cv::putText(*image,
                line1,
                cv::Point(8, 20),
                cv::FONT_HERSHEY_SIMPLEX,
                0.55,
                cv::Scalar(0, 255, 0),
                1,
                cv::LINE_AA);
    cv::putText(*image,
                line2,
                cv::Point(8, 44),
                cv::FONT_HERSHEY_SIMPLEX,
                0.55,
                cv::Scalar(0, 255, 255),
                1,
                cv::LINE_AA);
}

static void show_frame(icst7735_t *lcd,
                       const cv::Mat& bgr_frame,
                       const std::string& line1,
                       const std::string& line2)
{
    if (lcd == NULL || bgr_frame.empty()) {
        return;
    }

    cv::Mat display = bgr_frame.clone();
    draw_overlay(&display, line1, line2);
    (void)icst7735_show_camera_bgr888(lcd,
                                      display.data,
                                      (uint16_t)display.cols,
                                      (uint16_t)display.rows,
                                      (uint32_t)display.step);
}

static bool read_camera_bgr(cv::VideoCapture *camera, cv::Mat *bgr_frame)
{
    cv::Mat raw_frame;

    if (camera == NULL || bgr_frame == NULL) {
        return false;
    }

    if (!camera->read(raw_frame)) {
        return false;
    }

    cv::cvtColor(raw_frame, *bgr_frame, cv::COLOR_YUV2BGR_YUYV);
    return true;
}

static bool open_camera(cv::VideoCapture *camera)
{
    int fd;

    if (camera == NULL) {
        return false;
    }

    if (access(CAMERA_DEVICE, R_OK | W_OK) != 0) {
        perror("access " CAMERA_DEVICE);
        return false;
    }

    fd = open(CAMERA_DEVICE, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        perror("open " CAMERA_DEVICE);
        return false;
    }
    close(fd);

    printf("[INFO] open camera %s with OpenCV/V4L2\n", CAMERA_DEVICE);
    camera->open(CAMERA_DEVICE, cv::CAP_V4L2);
    if (!camera->isOpened()) {
        fprintf(stderr, "[ERROR] could not open camera\n");
        return false;
    }
    camera->set(cv::CAP_PROP_FOURCC,
                cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    camera->set(cv::CAP_PROP_CONVERT_RGB, 0);
    camera->set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
    camera->set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
    camera->set(cv::CAP_PROP_FPS, CAMERA_FPS);
    camera->set(cv::CAP_PROP_BUFFERSIZE, CAMERA_BUFFERS);
    camera->set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    camera->set(cv::CAP_PROP_EXPOSURE, 300);

    return true;
}

static ssize_t read_uart_byte_timeout(ls2k0300_uart_t *uart,
                                      uint8_t *byte,
                                      int64_t deadline_ms)
{
    if (uart == NULL || byte == NULL) {
        return -1;
    }

    while (running && monotonic_time_ms() < deadline_ms) {
        ssize_t n = ls2k0300_uart_read(uart, byte, 1);
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

static ssize_t read_x42_4byte_response_timeout(ls2k0300_uart_t *uart,
                                               uint8_t addr,
                                               uint8_t code,
                                               uint8_t response[4],
                                               int timeout_ms)
{
    size_t frame_pos = 0U;
    int64_t deadline_ms;

    if (uart == NULL || response == NULL || timeout_ms <= 0) {
        return -1;
    }

    deadline_ms = monotonic_time_ms() + (int64_t)timeout_ms;
    while (running && monotonic_time_ms() < deadline_ms) {
        uint8_t byte = 0U;
        ssize_t n = read_uart_byte_timeout(uart, &byte, deadline_ms);

        if (n < 0) {
            return -1;
        }
        if (n == 0) {
            break;
        }

        if (frame_pos == 0U) {
            if (byte != addr) {
                continue;
            }
            response[frame_pos++] = byte;
            continue;
        }

        if (frame_pos == 1U) {
            if (byte != code) {
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
        if (frame_pos == 4U) {
            if (response[3] == ARM_X42_CHECK_BYTE) {
                return 4;
            }
            frame_pos = 0U;
        }
    }

    return 0;
}

static bool arm_origin_status_done(uint8_t status)
{
    return ((status & ARM_ORIGIN_STATE_MASK) == ARM_ORIGIN_STATE_DONE) &&
           ((status & ARM_ORIGIN_READY_MASK) == ARM_ORIGIN_READY_VALUE);
}

static bool read_arm_origin_status(ls2k0300_uart_t *uart,
                                   uint8_t addr,
                                   uint8_t *status)
{
    uint8_t response[4] = {0};
    ssize_t response_size;

    if (uart == NULL || status == NULL) {
        return false;
    }

    if (ls2k0300_uart_flush(uart) != 0) {
        return false;
    }

    ZDT_X42_V2_Read_Sys_Params(uart, addr, S_OFLAG);
    response_size = read_x42_4byte_response_timeout(
        uart,
        addr,
        ARM_ORIGIN_STATUS_CODE,
        response,
        ARM_ORIGIN_STATUS_RESPONSE_TIMEOUT_MS);
    if (response_size != 4 ||
        response[0] != addr ||
        response[1] != ARM_ORIGIN_STATUS_CODE ||
        response[3] != ARM_X42_CHECK_BYTE) {
        return false;
    }

    *status = response[2];
    return true;
}

static bool home_arm_motor_nearest(ls2k0300_uart_t *uart, uint8_t addr)
{
    uint8_t response[4] = {0};
    ssize_t response_size;
    uint8_t stable_done_count = 0U;
    int64_t start_ms;

    if (uart == NULL || !running) {
        return false;
    }

    if (ls2k0300_uart_flush(uart) != 0) {
        return false;
    }

    printf("[INFO] motor%u nearest home\n", (unsigned int)addr);
    ZDT_X42_V2_Origin_Trigger_Return(uart,
                                     addr,
                                     ARM_ORIGIN_MODE_NEAREST_ZERO,
                                     false);

    response_size = read_x42_4byte_response_timeout(
        uart,
        addr,
        ARM_ORIGIN_COMMAND_CODE,
        response,
        ARM_ORIGIN_TRIGGER_RESPONSE_TIMEOUT_MS);
    if (response_size == 4 &&
        response[0] == addr &&
        response[1] == ARM_ORIGIN_COMMAND_CODE &&
        response[3] == ARM_X42_CHECK_BYTE) {
        if (response[2] == ARM_COMMAND_STATUS_ALREADY_DONE ||
            response[2] == ARM_COMMAND_STATUS_ACTION_DONE) {
            return true;
        }
        if (response[2] == ARM_COMMAND_STATUS_PARAM_ERROR ||
            response[2] == ARM_COMMAND_STATUS_FORMAT_ERROR) {
            fprintf(stderr,
                    "[ERROR] motor%u home command status=0x%02X\n",
                    (unsigned int)addr,
                    response[2]);
            return false;
        }
    }

    sleep_ms(ARM_ORIGIN_FIRST_POLL_DELAY_MS);
    start_ms = monotonic_time_ms();

    while (running) {
        uint8_t status;
        uint8_t origin_state;

        if (monotonic_time_ms() - start_ms > ARM_ORIGIN_WAIT_TIMEOUT_MS) {
            fprintf(stderr, "[ERROR] motor%u home timeout\n",
                    (unsigned int)addr);
            return false;
        }

        if (!read_arm_origin_status(uart, addr, &status)) {
            sleep_ms(ARM_ORIGIN_STATUS_POLL_MS);
            continue;
        }

        if (status == ARM_ORIGIN_NEAREST_DONE_STATUS) {
            return true;
        }

        origin_state = (uint8_t)(status & ARM_ORIGIN_STATE_MASK);
        if (origin_state == ARM_ORIGIN_STATE_BUSY) {
            stable_done_count = 0U;
        } else if (origin_state == ARM_ORIGIN_STATE_FAILED) {
            fprintf(stderr,
                    "[ERROR] motor%u home failed status=0x%02X\n",
                    (unsigned int)addr,
                    status);
            return false;
        } else if (arm_origin_status_done(status)) {
            stable_done_count++;
            if (stable_done_count >= ARM_ORIGIN_STABLE_DONE_POLLS) {
                return true;
            }
        } else {
            stable_done_count = 0U;
        }

        sleep_ms(ARM_ORIGIN_STATUS_POLL_MS);
    }

    return false;
}

static bool home_arm_motors_nearest(ls2k0300_uart_t *uart)
{
    const uint8_t addrs[] = {
        ARM_MOTOR_ADDR_1,
        ARM_MOTOR_ADDR_2,
        ARM_MOTOR_ADDR_3,
    };

    for (size_t i = 0U; i < sizeof(addrs) / sizeof(addrs[0]); ++i) {
        if (!home_arm_motor_nearest(uart, addrs[i])) {
            return false;
        }
        command_gap_ms(1);
    }

    return true;
}

static bool read_arm_motor_status(ls2k0300_uart_t *uart,
                                  uint8_t addr,
                                  uint8_t *status)
{
    uint8_t response[4] = {0};
    ssize_t response_size;

    if (uart == NULL || status == NULL) {
        return false;
    }

    if (ls2k0300_uart_flush(uart) != 0) {
        return false;
    }

    ZDT_X42_V2_Read_Sys_Params(uart, addr, S_SFLAG);
    response_size = read_x42_4byte_response_timeout(
        uart,
        addr,
        ARM_MOTOR_STATUS_CODE,
        response,
        ARM_POSITION_STATUS_RESPONSE_TIMEOUT_MS);
    if (response_size != 4 ||
        response[0] != addr ||
        response[1] != ARM_MOTOR_STATUS_CODE ||
        response[3] != ARM_X42_CHECK_BYTE) {
        return false;
    }

    *status = response[2];
    return true;
}

static bool wait_arm_position_arrived(ls2k0300_uart_t *uart,
                                      uint8_t addr,
                                      const char *name)
{
    uint8_t stable_arrived_count = 0U;
    int64_t start_ms = monotonic_time_ms();

    while (running) {
        uint8_t status;

        if (monotonic_time_ms() - start_ms > ARM_POSITION_WAIT_TIMEOUT_MS) {
            fprintf(stderr, "[ERROR] %s position timeout\n", name);
            return false;
        }

        if (!read_arm_motor_status(uart, addr, &status)) {
            sleep_ms(ARM_POSITION_STATUS_POLL_MS);
            continue;
        }

        if ((status & ARM_POSITION_ARRIVED_MASK) != 0U) {
            stable_arrived_count++;
            if (stable_arrived_count >= ARM_POSITION_STABLE_DONE_POLLS) {
                return true;
            }
        } else {
            stable_arrived_count = 0U;
        }

        sleep_ms(ARM_POSITION_STATUS_POLL_MS);
    }

    return false;
}

static double joint_delta_to_motor_degrees(double joint_delta_rad,
                                           double gear_ratio,
                                           double joint_direction)
{
    if (std::fabs(joint_direction) < 1e-9) {
        return 0.0;
    }

    return joint_delta_rad * 180.0 / M_PI * gear_ratio / joint_direction;
}

static uint8_t arm_position_dir_from_delta(double motor_delta_degrees)
{
    return (motor_delta_degrees >= 0.0) ? 0U : 1U;
}

static bool start_arm_relative_position_degrees(ls2k0300_uart_t *uart,
                                                uint8_t addr,
                                                const char *name,
                                                double motor_delta_degrees,
                                                bool *needs_wait)
{
    uint8_t response[4] = {0};
    ssize_t response_size;
    uint8_t dir;
    float abs_degrees;

    if (uart == NULL || name == NULL || needs_wait == NULL) {
        return false;
    }

    *needs_wait = false;
    if (std::fabs(motor_delta_degrees) <= ARM_POSITION_SKIP_EPS_DEG) {
        return true;
    }

    dir = arm_position_dir_from_delta(motor_delta_degrees);
    abs_degrees = (float)std::fabs(motor_delta_degrees);

    if (ls2k0300_uart_flush(uart) != 0) {
        return false;
    }

    printf("[INFO] %s relative position motor_delta=%.2fdeg dir=%u\n",
           name,
           motor_delta_degrees,
           (unsigned int)dir);
    ZDT_X42_V2_Bypass_Position_LV_Control(uart,
                                          addr,
                                          dir,
                                          ARM_POSITION_MOVE_SPEED_RPM,
                                          abs_degrees,
                                          ARM_POSITION_MODE_REL_CURRENT,
                                          0U);

    response_size = read_x42_4byte_response_timeout(
        uart,
        addr,
        ARM_POSITION_COMMAND_CODE,
        response,
        ARM_POSITION_RESPONSE_TIMEOUT_MS);
    if (response_size != 4) {
        *needs_wait = true;
        return true;
    }

    if (response[0] != addr ||
        response[1] != ARM_POSITION_COMMAND_CODE ||
        response[3] != ARM_X42_CHECK_BYTE) {
        *needs_wait = true;
        return true;
    }

    if (response[2] == ARM_COMMAND_STATUS_PARAM_ERROR ||
        response[2] == ARM_COMMAND_STATUS_FORMAT_ERROR) {
        fprintf(stderr,
                "[ERROR] %s position command status=0x%02X\n",
                name,
                response[2]);
        return false;
    }

    if (response[2] == ARM_COMMAND_STATUS_ACTION_DONE ||
        response[2] == ARM_COMMAND_STATUS_ALREADY_DONE) {
        return true;
    }

    *needs_wait = true;
    return true;
}

static bool read_motor_position_degrees_retry(ls2k0300_uart_t *uart,
                                              uint8_t addr,
                                              const char *name,
                                              double *motor_degrees)
{
    if (uart == NULL || name == NULL || motor_degrees == NULL) {
        return false;
    }

    for (int try_index = 0; try_index < ARM_POSITION_READ_MAX_TRIES;
         ++try_index) {
        if (arm_read_motor_position_degrees(uart,
                                            addr,
                                            motor_degrees,
                                            ARM_MOTOR_POSITION_TIMEOUT_MS)) {
            if (try_index > 0) {
                printf("[WARN] %s position read recovered after retry\n",
                       name);
            }
            return true;
        }

        if (try_index + 1 < ARM_POSITION_READ_MAX_TRIES) {
            printf("[WARN] %s position read missed once, retry\n", name);
            command_gap_ms(ARM_POSITION_READ_RETRY_DELAY_MS);
        }
    }

    fprintf(stderr,
            "[ERROR] %s position read failed on same address twice\n",
            name);
    return false;
}

static bool read_motor23_degrees(ls2k0300_uart_t *uart,
                                 double *motor2_degrees,
                                 double *motor3_degrees)
{
    if (!read_motor_position_degrees_retry(uart,
                                           ARM_MOTOR_ADDR_2,
                                           "motor2",
                                           motor2_degrees)) {
        return false;
    }
    command_gap_ms(1);

    if (!read_motor_position_degrees_retry(uart,
                                           ARM_MOTOR_ADDR_3,
                                           "motor3",
                                           motor3_degrees)) {
        return false;
    }
    command_gap_ms(1);

    return true;
}

static bool calibrate_runtime_motor_zero(ls2k0300_uart_t *uart,
                                         ArmRuntime *runtime)
{
    if (uart == NULL || runtime == NULL) {
        return false;
    }

    if (!read_motor23_degrees(uart,
                              &runtime->motor2_zero_degrees,
                              &runtime->motor3_zero_degrees)) {
        runtime->motor_zero_valid = false;
        return false;
    }

    runtime->theta1 = ARM_INITIAL_THETA1_RAD;
    runtime->theta2 = ARM_INITIAL_THETA2_RAD;
    runtime->theta3 = ARM_INITIAL_THETA3_RAD;
    runtime->target_theta2 = runtime->theta2;
    runtime->target_theta3 = runtime->theta3;
    runtime->motor_zero_valid = true;
    runtime->joint_target_valid = true;

    printf("[INFO] startup zero: theta1=0deg, M2=%.2fdeg -> theta2=90deg, "
           "M3=%.2fdeg -> theta3=180deg\n",
           runtime->motor2_zero_degrees,
           runtime->motor3_zero_degrees);
    return true;
}

static bool read_runtime_joint_angles(ls2k0300_uart_t *uart,
                                      ArmRuntime *runtime)
{
    double motor2_degrees;
    double motor3_degrees;

    if (uart == NULL || runtime == NULL || !runtime->motor_zero_valid) {
        return false;
    }

    if (!read_motor23_degrees(uart, &motor2_degrees, &motor3_degrees)) {
        return false;
    }

    double motor2_delta = motor2_degrees - runtime->motor2_zero_degrees;
    double motor3_delta = motor3_degrees - runtime->motor3_zero_degrees;

    runtime->theta2 = ARM_INITIAL_THETA2_RAD +
                      ARM_JOINT_DIRECTION_2 *
                      (motor2_delta * M_PI / 180.0) / ARM_GEAR_RATIO_2;
    runtime->theta3 = ARM_INITIAL_THETA3_RAD +
                      ARM_JOINT_DIRECTION_3 *
                      (motor3_delta * M_PI / 180.0) / ARM_GEAR_RATIO_3;
    return true;
}

static bool compute_runtime_endpoint_position(const ArmRuntime *runtime,
                                              point3d_t *position)
{
    if (runtime == NULL || position == NULL) {
        return false;
    }

    return arm_forward_kinematics(runtime->theta1,
                                  runtime->theta2,
                                  runtime->theta3,
                                  position) != 0;
}

static double normalize_angle_delta(double angle_rad)
{
    while (angle_rad > M_PI) {
        angle_rad -= 2.0 * M_PI;
    }
    while (angle_rad < -M_PI) {
        angle_rad += 2.0 * M_PI;
    }

    return angle_rad;
}

static double startup_joint_solution_distance(double theta2,
                                              double theta3,
                                              double ref_theta2,
                                              double ref_theta3)
{
    return std::fabs(normalize_angle_delta(theta2 - ref_theta2)) +
           std::fabs(normalize_angle_delta(theta3 - ref_theta3));
}

static bool solve_startup_xz_to_joint_angles(double x,
                                             double z,
                                             double ref_theta2,
                                             double ref_theta3,
                                             double *target_theta2,
                                             double *target_theta3)
{
    double delta_z;
    double reach;
    double gamma;
    double half_angle;
    double theta2_a;
    double theta3_a;
    double theta2_b;
    double theta3_b;
    double distance_a;
    double distance_b;

    if (target_theta2 == NULL || target_theta3 == NULL) {
        return false;
    }

    delta_z = z - ARM_LINK_HEIGHT_MM;
    reach = std::sqrt(x * x + delta_z * delta_z);
    if (reach > 2.0 * ARM_LINK_LENGTH_MM) {
        fprintf(stderr,
                "[ERROR] startup target unreachable: x=%.2fmm, z=%.2fmm\n",
                x,
                z);
        return false;
    }

    gamma = std::atan2(x, delta_z);
    half_angle = std::acos(clamp_double(reach / (2.0 * ARM_LINK_LENGTH_MM),
                                        -1.0,
                                        1.0));

    theta2_a = gamma - half_angle;
    theta3_a = gamma + half_angle;
    theta2_b = gamma + half_angle;
    theta3_b = gamma - half_angle;

    distance_a = startup_joint_solution_distance(theta2_a,
                                                 theta3_a,
                                                 ref_theta2,
                                                 ref_theta3);
    distance_b = startup_joint_solution_distance(theta2_b,
                                                 theta3_b,
                                                 ref_theta2,
                                                 ref_theta3);
    if (distance_a <= distance_b) {
        *target_theta2 = theta2_a;
        *target_theta3 = theta3_a;
    } else {
        *target_theta2 = theta2_b;
        *target_theta3 = theta3_b;
    }

    return true;
}

static bool is_low_z_band(double z)
{
    return z >= ARM_LIMIT_LOW_Z_MIN_MM && z <= ARM_LIMIT_LOW_Z_MAX_MM;
}

static double arm_patrol_axis_x(const point3d_t *position)
{
    if (position == NULL) {
        return 0.0;
    }

    return std::sqrt((double)position->x * (double)position->x +
                     (double)position->y * (double)position->y);
}

static unsigned int limit_endpoint_velocity_by_position(const point3d_t *position,
                                                        double *vx,
                                                        double *vy,
                                                        double *vz)
{
    unsigned int flags = 0U;
    double x;
    double z;

    if (position == NULL || vx == NULL || vy == NULL || vz == NULL) {
        return flags;
    }

    (void)vy;
    x = arm_patrol_axis_x(position);
    z = position->z;

    if (x >= ARM_LIMIT_MAX_X_MM && *vx > 0.0) {
        *vx = 0.0;
        flags |= 0x01U;
    }

    if (z >= ARM_LIMIT_MAX_Z_MM && *vz > 0.0) {
        *vz = 0.0;
        flags |= 0x02U;
    }

    if (is_low_z_band(z)) {
        if (x <= ARM_LIMIT_LOW_Z_MIN_X_MM && *vx < 0.0) {
            *vx = 0.0;
            flags |= 0x04U;
        }
        if (x < ARM_LIMIT_LOW_Z_MIN_X_MM && std::fabs(*vz) > 1e-9) {
            *vz = 0.0;
            flags |= 0x04U;
        }
    } else if (x < ARM_LIMIT_LOW_Z_MIN_X_MM) {
        if (z < ARM_LIMIT_LOW_Z_MIN_MM && *vz > 0.0) {
            *vz = 0.0;
            flags |= 0x04U;
        } else if (z > ARM_LIMIT_LOW_Z_MAX_MM && *vz < 0.0) {
            *vz = 0.0;
            flags |= 0x04U;
        }
    }

    if (z >= ARM_LIMIT_Z100_MM) {
        if (x >= ARM_LIMIT_Z100_MAX_X_MM && *vx > 0.0) {
            *vx = 0.0;
            flags |= 0x08U;
        }
        if (x > ARM_LIMIT_Z100_MAX_X_MM && *vz > 0.0) {
            *vz = 0.0;
            flags |= 0x08U;
        }
    } else if (z >= ARM_LIMIT_Z90_MM) {
        if (x >= ARM_LIMIT_Z90_MAX_X_MM && *vx > 0.0) {
            *vx = 0.0;
            flags |= 0x10U;
        }
        if (x > ARM_LIMIT_Z90_MAX_X_MM && *vz > 0.0) {
            *vz = 0.0;
            flags |= 0x10U;
        }
    }

    return flags;
}

static bool patrol_turnaround_reached(double patrol_axis_x, double patrol_dir)
{
    if (patrol_dir > 0.0) {
        return patrol_axis_x >= ARM_PATROL_MAX_X_MM - ARM_PATROL_X_TOLERANCE_MM;
    }

    if (patrol_dir < 0.0) {
        return patrol_axis_x <= ARM_PATROL_MIN_X_MM + ARM_PATROL_X_TOLERANCE_MM;
    }

    return false;
}

static void prepare_patrol_direction(double patrol_axis_x, double *patrol_dir)
{
    if (patrol_dir == NULL) {
        return;
    }

    if (*patrol_dir > 0.0 &&
        patrol_axis_x >= ARM_PATROL_MAX_X_MM - ARM_PATROL_X_TOLERANCE_MM) {
        *patrol_dir = -1.0;
    } else if (*patrol_dir < 0.0 &&
               patrol_axis_x <= ARM_PATROL_MIN_X_MM + ARM_PATROL_X_TOLERANCE_MM) {
        *patrol_dir = 1.0;
    }
}

static void apply_motor_limit(JointMotorSpeeds *speeds)
{
    float max_rpm;
    float scale;

    if (speeds == NULL) {
        return;
    }

    max_rpm = std::max(speeds->velocity_1.rpm_abs,
                       std::max(speeds->velocity_2.rpm_abs,
                                speeds->velocity_3.rpm_abs));
    if (max_rpm <= ARM_MAX_MOTOR_RPM || max_rpm <= 0.0f) {
        return;
    }

    scale = ARM_MAX_MOTOR_RPM / max_rpm;
    speeds->velocity_1.rpm *= scale;
    speeds->velocity_2.rpm *= scale;
    speeds->velocity_3.rpm *= scale;
    speeds->velocity_1.rpm_abs = fabsf(speeds->velocity_1.rpm);
    speeds->velocity_2.rpm_abs = fabsf(speeds->velocity_2.rpm);
    speeds->velocity_3.rpm_abs = fabsf(speeds->velocity_3.rpm);
}

static void send_arm_speeds(ls2k0300_uart_t *uart,
                            const JointMotorSpeeds *speeds)
{
    if (uart == NULL || speeds == NULL) {
        return;
    }

    ZDT_X42_V2_Velocity_Control(uart,
                                ARM_MOTOR_ADDR_1,
                                speeds->velocity_1.dir,
                                ARM_VELOCITY_RAMP_RPMPS,
                                speeds->velocity_1.rpm_abs,
                                0);
    command_gap_ms(1);
    ZDT_X42_V2_Velocity_Control(uart,
                                ARM_MOTOR_ADDR_2,
                                speeds->velocity_2.dir,
                                ARM_VELOCITY_RAMP_RPMPS,
                                speeds->velocity_2.rpm_abs,
                                0);
    command_gap_ms(1);
    ZDT_X42_V2_Velocity_Control(uart,
                                ARM_MOTOR_ADDR_3,
                                speeds->velocity_3.dir,
                                ARM_VELOCITY_RAMP_RPMPS,
                                speeds->velocity_3.rpm_abs,
                                0);
    command_gap_ms(1);
}

static void stop_arm_motion(ls2k0300_uart_t *uart, ArmRuntime *runtime)
{
    JointMotorSpeeds zero;

    zero_speeds(&zero);
    send_arm_speeds(uart, &zero);
    if (runtime != NULL) {
        runtime->last_speeds = zero;
        runtime->joint_target_valid = false;
    }
}

static bool move_arm_joint_relative(ls2k0300_uart_t *uart,
                                    uint8_t addr,
                                    const char *name,
                                    double joint_delta_rad,
                                    double gear_ratio,
                                    double joint_direction)
{
    bool needs_wait = false;
    double motor_delta_degrees =
        joint_delta_to_motor_degrees(joint_delta_rad,
                                     gear_ratio,
                                     joint_direction);

    if (!start_arm_relative_position_degrees(uart,
                                             addr,
                                             name,
                                             motor_delta_degrees,
                                             &needs_wait)) {
        return false;
    }

    if (needs_wait &&
        !wait_arm_position_arrived(uart, addr, name)) {
        return false;
    }

    return true;
}

static bool move_arm_theta23_to_target(ls2k0300_uart_t *uart,
                                       ArmRuntime *runtime,
                                       const char *stage_name,
                                       double target_theta2,
                                       double target_theta3)
{
    bool wait_motor2 = false;
    bool wait_motor3 = false;
    double motor2_delta_degrees;
    double motor3_delta_degrees;
    point3d_t measured_position;

    if (uart == NULL || runtime == NULL || stage_name == NULL) {
        return false;
    }

    motor2_delta_degrees = joint_delta_to_motor_degrees(
        target_theta2 - runtime->theta2,
        ARM_GEAR_RATIO_2,
        ARM_JOINT_DIRECTION_2);
    motor3_delta_degrees = joint_delta_to_motor_degrees(
        target_theta3 - runtime->theta3,
        ARM_GEAR_RATIO_3,
        ARM_JOINT_DIRECTION_3);

    printf("[INFO] %s target theta2=%.2fdeg theta3=%.2fdeg "
           "motor_delta2=%.2fdeg motor_delta3=%.2fdeg\n",
           stage_name,
           target_theta2 * 180.0 / M_PI,
           target_theta3 * 180.0 / M_PI,
           motor2_delta_degrees,
           motor3_delta_degrees);

    if (!start_arm_relative_position_degrees(uart,
                                             ARM_MOTOR_ADDR_2,
                                             "arm motor2",
                                             motor2_delta_degrees,
                                             &wait_motor2)) {
        return false;
    }
    command_gap_ms(1);

    if (!start_arm_relative_position_degrees(uart,
                                             ARM_MOTOR_ADDR_3,
                                             "arm motor3",
                                             motor3_delta_degrees,
                                             &wait_motor3)) {
        return false;
    }

    if (wait_motor2 &&
        !wait_arm_position_arrived(uart, ARM_MOTOR_ADDR_2, "arm motor2")) {
        return false;
    }
    if (wait_motor3 &&
        !wait_arm_position_arrived(uart, ARM_MOTOR_ADDR_3, "arm motor3")) {
        return false;
    }

    if (!read_runtime_joint_angles(uart, runtime)) {
        return false;
    }

    if (compute_runtime_endpoint_position(runtime, &measured_position)) {
        printf("[INFO] %s measured endpoint x=%.2f y=%.2f z=%.2fmm\n",
               stage_name,
               (double)measured_position.x,
               (double)measured_position.y,
               (double)measured_position.z);
    }
    return true;
}

static bool move_arm_to_startup_pose(ls2k0300_uart_t *uart,
                                     ArmRuntime *runtime)
{
    point3d_t current_position;
    point3d_t target_position;
    double stage_z_theta2;
    double stage_z_theta3;
    double theta2_error_deg;
    double theta3_error_deg;
    point3d_t measured_position;

    if (uart == NULL || runtime == NULL || !runtime->motor_zero_valid) {
        return false;
    }

    if (std::fabs(ARM_STARTUP_TARGET_THETA1_RAD -
                  ARM_INITIAL_THETA1_RAD) > 1e-6) {
        fprintf(stderr,
                "[ERROR] startup theta1 target must stay at calibrated 0deg "
                "so z moves before x\n");
        return false;
    }

    if (!read_runtime_joint_angles(uart, runtime)) {
        return false;
    }

    printf("[INFO] move arm to startup pose theta=(%.2f, %.2f, %.2f)deg\n",
           ARM_STARTUP_TARGET_THETA1_RAD * 180.0 / M_PI,
           ARM_STARTUP_TARGET_THETA2_RAD * 180.0 / M_PI,
           ARM_STARTUP_TARGET_THETA3_RAD * 180.0 / M_PI);

    if (!compute_runtime_endpoint_position(runtime, &current_position) ||
        !arm_forward_kinematics(ARM_STARTUP_TARGET_THETA1_RAD,
                                ARM_STARTUP_TARGET_THETA2_RAD,
                                ARM_STARTUP_TARGET_THETA3_RAD,
                                &target_position)) {
        return false;
    }

    runtime->theta1 = ARM_STARTUP_TARGET_THETA1_RAD;

    printf("[INFO] startup stage order: z first, then x/final\n");
    printf("[INFO] startup stage z: keep x=%.2fmm, z %.2fmm -> %.2fmm\n",
           (double)current_position.x,
           (double)current_position.z,
           (double)target_position.z);

    if (!solve_startup_xz_to_joint_angles(current_position.x,
                                          target_position.z,
                                          runtime->theta2,
                                          runtime->theta3,
                                          &stage_z_theta2,
                                          &stage_z_theta3)) {
        return false;
    }

    if (!move_arm_theta23_to_target(uart,
                                    runtime,
                                    "startup stage z",
                                    stage_z_theta2,
                                    stage_z_theta3)) {
        return false;
    }

    printf("[INFO] startup stage x/final: x %.2fmm -> %.2fmm, z=%.2fmm\n",
           (double)current_position.x,
           (double)target_position.x,
           (double)target_position.z);
    if (!move_arm_theta23_to_target(uart,
                                    runtime,
                                    "startup final",
                                    ARM_STARTUP_TARGET_THETA2_RAD,
                                    ARM_STARTUP_TARGET_THETA3_RAD)) {
        return false;
    }

    theta2_error_deg =
        (runtime->theta2 - ARM_STARTUP_TARGET_THETA2_RAD) * 180.0 / M_PI;
    theta3_error_deg =
        (runtime->theta3 - ARM_STARTUP_TARGET_THETA3_RAD) * 180.0 / M_PI;
    if (std::fabs(theta2_error_deg) > ARM_STARTUP_JOINT_ARRIVE_EPS_DEG ||
        std::fabs(theta3_error_deg) > ARM_STARTUP_JOINT_ARRIVE_EPS_DEG) {
        fprintf(stderr,
                "[ERROR] startup pose error too large: "
                "theta2_err=%.2fdeg theta3_err=%.2fdeg\n",
                theta2_error_deg,
                theta3_error_deg);
        return false;
    }

    runtime->target_theta2 = runtime->theta2;
    runtime->target_theta3 = runtime->theta3;
    runtime->joint_target_valid = true;
    printf("[INFO] startup measured theta=(%.2f, %.2f, %.2f)deg\n",
           runtime->theta1 * 180.0 / M_PI,
           runtime->theta2 * 180.0 / M_PI,
           runtime->theta3 * 180.0 / M_PI);
    if (compute_runtime_endpoint_position(runtime, &measured_position)) {
        printf("[INFO] startup measured endpoint x=%.2f y=%.2f z=%.2fmm\n",
               (double)measured_position.x,
               (double)measured_position.y,
               (double)measured_position.z);
    }
    return true;
}

static bool move_arm_to_shutdown_pose(ls2k0300_uart_t *uart,
                                      ArmRuntime *runtime)
{
    point3d_t current_position;
    point3d_t final_position;
    point3d_t measured_position;
    double current_axis_x;
    double final_axis_x;
    double stage_x_theta2;
    double stage_x_theta3;
    double theta2_error_deg;
    double theta3_error_deg;

    if (uart == NULL || runtime == NULL || !runtime->motor_zero_valid) {
        return false;
    }

    printf("[INFO] shutdown return target theta=(0,90,180)deg, "
           "x first, then z/final\n");
    if (!read_runtime_joint_angles(uart, runtime)) {
        fprintf(stderr, "[ERROR] shutdown pose read current joint angles failed\n");
        return false;
    }

    if (!compute_runtime_endpoint_position(runtime, &current_position) ||
        !arm_forward_kinematics(ARM_SHUTDOWN_TARGET_THETA1_RAD,
                                ARM_SHUTDOWN_TARGET_THETA2_RAD,
                                ARM_SHUTDOWN_TARGET_THETA3_RAD,
                                &final_position)) {
        return false;
    }

    current_axis_x = arm_patrol_axis_x(&current_position);
    final_axis_x = arm_patrol_axis_x(&final_position);

    printf("[INFO] shutdown stage x: x_axis %.2fmm -> %.2fmm, "
           "keep z=%.2fmm, final z=%.2fmm\n",
           current_axis_x,
           final_axis_x,
           (double)current_position.z,
           (double)final_position.z);

    if (!solve_startup_xz_to_joint_angles(final_axis_x,
                                          current_position.z,
                                          runtime->theta2,
                                          runtime->theta3,
                                          &stage_x_theta2,
                                          &stage_x_theta3)) {
        return false;
    }

    if (!move_arm_theta23_to_target(uart,
                                    runtime,
                                    "shutdown stage x",
                                    stage_x_theta2,
                                    stage_x_theta3)) {
        return false;
    }

    printf("[INFO] shutdown stage z/final: theta2=%.2fdeg theta3=%.2fdeg\n",
           ARM_SHUTDOWN_TARGET_THETA2_RAD * 180.0 / M_PI,
           ARM_SHUTDOWN_TARGET_THETA3_RAD * 180.0 / M_PI);
    if (!move_arm_theta23_to_target(uart,
                                    runtime,
                                    "shutdown zero",
                                    ARM_SHUTDOWN_TARGET_THETA2_RAD,
                                    ARM_SHUTDOWN_TARGET_THETA3_RAD)) {
        return false;
    }

    theta2_error_deg =
        (runtime->theta2 - ARM_SHUTDOWN_TARGET_THETA2_RAD) * 180.0 / M_PI;
    theta3_error_deg =
        (runtime->theta3 - ARM_SHUTDOWN_TARGET_THETA3_RAD) * 180.0 / M_PI;
    if (std::fabs(theta2_error_deg) > ARM_SHUTDOWN_JOINT_ARRIVE_EPS_DEG ||
        std::fabs(theta3_error_deg) > ARM_SHUTDOWN_JOINT_ARRIVE_EPS_DEG) {
        fprintf(stderr,
                "[ERROR] shutdown pose error too large: "
                "theta2_err=%.2fdeg theta3_err=%.2fdeg\n",
                theta2_error_deg,
                theta3_error_deg);
        return false;
    }

    if (!move_arm_joint_relative(uart,
                                 ARM_MOTOR_ADDR_1,
                                 "arm motor1/y shutdown",
                                 ARM_SHUTDOWN_TARGET_THETA1_RAD -
                                     runtime->theta1,
                                 ARM_GEAR_RATIO_1,
                                 ARM_JOINT_DIRECTION_1)) {
        return false;
    }
    runtime->theta1 = ARM_SHUTDOWN_TARGET_THETA1_RAD;
    command_gap_ms(1);

    runtime->target_theta2 = runtime->theta2;
    runtime->target_theta3 = runtime->theta3;
    runtime->joint_target_valid = true;
    if (compute_runtime_endpoint_position(runtime, &measured_position)) {
        printf("[INFO] shutdown measured endpoint x=%.2f y=%.2f z=%.2fmm\n",
               (double)measured_position.x,
               (double)measured_position.y,
               (double)measured_position.z);
    }
    return true;
}

static bool move_y_to_degree(ls2k0300_uart_t *uart,
                             ArmRuntime *runtime,
                             int current_degree,
                             int target_degree)
{
    double delta_rad;

    if (uart == NULL || runtime == NULL) {
        return false;
    }

    delta_rad = (double)(target_degree - current_degree) * M_PI / 180.0;
    if (!move_arm_joint_relative(uart,
                                 ARM_MOTOR_ADDR_1,
                                 "arm motor1/y",
                                 delta_rad,
                                 ARM_GEAR_RATIO_1,
                                 ARM_JOINT_DIRECTION_1)) {
        return false;
    }

    runtime->theta1 = ARM_INITIAL_THETA1_RAD +
                      (double)target_degree * M_PI / 180.0;
    runtime->joint_target_valid = false;
    printf("[INFO] y offset now %+ddeg\n", target_degree);
    return true;
}

static bool update_arm_sine_motion(ls2k0300_uart_t *uart,
                                   ArmRuntime *runtime,
                                   int clip_index,
                                   double clip_start_axis_x,
                                   double clip_target_axis_x,
                                   double *patrol_dir,
                                   bool *turnaround_reached)
{
    point3d_t position;
    double patrol_axis_x;
    double sweep_distance;
    double sweep_progress;
    double radial_vx;
    double limited_radial_vx;
    double limited_vy = 0.0;
    double vz;
    double omega;
    double phase_rad;
    double vx_world;
    double vy_world;
    double dt;
    int64_t now_us;

    if (uart == NULL || runtime == NULL || patrol_dir == NULL ||
        turnaround_reached == NULL) {
        return false;
    }
    *turnaround_reached = false;

    if (!read_runtime_joint_angles(uart, runtime)) {
        fprintf(stderr, "[ERROR] read runtime joint angles failed\n");
        stop_arm_motion(uart, runtime);
        return false;
    }

    if (!compute_runtime_endpoint_position(runtime, &position)) {
        fprintf(stderr, "[ERROR] compute runtime endpoint failed\n");
        stop_arm_motion(uart, runtime);
        return false;
    }

    patrol_axis_x = arm_patrol_axis_x(&position);
    if (patrol_turnaround_reached(patrol_axis_x, *patrol_dir)) {
        *patrol_dir = -*patrol_dir;
        *turnaround_reached = true;
        zero_speeds(&runtime->last_speeds);
        send_arm_speeds(uart, &runtime->last_speeds);
        printf("[INFO] half-cycle boundary reached, x_axis=%.2fmm, "
               "next_dir=%.0f\n",
               patrol_axis_x,
               *patrol_dir);
        return true;
    }

    sweep_distance = std::fabs(clip_target_axis_x - clip_start_axis_x);
    if (sweep_distance < 1.0) {
        sweep_distance = 1.0;
    }
    sweep_progress = std::fabs(patrol_axis_x - clip_start_axis_x) /
                     sweep_distance;
    sweep_progress = clamp_double(sweep_progress, 0.0, 1.0);
    phase_rad = (double)clip_index * M_PI + sweep_progress * M_PI;

    omega = M_PI / (sweep_distance / ARM_PATROL_X_SPEED_MMPS);
    radial_vx = ARM_ENDPOINT_X_DIRECTION *
                (*patrol_dir) *
                ARM_PATROL_X_SPEED_MMPS;
    vz = ARM_SINE_Z_AMPLITUDE_MM * omega * std::cos(phase_rad);

    limited_radial_vx = radial_vx;
    (void)limit_endpoint_velocity_by_position(&position,
                                              &limited_radial_vx,
                                              &limited_vy,
                                              &vz);

    vx_world = limited_radial_vx * std::cos(runtime->theta1);
    vy_world = limited_radial_vx * std::sin(runtime->theta1);

    now_us = monotonic_time_us();
    dt = 0.0;
    if (runtime->last_correction_us > 0) {
        dt = (double)(now_us - runtime->last_correction_us) / 1000000.0;
    }
    if (dt < 0.0) {
        dt = 0.0;
    }
    if (dt > 0.2) {
        dt = 0.2;
    }
    runtime->last_correction_us = now_us;

    JointMotorSpeeds target_speeds;
    zero_speeds(&target_speeds);
    compute_motor_speeds(vx_world,
                         vy_world,
                         vz,
                         runtime->theta1,
                         runtime->theta2,
                         runtime->theta3,
                         ARM_GEAR_RATIO_1,
                         ARM_GEAR_RATIO_2,
                         ARM_GEAR_RATIO_3,
                         &target_speeds);
    if (!target_speeds.valid) {
        fprintf(stderr, "[ERROR] arm velocity solve failed\n");
        stop_arm_motion(uart, runtime);
        return false;
    }

    double target_dtheta2 =
        arm_motor_rpm_to_joint_radps(target_speeds.velocity_2.rpm,
                                     ARM_GEAR_RATIO_2);
    double target_dtheta3 =
        arm_motor_rpm_to_joint_radps(target_speeds.velocity_3.rpm,
                                     ARM_GEAR_RATIO_3);

    if (!runtime->joint_target_valid) {
        runtime->target_theta2 = runtime->theta2;
        runtime->target_theta3 = runtime->theta3;
        runtime->joint_target_valid = true;
    }

    runtime->target_theta2 += target_dtheta2 * dt;
    runtime->target_theta3 += target_dtheta3 * dt;

    double error2 = runtime->theta2 - runtime->target_theta2;
    double error3 = runtime->theta3 - runtime->target_theta3;
    double correction2 = clamp_double(error2 * ARM_JOINT_ERROR_GAIN,
                                      -ARM_MAX_CORRECTION_RADPS,
                                      ARM_MAX_CORRECTION_RADPS);
    double correction3 = clamp_double(error3 * ARM_JOINT_ERROR_GAIN,
                                      -ARM_MAX_CORRECTION_RADPS,
                                      ARM_MAX_CORRECTION_RADPS);

    JointMotorSpeeds speeds = target_speeds;
    zero_motor_speed(&speeds.velocity_1);
    arm_motor_speed_from_joint_radps(&speeds.velocity_2,
                                     target_dtheta2 - correction2,
                                     ARM_GEAR_RATIO_2,
                                     1U);
    arm_motor_speed_from_joint_radps(&speeds.velocity_3,
                                     target_dtheta3 - correction3,
                                     ARM_GEAR_RATIO_3,
                                     0U);
    apply_motor_limit(&speeds);
    send_arm_speeds(uart, &speeds);
    runtime->last_speeds = speeds;

    return true;
}

static bool show_live_for_ms(cv::VideoCapture *camera,
                             icst7735_t *lcd,
                             int duration_ms,
                             const std::string& line1,
                             const std::string& line2)
{
    int64_t start_ms = monotonic_time_ms();

    if (camera == NULL || lcd == NULL) {
        printf("[INFO] %s | %s\n", line1.c_str(), line2.c_str());
        while (running && monotonic_time_ms() - start_ms < duration_ms) {
            sleep_ms(20L);
        }
        return running != 0;
    }

    while (running && monotonic_time_ms() - start_ms < duration_ms) {
        cv::Mat bgr_frame;
        if (!read_camera_bgr(camera, &bgr_frame)) {
            fprintf(stderr, "[ERROR] camera read failed\n");
            return false;
        }
        show_frame(lcd, bgr_frame, line1, line2);
    }

    return running != 0;
}

static bool record_half_cycle(cv::VideoCapture *camera,
                              icst7735_t *lcd,
                              ls2k0300_uart_t *motor_uart,
                              ArmRuntime *runtime,
                              double *patrol_dir,
                              int y_degree,
                              int clip_index,
                              int clip_count,
                              const std::string& video_path)
{
    cv::VideoWriter writer;
    point3d_t start_position;
    double start_axis_x;
    double target_axis_x;
    double sweep_distance;
    int64_t expected_ms;
    int64_t timeout_ms;

    if (camera == NULL || video_path.empty()) {
        fprintf(stderr, "[ERROR] camera/video path is required\n");
        return false;
    }

    if (!read_runtime_joint_angles(motor_uart, runtime) ||
        !compute_runtime_endpoint_position(runtime, &start_position)) {
        fprintf(stderr, "[ERROR] could not read half-cycle start position\n");
        return false;
    }

    start_axis_x = arm_patrol_axis_x(&start_position);
    prepare_patrol_direction(start_axis_x, patrol_dir);
    target_axis_x = (*patrol_dir > 0.0) ? ARM_PATROL_MAX_X_MM
                                        : ARM_PATROL_MIN_X_MM;
    sweep_distance = std::fabs(target_axis_x - start_axis_x);
    if (sweep_distance < 1.0) {
        sweep_distance = 1.0;
    }
    expected_ms = (int64_t)((sweep_distance / ARM_PATROL_X_SPEED_MMPS) *
                            1000.0);
    if (expected_ms < 1000LL) {
        expected_ms = 1000LL;
    }
    timeout_ms = expected_ms * 2LL + ARM_PATROL_HALF_CYCLE_TIMEOUT_MARGIN_MS;

    writer.open(video_path,
                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                CAMERA_FPS,
                cv::Size(VIDEO_WIDTH, VIDEO_HEIGHT));
    if (!writer.isOpened()) {
        fprintf(stderr, "[ERROR] could not open writer: %s\n",
                video_path.c_str());
        return false;
    }
    printf("[INFO] recording %s, clip %d/%d y=%+d x_axis %.2f -> %.2fmm\n",
           video_path.c_str(),
           clip_index + 1,
           clip_count,
           y_degree,
           start_axis_x,
           target_axis_x);

    int64_t start_us = monotonic_time_us();
    int64_t last_control_us = 0;
    bool ok = true;
    bool clip_complete = false;
    runtime->last_correction_us = start_us;
    runtime->joint_target_valid = false;

    while (running) {
        int64_t now_us = monotonic_time_us();
        double elapsed_ms = (double)(now_us - start_us) / 1000.0;
        if (elapsed_ms > (double)timeout_ms) {
            fprintf(stderr,
                    "[ERROR] half-cycle timeout before x boundary: "
                    "elapsed=%.0fms timeout=%lldms\n",
                    elapsed_ms,
                    (long long)timeout_ms);
            ok = false;
            break;
        }

        if (last_control_us == 0 ||
            now_us - last_control_us >= ARM_CORRECTION_PERIOD_MS * 1000LL) {
            bool reached_boundary = false;
            if (!update_arm_sine_motion(motor_uart,
                                        runtime,
                                        clip_index,
                                        start_axis_x,
                                        target_axis_x,
                                        patrol_dir,
                                        &reached_boundary)) {
                ok = false;
                break;
            }
            last_control_us = now_us;
            if (reached_boundary) {
                clip_complete = true;
            }
        }

        cv::Mat bgr_frame;
        if (!read_camera_bgr(camera, &bgr_frame)) {
            fprintf(stderr, "[ERROR] camera read failed while recording\n");
            ok = false;
            break;
        }

        writer.write(bgr_frame);

        std::ostringstream line1;
        std::ostringstream line2;
        line1 << "REC " << (clip_index + 1) << "/" << clip_count
              << " y=" << (y_degree >= 0 ? "+" : "") << y_degree;
        line2 << "x " << (int)start_axis_x << "->" << (int)target_axis_x
              << " z sine";
        show_frame(lcd, bgr_frame, line1.str(), line2.str());

        if (clip_complete) {
            break;
        }
    }

    writer.release();
    stop_arm_motion(motor_uart, runtime);

    if (!ok || !running || !clip_complete) {
        remove_partial_video(video_path);
        return false;
    }

    printf("[INFO] saved %s\n", video_path.c_str());
    return true;
}

static void print_usage(const char *program)
{
    printf("usage: %s\n", program);
    printf("  camera, LCD preview and video recording are always enabled\n");
}

int main(int argc, char **argv)
{
    bool lcd_ready = false;
    cv::VideoCapture camera;
    cv::VideoCapture *camera_ptr = NULL;
    icst7735_t lcd;
    icst7735_t *lcd_ptr = NULL;

    setvbuf(stdout, NULL, _IOLBF, 0);

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--camera") == 0) {
            continue;
        } else if (std::strcmp(argv[i], "--no-camera") == 0) {
            fprintf(stderr, "[ERROR] --no-camera is disabled for this test\n");
            print_usage(argv[0]);
            return 1;
        } else if (std::strcmp(argv[i], "--help") == 0 ||
                   std::strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "[ERROR] unknown argument: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    if (install_signal_handlers() != 0) {
        return 1;
    }
    if (!ensure_save_dir()) {
        return 1;
    }

    std::memset(&lcd, 0, sizeof(lcd));
    printf("[INFO] camera mode enabled\n");
    if (!open_camera(&camera)) {
        return 1;
    }
    camera_ptr = &camera;

    printf("[INFO] init lcd spi=%d dc=%d rst=%d bl=%d\n",
           (int)LS_SPI2,
           (int)LCD_DC_PIN,
           (int)LCD_RST_PIN,
           (int)LCD_BL_PIN);
    if (icst7735_init(&lcd,
                      LS_SPI2,
                      80000000U,
                      LCD_DC_PIN,
                      LCD_RST_PIN,
                      LCD_BL_PIN,
                      160,
                      128,
                      0,
                      0) != 0) {
        fprintf(stderr, "[ERROR] could not init lcd\n");
        camera.release();
        return 1;
    }
    lcd_ready = true;
    lcd_ptr = &lcd;

    ls2k0300_uart_t motor_uart;
    std::memset(&motor_uart, 0, sizeof(motor_uart));
    printf("[INFO] init motor uart %s\n", ARM_MOTOR_UART);
    if (ls2k0300_uart_init(&motor_uart,
                           ARM_MOTOR_UART,
                           ARM_MOTOR_BAUD,
                           LS_UART_STOP1,
                           LS_UART_DATA8,
                           LS_UART_PARITY_NONE,
                           LS_UART_MODE_NON_BLOCKING) != 0) {
        fprintf(stderr, "[ERROR] failed to init motor uart: %s\n",
                ARM_MOTOR_UART);
        if (lcd_ready) {
            icst7735_deinit(&lcd);
        }
        if (camera.isOpened()) {
            camera.release();
        }
        return 1;
    }

    ArmRuntime runtime;
    std::memset(&runtime, 0, sizeof(runtime));
    runtime.theta1 = ARM_INITIAL_THETA1_RAD;
    runtime.theta2 = ARM_INITIAL_THETA2_RAD;
    runtime.theta3 = ARM_INITIAL_THETA3_RAD;
    runtime.last_correction_us = monotonic_time_us();
    zero_speeds(&runtime.last_speeds);

    printf("[INFO] send initial arm stop\n");
    stop_arm_motion(&motor_uart, &runtime);
    sleep_ms(100);

    printf("[INFO] arm sine video test start\n");
    printf("[INFO] videos: %s, half_period=%lldms, z_amp=%.1fmm\n",
           VIDEO_SAVE_DIR,
           (long long)ARM_SINE_HALF_PERIOD_MS,
           ARM_SINE_Z_AMPLITUDE_MM);

    printf("[INFO] startup home arm motors\n");
    if (!home_arm_motors_nearest(&motor_uart)) {
        fprintf(stderr, "[ERROR] arm home failed\n");
        stop_arm_motion(&motor_uart, &runtime);
        ls2k0300_uart_deinit(&motor_uart);
        if (lcd_ready) {
            icst7735_deinit(&lcd);
        }
        if (camera.isOpened()) {
            camera.release();
        }
        return 1;
    }
    printf("[INFO] startup calibrate arm zero\n");
    if (!calibrate_runtime_motor_zero(&motor_uart, &runtime)) {
        fprintf(stderr, "[ERROR] arm zero calibration failed\n");
        stop_arm_motion(&motor_uart, &runtime);
        ls2k0300_uart_deinit(&motor_uart);
        if (lcd_ready) {
            icst7735_deinit(&lcd);
        }
        if (camera.isOpened()) {
            camera.release();
        }
        return 1;
    }
    printf("[INFO] startup move to theta 0,0,120\n");
    if (!move_arm_to_startup_pose(&motor_uart, &runtime)) {
        fprintf(stderr, "[ERROR] arm startup failed\n");
        stop_arm_motion(&motor_uart, &runtime);
        if (running && runtime.motor_zero_valid) {
            (void)move_arm_to_shutdown_pose(&motor_uart, &runtime);
            stop_arm_motion(&motor_uart, &runtime);
        }
        ls2k0300_uart_deinit(&motor_uart);
        if (lcd_ready) {
            icst7735_deinit(&lcd);
        }
        if (camera.isOpened()) {
            camera.release();
        }
        return 1;
    }

    stop_arm_motion(&motor_uart, &runtime);
    if (!show_live_for_ms(camera_ptr,
                          lcd_ptr,
                          VIDEO_HOLD_BEFORE_RECORD_MS,
                          "STARTUP HOLD theta 0,0,120",
                          "wait 2s before motion")) {
        stop_arm_motion(&motor_uart, &runtime);
        if (running && runtime.motor_zero_valid) {
            (void)move_arm_to_shutdown_pose(&motor_uart, &runtime);
            stop_arm_motion(&motor_uart, &runtime);
        }
        ls2k0300_uart_deinit(&motor_uart);
        if (lcd_ready) {
            icst7735_deinit(&lcd);
        }
        if (camera.isOpened()) {
            camera.release();
        }
        return 1;
    }

    const std::vector<int> y_degrees = make_y_scan_degrees();
    int current_y_degree = 0;
    double patrol_dir = 1.0;
    bool clips_ok = true;

    for (size_t i = 0; running && i < y_degrees.size(); ++i) {
        int target_y_degree = y_degrees[i];
        bool y_changed = (target_y_degree != current_y_degree);
        if (!move_y_to_degree(&motor_uart,
                              &runtime,
                              current_y_degree,
                              target_y_degree)) {
            fprintf(stderr, "[ERROR] y move failed\n");
            clips_ok = false;
            break;
        }
        current_y_degree = target_y_degree;

        if (y_changed) {
            std::ostringstream hold_line1;
            std::ostringstream hold_line2;
            hold_line1 << "HOLD " << (i + 1) << "/" << y_degrees.size()
                       << " y=" << (target_y_degree >= 0 ? "+" : "")
                       << target_y_degree;
            hold_line2 << "wait 2s before record";
            if (!show_live_for_ms(camera_ptr,
                                  lcd_ptr,
                                  VIDEO_HOLD_BEFORE_RECORD_MS,
                                  hold_line1.str(),
                                  hold_line2.str())) {
                clips_ok = false;
                break;
            }
        }

        std::string video_path;
        int latest_sequence = find_latest_video_sequence(VIDEO_SAVE_DIR);
        int next_sequence = latest_sequence + 1;
        video_path = make_video_path(target_y_degree, next_sequence);
        printf("[INFO] latest video sequence=%d, next=%d\n",
               latest_sequence,
               next_sequence);

        if (!record_half_cycle(camera_ptr,
                               lcd_ptr,
                               &motor_uart,
                               &runtime,
                               &patrol_dir,
                               target_y_degree,
                               (int)i,
                               (int)y_degrees.size(),
                               video_path)) {
            clips_ok = false;
            break;
        }
    }

    stop_arm_motion(&motor_uart, &runtime);
    if (running && runtime.motor_zero_valid) {
        if (!move_arm_to_shutdown_pose(&motor_uart, &runtime)) {
            fprintf(stderr, "[ERROR] shutdown return failed\n");
            clips_ok = false;
        }
        stop_arm_motion(&motor_uart, &runtime);
    }
    ls2k0300_uart_deinit(&motor_uart);
    if (lcd_ready) {
        icst7735_deinit(&lcd);
    }
    if (camera.isOpened()) {
        camera.release();
    }

    printf("[INFO] arm sine video test finished\n");
    return (running && clips_ok) ? 0 : 1;
}
