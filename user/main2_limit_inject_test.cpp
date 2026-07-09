#include "ArmMotorPosition.h"
#include "ArmKinematics.h"
#include "All_control.h"
#include "AppLog.h"

#include "LS2K0300_UART.h"
#include "X42_V2.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <unistd.h>
#include <pthread.h>

/* 串口分配 */
#define ARM_VISION_UART UART1 /* main1 与 main2 的通讯串口 */
#define ARM_MOTOR_UART UART4  /* 机械臂电机与注射电机共用控制串口 */

/* 机械臂三轴电机地址 */
#define ARM_MOTOR_ADDR_1 1U /* 基座旋转电机地址，启动时参与就近回零 */
#define ARM_MOTOR_ADDR_2 2U /* 机械臂第二轴电机地址 */
#define ARM_MOTOR_ADDR_3 3U /* 机械臂第三轴电机地址 */

/* 电机输出轴到关节角的减速比 */
#define ARM_GEAR_RATIO_1 (32.0/9.0) /* 第一轴减速比 */
#define ARM_GEAR_RATIO_2 (32.0/9.0) /* 第二轴减速比 */
#define ARM_GEAR_RATIO_3 (32.0/9.0) /* 第三轴减速比 */

/* 上电标定时对应的机械臂实际零位角，单位 rad */
#define ARM_INITIAL_THETA1_RAD 0.0                     /* 第一轴上电标定角：0 deg */
#define ARM_INITIAL_THETA2_RAD (90.0 * M_PI / 180.0)  /* 第二轴上电标定角：90 deg */
#define ARM_INITIAL_THETA3_RAD (180.0 * M_PI / 180.0) /* 第三轴上电标定角：180 deg */

/* 启动回零并完成上电标定后，要运动到的工作初始姿态，单位 rad */
#define ARM_STARTUP_TARGET_THETA1_RAD 0.0                    /* 第一轴启动目标角：0 deg */
#define ARM_STARTUP_TARGET_THETA2_RAD 0.0                    /* 第二轴启动目标角：0 deg */
#define ARM_STARTUP_TARGET_THETA3_RAD (140.0 * M_PI / 180.0) /* 第三轴启动目标角：140 deg */

/* 程序退出时按角度退回的目标姿态，默认回到上电标定零位，单位 rad */
#define ARM_SHUTDOWN_TARGET_THETA1_RAD ARM_INITIAL_THETA1_RAD /* 退出目标第一轴角 */
#define ARM_SHUTDOWN_TARGET_THETA2_RAD ARM_INITIAL_THETA2_RAD /* 退出目标第二轴角 */
#define ARM_SHUTDOWN_TARGET_THETA3_RAD ARM_INITIAL_THETA3_RAD /* 退出目标第三轴角 */

/* 电机角度增量到关节角增量的方向修正 */
#define ARM_JOINT_DIRECTION_1 1.0  /* 第一轴正方向修正系数 */
#define ARM_JOINT_DIRECTION_2 -1.0 /* 第二轴实时角换算为 theta2 时按 0626 方向取反 */
#define ARM_JOINT_DIRECTION_3 1.0  /* 第三轴正方向修正系数 */

/* 末端速度解算与闭环修正参数 */
#define ARM_ENDPOINT_SPEED_SCALE 0.2      /* main1 发送速度到末端速度的缩放系数 */
#define ARM_ENDPOINT_X_DIRECTION 1.0      /* 实际机械臂 x 正方向相对运动学模型 x 的符号 */
#define ARM_MAX_ENDPOINT_SPEED_MMPS 10.0  /* 末端速度限幅，单位 mm/s */
#define ARM_MAX_MOTOR_RPM 30.0f           /* 三轴电机转速限幅，单位 RPM */
#define ARM_VELOCITY_RAMP_RPMPS 20U       /* X42 速度模式加速度/斜率，单位 RPM/s */
#define ARM_STALE_STOP_TIMEOUT_MS 700LL   /* 超过该时间未收到视觉速度则停止，单位 ms */
#define ARM_CORRECTION_PERIOD_MS 30LL     /* 机械臂速度闭环修正周期，单位 ms */
#define ARM_POSITION_PRINT_PERIOD_MS 300LL /* 实时位置与速度日志打印周期，单位 ms */
#define ARM_STATE_PRINT_PERIOD_MS 1000LL   /* 主状态机心跳日志打印周期，单位 ms */
#define ARM_VERBOSE_MOTION_LOG 0           /* 仅调试速度/位置/电机输出时打开 */
#define ARM_JOINT_ERROR_GAIN 1.0          /* 关节目标角与实测角误差修正增益 */
#define ARM_MAX_CORRECTION_RADPS 3.0      /* 单轴最大误差修正角速度，单位 rad/s */

/* 机械臂末端空间限位参数，坐标单位 mm，z=0 为第二轴/下臂旋转轴心 */
#define ARM_LIMIT_LOW_Z_MIN_MM -110.0
#define ARM_LIMIT_LOW_Z_MAX_MM -30.0
#define ARM_LIMIT_LOW_Z_MIN_X_MM 110.0
#define ARM_LIMIT_Z90_MM 90.0
#define ARM_LIMIT_Z90_MAX_X_MM 145.0
#define ARM_LIMIT_Z100_MM 100.0
#define ARM_LIMIT_Z100_MAX_X_MM 125.0
#define ARM_LIMIT_MAX_X_MM 200.0
#define ARM_LIMIT_MAX_Z_MM 200.0

/* 限位命中标志位，用于日志打印 */
#define ARM_LIMIT_FLAG_LOW_Z_NEED_X 0x01U
#define ARM_LIMIT_FLAG_Z90_MAX_X 0x02U
#define ARM_LIMIT_FLAG_Z100_MAX_X 0x04U
#define ARM_LIMIT_FLAG_MAX_X 0x08U
#define ARM_LIMIT_FLAG_MAX_Z 0x10U

/* X42 回零与状态读取协议参数 */
#define ARM_ORIGIN_MODE_NEAREST_ZERO 0x00U       /* 回零模式：就近回零 */
#define ARM_ORIGIN_COMMAND_CODE 0x9AU            /* 触发回零命令功能码 */
#define ARM_ORIGIN_STATUS_CODE 0x3BU             /* 读取回零状态功能码 */
#define ARM_POSITION_COMMAND_CODE 0xFBU          /* 直通限速位置模式命令功能码 */
#define ARM_MOTOR_STATUS_CODE 0x3AU              /* 读取电机状态标志功能码 */
#define ARM_X42_CHECK_BYTE 0x6BU                 /* X42 协议固定校验字节 */
#define ARM_COMMAND_STATUS_DONE 0x02U            /* 命令接收正确 */
#define ARM_COMMAND_STATUS_ALREADY_DONE 0x12U    /* 已在零点或限位处，无需运动 */
#define ARM_COMMAND_STATUS_ACTION_DONE 0x9FU     /* 动作执行完成主动返回 */
#define ARM_COMMAND_STATUS_PARAM_ERROR 0xE2U     /* 命令参数错误或条件不满足 */
#define ARM_COMMAND_STATUS_FORMAT_ERROR 0xEEU    /* 命令格式错误 */
#define ARM_ORIGIN_READY_MASK 0x03U              /* Enc_Rdy 和 Cal_Rdy 掩码 */
#define ARM_ORIGIN_READY_VALUE 0x03U             /* 编码器和校准均就绪 */
#define ARM_ORIGIN_STATE_MASK 0x0CU              /* Org_SF/Org_CF 状态位掩码 */
#define ARM_ORIGIN_STATE_BUSY 0x04U              /* 正在回零 */
#define ARM_ORIGIN_STATE_FAILED 0x08U            /* 回零失败 */
#define ARM_ORIGIN_STATE_DONE 0x00U              /* 回零完成状态 */
#define ARM_ORIGIN_NEAREST_DONE_STATUS 0x0BU     /* 当前电机就近回零完成时观测到的状态值 */
#define ARM_ORIGIN_TRIGGER_RESPONSE_TIMEOUT_MS 200 /* 触发回零命令应答超时，单位 ms */
#define ARM_ORIGIN_STATUS_RESPONSE_TIMEOUT_MS 200  /* 读取回零状态应答超时，单位 ms */
#define ARM_ORIGIN_FIRST_POLL_DELAY_MS 1         /* 触发回零后首次查询延时，单位 ms */
#define ARM_ORIGIN_STATUS_POLL_MS 20             /* 回零状态轮询周期，单位 ms */
#define ARM_ORIGIN_WAIT_TIMEOUT_MS 15000LL       /* 单轴回零总等待超时，单位 ms */
#define ARM_ORIGIN_STABLE_DONE_POLLS 3U          /* 连续完成判定次数，防止瞬时误判 */
#define ARM_POSITION_MODE_REL_CURRENT 0x02U      /* 位置模式：相对当前实时位置运动 */
#define ARM_POSITION_RESPONSE_TIMEOUT_MS 200     /* 位置模式命令应答超时，单位 ms */
#define ARM_POSITION_STATUS_RESPONSE_TIMEOUT_MS 200 /* 读取位置到位状态应答超时，单位 ms */
#define ARM_POSITION_STATUS_POLL_MS 20           /* 位置到位状态轮询周期，单位 ms */
#define ARM_POSITION_WAIT_TIMEOUT_MS 15000LL     /* 单轴位置运动到位总超时，单位 ms */
#define ARM_POSITION_STABLE_DONE_POLLS 3U        /* 连续到位判定次数，防止瞬时误判 */
#define ARM_POSITION_ARRIVED_MASK 0x02U          /* 电机状态标志 bit1：位置到位 */
#define ARM_POSITION_MOVE_SPEED_RPM 10.0f        /* 启动和退出角度回零位置模式最大速度，单位 RPM */
#define ARM_STARTUP_POSITION_SKIP_EPS_DEG 0.2    /* 电机相对角小于该值时跳过运动，单位 deg */
#define ARM_STARTUP_JOINT_ARRIVE_EPS_DEG 2.0     /* 启动目标姿态允许关节误差，单位 deg */
#define ARM_SHUTDOWN_JOINT_ARRIVE_EPS_DEG 2.0    /* 退出角度回零允许关节误差，单位 deg */
#define ARM_SHUTDOWN_STAGE_X_EPS_MM 3.0          /* 退出第一段 x 到位确认容差，单位 mm */

/* 主流程巡航和退出角度回零参数 */
#define ARM_PATROL_X_SPEED_MMPS 10.0       /* 未检测目标时沿 x 往返巡航速度，单位 mm/s */
#define ARM_PATROL_MIN_X_MM 100.0          /* 巡航反向的最小 x 位置 */
#define ARM_PATROL_MAX_X_MM 200.0          /* 巡航反向的最大 x 位置 */
#define ARM_PATROL_X_TOLERANCE_MM 4.0      /* 巡航到边界后的反向容差，单位 mm */
#define ARM_POST_INJECT_PATROL_MS 2000LL   /* 注射结束后继续 x 往返巡航时间，单位 ms */
#define ARM_SHUTDOWN_TIMEOUT_MS 30000LL    /* 退出角度回零流程总超时，单位 ms */

/* 注射流程参数 */
#define INJECT_NEEDLE_DIRECTION 0U          /* 注射进针方向，按现场电机方向定义 */
#define INJECT_PUSH_VOLUME_ML 1.0f          /* 推药剂体积，单位 ml */
#define INJECT_PUSH_WAIT_TIMEOUT_MS 20000LL /* 主推药电机推药到位等待超时，单位 ms */

typedef enum {
    ARM_STATE_WAIT_HANDSHAKE = 0,
    ARM_STATE_PATROL_X,
    ARM_STATE_TRACK_TARGET,
    ARM_STATE_INJECTION,
    ARM_STATE_POST_INJECT_PATROL,
    ARM_STATE_SHUTDOWN_RETURN_SAFE,
} ArmControlState;

static const char *arm_state_name(ArmControlState state)
{
    switch (state) {
    case ARM_STATE_WAIT_HANDSHAKE:
        return "WAIT_HANDSHAKE";
    case ARM_STATE_PATROL_X:
        return "PATROL_X";
    case ARM_STATE_TRACK_TARGET:
        return "TRACK_TARGET";
    case ARM_STATE_INJECTION:
        return "INJECTION";
    case ARM_STATE_POST_INJECT_PATROL:
        return "POST_INJECT_PATROL";
    case ARM_STATE_SHUTDOWN_RETURN_SAFE:
        return "SHUTDOWN_RETURN_SAFE";
    default:
        return "UNKNOWN";
    }
}

typedef enum {
    INJECT_SEQUENCE_SUCCESS = 0,
    INJECT_SEQUENCE_FAIL,
    INJECT_SEQUENCE_TARGET_LOST,
    INJECT_SEQUENCE_INTERRUPTED,
} InjectSequenceResult;

static const char main1_handshake_cmd[] = "MAIN1_HELLO";
static const char main2_handshake_ack[] = "MAIN2_ACK\n";
static const char main1_exit_cmd[] = "MAIN1_EXIT";
static const char main2_exit_cmd[] = "MAIN2_EXIT\n";
static const char cmd_inject_start[] = "CMD:INJECT_START";
static const char cmd_target_lost[] = "CMD:TARGET_LOST";
static const char cmd_detect_pause[] = "CMD:DETECT_PAUSE\n";
static const char cmd_detect_resume[] = "CMD:DETECT_RESUME\n";
static const char cmd_detect_5x5[] = "CMD:DETECT_5X5\n";
static const char cmd_detect_150x150[] = "CMD:DETECT_150X150\n";
static const char cmd_inject_done[] = "CMD:INJECT_DONE\n";
static const char cmd_inject_fail[] = "CMD:INJECT_FAIL\n";

static volatile std::sig_atomic_t running = 1;
static volatile std::sig_atomic_t shutdown_requested = 0;
static volatile std::sig_atomic_t graceful_shutdown_ready = 0;
static volatile std::sig_atomic_t injection_running = 1;
static volatile std::sig_atomic_t limit_inject_test_ignore_main1 = 1;
static pthread_mutex_t control_mutex = PTHREAD_MUTEX_INITIALIZER;

typedef struct {
    double theta1;
    double theta2;
    double theta3;
    double motor2_zero_degrees;
    double motor3_zero_degrees;
    double target_vx;
    double target_vy;
    double target_vz;
    double target_theta2;
    double target_theta3;
    int64_t last_correction_us;
    int64_t last_position_print_ms;
    JointMotorSpeeds last_speeds;
    int64_t last_rx_ms;
    bool stale_stop_sent;
    bool motor_zero_valid;
    bool target_velocity_valid;
    bool target_velocity_dirty;
    bool joint_target_valid;
    bool vision_velocity_valid;
    bool handshake_done;
    bool inject_start_requested;
    bool target_lost_requested;
} ArmRuntime;

typedef struct {
    ls2k0300_uart_t *vision_uart;
    ls2k0300_uart_t *motor_uart;
    ArmRuntime *runtime;
} ArmThreadContext;

static void signal_handler(int signum)
{
    (void)signum;
    if (shutdown_requested || !graceful_shutdown_ready) {
        running = 0;
    }
    shutdown_requested = 1;
    injection_running = 0;
}

static bool uart_write_all(ls2k0300_uart_t *uart, const char *text)
{
    const uint8_t *data;
    size_t total;
    size_t written_total = 0U;
    unsigned int retry_count = 0U;

    if (uart == NULL || text == NULL) {
        return false;
    }

    data = (const uint8_t *)text;
    total = strlen(text);
    while (written_total < total && retry_count < 50U) {
        ssize_t written =
            ls2k0300_uart_write(uart,
                                data + written_total,
                                total - written_total);
        if (written > 0) {
            written_total += (size_t)written;
            retry_count = 0U;
            continue;
        }

        ++retry_count;
        usleep(1000);
    }

    if (written_total != total) {
        APP_LOG_WARN("uart write incomplete: %zu/%zu", written_total, total);
        return false;
    }

    return true;
}

static void notify_main1_exit(ls2k0300_uart_t *vision_uart)
{
    if (vision_uart == NULL) {
        return;
    }

    for (int i = 0; i < 3; ++i) {
        (void)uart_write_all(vision_uart, main2_exit_cmd);
        usleep(10000);
    }
}

static void send_main1_command(ls2k0300_uart_t *vision_uart, const char *cmd)
{
    if (vision_uart == NULL || cmd == NULL) {
        return;
    }

    if (uart_write_all(vision_uart, cmd)) {
        APP_LOG_INFO("main2 tx command: %s", cmd);
    }
}

static void send_main1_command_repeat(ls2k0300_uart_t *vision_uart,
                                      const char *cmd,
                                      unsigned int count,
                                      useconds_t interval_us)
{
    for (unsigned int i = 0U; i < count; ++i) {
        send_main1_command(vision_uart, cmd);
        if (i + 1U < count) {
            usleep(interval_us);
        }
    }
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

static ssize_t read_uart_byte_timeout(ls2k0300_uart_t *uart, uint8_t *byte,
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

static void print_x42_frame(const char *prefix, uint8_t addr,
                            const uint8_t *buffer, size_t size)
{
    char label[64];

    snprintf(label, sizeof(label), "%s addr=%u", prefix, (unsigned int)addr);
    app_log_bytes("INFO", label, buffer, size);
}

static ssize_t read_x42_4byte_response_timeout(ls2k0300_uart_t *uart,
                                               uint8_t addr,
                                               uint8_t code,
                                               uint8_t response[4],
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
    while (running && monotonic_time_ms() < deadline_ms) {
        uint8_t byte = 0U;
        ssize_t n = read_uart_byte_timeout(uart, &byte, deadline_ms);

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

    if (running) {
        print_x42_frame("invalid x42 rx", addr, debug_buffer, debug_size);
    }
    return 0;
}

static void zero_speeds(JointMotorSpeeds *speeds)
{
    memset(speeds, 0, sizeof(*speeds));
    speeds->valid = 1;
}

static bool arm_origin_status_done(uint8_t status)
{
    return ((status & ARM_ORIGIN_STATE_MASK) == ARM_ORIGIN_STATE_DONE) &&
           ((status & ARM_ORIGIN_READY_MASK) == ARM_ORIGIN_READY_VALUE);
}

static bool read_arm_origin_status(ls2k0300_uart_t *motor_uart,
                                   uint8_t addr,
                                   uint8_t *status)
{
    uint8_t response[4] = {0};
    ssize_t response_size;

    if (motor_uart == NULL || status == NULL) {
        return false;
    }

    if (ls2k0300_uart_flush(motor_uart) != 0) {
        return false;
    }

    ZDT_X42_V2_Read_Sys_Params(motor_uart, addr, S_OFLAG);
    response_size = read_x42_4byte_response_timeout(
        motor_uart,
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

static bool home_arm_motor_nearest(ls2k0300_uart_t *motor_uart,
                                   uint8_t addr)
{
    uint8_t response[4] = {0};
    ssize_t response_size;
    uint8_t stable_done_count = 0U;
    uint8_t last_printed_status = 0xFFU;
    int64_t start_ms;

    if (motor_uart == NULL || !running || shutdown_requested) {
        return false;
    }

    if (ls2k0300_uart_flush(motor_uart) != 0) {
        return false;
    }

    APP_LOG_INFO("arm motor%u nearest home: send 9A mode 00",
                 (unsigned int)addr);
    ZDT_X42_V2_Origin_Trigger_Return(motor_uart,
                                     addr,
                                     ARM_ORIGIN_MODE_NEAREST_ZERO,
                                     false);

    response_size = read_x42_4byte_response_timeout(
        motor_uart,
        addr,
        ARM_ORIGIN_COMMAND_CODE,
        response,
        ARM_ORIGIN_TRIGGER_RESPONSE_TIMEOUT_MS);
    if (response_size == 4 &&
        response[0] == addr &&
        response[1] == ARM_ORIGIN_COMMAND_CODE &&
        response[3] == ARM_X42_CHECK_BYTE) {
        print_x42_frame("origin trigger rx", addr, response,
                        (size_t)response_size);
        if (response[2] == ARM_COMMAND_STATUS_ALREADY_DONE ||
            response[2] == ARM_COMMAND_STATUS_ACTION_DONE) {
            APP_LOG_INFO("arm motor%u nearest home done by command status 0x%02X",
                         (unsigned int)addr,
                         response[2]);
            return true;
        }
        if (response[2] == ARM_COMMAND_STATUS_PARAM_ERROR ||
            response[2] == ARM_COMMAND_STATUS_FORMAT_ERROR) {
            APP_LOG_ERROR("arm motor%u nearest home command failed, status=0x%02X",
                          (unsigned int)addr,
                          response[2]);
            return false;
        }
    }

    usleep(ARM_ORIGIN_FIRST_POLL_DELAY_MS * 1000);
    start_ms = monotonic_time_ms();

    while (running && !shutdown_requested) {
        uint8_t status;
        uint8_t origin_state;

        if (monotonic_time_ms() - start_ms > ARM_ORIGIN_WAIT_TIMEOUT_MS) {
            APP_LOG_WARN("arm motor%u nearest home timeout", (unsigned int)addr);
            return false;
        }

        if (!read_arm_origin_status(motor_uart, addr, &status)) {
            usleep(ARM_ORIGIN_STATUS_POLL_MS * 1000);
            continue;
        }

        if (status != last_printed_status) {
            APP_LOG_INFO("arm motor%u origin status=0x%02X",
                         (unsigned int)addr, status);
            last_printed_status = status;
        }

        if (status == ARM_ORIGIN_NEAREST_DONE_STATUS) {
            APP_LOG_INFO("arm motor%u nearest home done, status=0x%02X",
                         (unsigned int)addr, status);
            return true;
        }

        origin_state = (uint8_t)(status & ARM_ORIGIN_STATE_MASK);
        if (origin_state == ARM_ORIGIN_STATE_BUSY) {
            stable_done_count = 0U;
        } else if (origin_state == ARM_ORIGIN_STATE_FAILED) {
            APP_LOG_ERROR("arm motor%u nearest home failed, status=0x%02X",
                          (unsigned int)addr, status);
            return false;
        } else if (arm_origin_status_done(status)) {
            stable_done_count++;
            if (stable_done_count >= ARM_ORIGIN_STABLE_DONE_POLLS) {
                APP_LOG_INFO("arm motor%u nearest home done",
                             (unsigned int)addr);
                return true;
            }
        } else {
            stable_done_count = 0U;
        }

        usleep(ARM_ORIGIN_STATUS_POLL_MS * 1000);
    }

    return false;
}

static bool home_arm_motors_nearest(ls2k0300_uart_t *motor_uart)
{
    const uint8_t addrs[] = {
        ARM_MOTOR_ADDR_1,
        ARM_MOTOR_ADDR_2,
        ARM_MOTOR_ADDR_3,
    };

    APP_LOG_INFO("main2 startup: nearest home arm motors before control");
    for (size_t i = 0U; i < sizeof(addrs) / sizeof(addrs[0]); ++i) {
        if (!running || shutdown_requested) {
            return false;
        }
        if (!home_arm_motor_nearest(motor_uart, addrs[i])) {
            return false;
        }
        usleep(1000);
    }

    APP_LOG_INFO("main2 startup: all arm motors nearest home done");
    return true;
}

static bool read_motor23_degrees(ls2k0300_uart_t *motor_uart,
                                 double *motor2_degrees,
                                 double *motor3_degrees)
{
    if (!arm_read_motor_position_degrees(motor_uart, ARM_MOTOR_ADDR_2,
                                         motor2_degrees,
                                         ARM_MOTOR_POSITION_TIMEOUT_MS)) {
        return false;
    }

    if (!arm_read_motor_position_degrees(motor_uart, ARM_MOTOR_ADDR_3,
                                         motor3_degrees,
                                         ARM_MOTOR_POSITION_TIMEOUT_MS)) {
        return false;
    }

    return true;
}

static bool read_runtime_joint_angles(ls2k0300_uart_t *motor_uart,
                                      ArmRuntime *runtime)
{
    double motor2_degrees;
    double motor3_degrees;

    if (!runtime->motor_zero_valid) {
        return false;
    }

    if (!read_motor23_degrees(motor_uart, &motor2_degrees, &motor3_degrees)) {
        return false;
    }

    double motor2_delta = motor2_degrees - runtime->motor2_zero_degrees;
    double motor3_delta = motor3_degrees - runtime->motor3_zero_degrees;

    runtime->theta1 = ARM_INITIAL_THETA1_RAD;
    runtime->theta2 = ARM_INITIAL_THETA2_RAD +
                      ARM_JOINT_DIRECTION_2 *
                      (motor2_delta * M_PI / 180.0) / ARM_GEAR_RATIO_2;
    runtime->theta3 = ARM_INITIAL_THETA3_RAD +
                      ARM_JOINT_DIRECTION_3 *
                      (motor3_delta * M_PI / 180.0) / ARM_GEAR_RATIO_3;
    return true;
}

static bool calibrate_runtime_motor_zero(ls2k0300_uart_t *motor_uart,
                                         ArmRuntime *runtime)
{
    if (!read_motor23_degrees(motor_uart,
                              &runtime->motor2_zero_degrees,
                              &runtime->motor3_zero_degrees)) {
        runtime->motor_zero_valid = false;
        return false;
    }

    runtime->theta1 = ARM_INITIAL_THETA1_RAD;
    runtime->theta2 = ARM_INITIAL_THETA2_RAD;
    runtime->theta3 = ARM_INITIAL_THETA3_RAD;
    runtime->motor_zero_valid = true;

    APP_LOG_INFO("startup motor zero: M2=%.2fdeg -> theta2=%.2fdeg, "
                 "M3=%.2fdeg -> theta3=%.2fdeg",
                 runtime->motor2_zero_degrees,
                 ARM_INITIAL_THETA2_RAD * 180.0 / M_PI,
                 runtime->motor3_zero_degrees,
                 ARM_INITIAL_THETA3_RAD * 180.0 / M_PI);
    return true;
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

static bool read_arm_motor_status(ls2k0300_uart_t *motor_uart,
                                  uint8_t addr,
                                  uint8_t *status)
{
    uint8_t response[4] = {0};
    ssize_t response_size;

    if (motor_uart == NULL || status == NULL) {
        return false;
    }

    if (ls2k0300_uart_flush(motor_uart) != 0) {
        return false;
    }

    ZDT_X42_V2_Read_Sys_Params(motor_uart, addr, S_SFLAG);
    response_size = read_x42_4byte_response_timeout(
        motor_uart,
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

static bool wait_arm_position_arrived(ls2k0300_uart_t *motor_uart,
                                      uint8_t addr,
                                      const char *name)
{
    uint8_t stable_arrived_count = 0U;
    uint8_t last_printed_status = 0xFFU;
    int64_t start_ms = monotonic_time_ms();

    while (running) {
        uint8_t status;

        if (monotonic_time_ms() - start_ms > ARM_POSITION_WAIT_TIMEOUT_MS) {
            APP_LOG_WARN("%s position arrive timeout", name);
            return false;
        }

        if (!read_arm_motor_status(motor_uart, addr, &status)) {
            usleep(ARM_POSITION_STATUS_POLL_MS * 1000);
            continue;
        }

        if (status != last_printed_status) {
            APP_LOG_INFO("%s motor status=0x%02X", name, status);
            last_printed_status = status;
        }

        if ((status & ARM_POSITION_ARRIVED_MASK) != 0U) {
            stable_arrived_count++;
            if (stable_arrived_count >= ARM_POSITION_STABLE_DONE_POLLS) {
                APP_LOG_INFO("%s position arrived", name);
                return true;
            }
        } else {
            stable_arrived_count = 0U;
        }

        usleep(ARM_POSITION_STATUS_POLL_MS * 1000);
    }

    return false;
}

static bool start_arm_relative_position_degrees(ls2k0300_uart_t *motor_uart,
                                                uint8_t addr,
                                                const char *name,
                                                double motor_delta_degrees,
                                                bool *needs_wait)
{
    uint8_t response[4] = {0};
    ssize_t response_size;
    uint8_t dir;
    float abs_degrees;

    if (motor_uart == NULL || name == NULL || needs_wait == NULL) {
        return false;
    }

    *needs_wait = false;
    if (std::fabs(motor_delta_degrees) <=
        ARM_STARTUP_POSITION_SKIP_EPS_DEG) {
        APP_LOG_INFO("%s startup position skip, motor delta=%.2fdeg",
                     name, motor_delta_degrees);
        return true;
    }

    dir = arm_position_dir_from_delta(motor_delta_degrees);
    abs_degrees = (float)std::fabs(motor_delta_degrees);

    if (ls2k0300_uart_flush(motor_uart) != 0) {
        return false;
    }

    APP_LOG_INFO("%s startup relative position: motor_delta=%.2fdeg, dir=%u",
                 name, motor_delta_degrees, (unsigned int)dir);
    ZDT_X42_V2_Bypass_Position_LV_Control(motor_uart,
                                          addr,
                                          dir,
                                          ARM_POSITION_MOVE_SPEED_RPM,
                                          abs_degrees,
                                          ARM_POSITION_MODE_REL_CURRENT,
                                          0U);

    response_size = read_x42_4byte_response_timeout(
        motor_uart,
        addr,
        ARM_POSITION_COMMAND_CODE,
        response,
        ARM_POSITION_RESPONSE_TIMEOUT_MS);
    if (response_size != 4) {
        *needs_wait = true;
        return true;
    }

    print_x42_frame("startup position rx", addr, response,
                    (size_t)response_size);
    if (response[0] != addr ||
        response[1] != ARM_POSITION_COMMAND_CODE ||
        response[3] != ARM_X42_CHECK_BYTE) {
        *needs_wait = true;
        return true;
    }

    if (response[2] == ARM_COMMAND_STATUS_PARAM_ERROR ||
        response[2] == ARM_COMMAND_STATUS_FORMAT_ERROR) {
        APP_LOG_ERROR("%s startup position command failed, status=0x%02X",
                      name, response[2]);
        return false;
    }

    if (response[2] == ARM_COMMAND_STATUS_ACTION_DONE ||
        response[2] == ARM_COMMAND_STATUS_ALREADY_DONE) {
        APP_LOG_INFO("%s startup position done by command status 0x%02X",
                     name, response[2]);
        return true;
    }

    *needs_wait = true;
    return true;
}

static void print_runtime_joint_angles(const char *prefix,
                                       const ArmRuntime *runtime);
static void print_runtime_endpoint_position(const char *prefix,
                                            const ArmRuntime *runtime);

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
        APP_LOG_ERROR("startup stage target unreachable: x=%.2fmm, z=%.2fmm",
                      x, z);
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

    distance_a = startup_joint_solution_distance(theta2_a, theta3_a,
                                                 ref_theta2, ref_theta3);
    distance_b = startup_joint_solution_distance(theta2_b, theta3_b,
                                                 ref_theta2, ref_theta3);
    if (distance_a <= distance_b) {
        *target_theta2 = theta2_a;
        *target_theta3 = theta3_a;
    } else {
        *target_theta2 = theta2_b;
        *target_theta3 = theta3_b;
    }

    return true;
}

static bool move_arm_joints_to_target(ls2k0300_uart_t *motor_uart,
                                      ArmRuntime *runtime,
                                      const char *stage_name,
                                      double target_theta2,
                                      double target_theta3)
{
    bool wait_motor2 = false;
    bool wait_motor3 = false;
    double motor2_delta_degrees;
    double motor3_delta_degrees;

    if (motor_uart == NULL || runtime == NULL || stage_name == NULL) {
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

    APP_LOG_INFO("%s target: theta2=%.2fdeg, theta3=%.2fdeg, "
                 "motor_delta2=%.2fdeg, motor_delta3=%.2fdeg",
                 stage_name,
                 target_theta2 * 180.0 / M_PI,
                 target_theta3 * 180.0 / M_PI,
                 motor2_delta_degrees,
                 motor3_delta_degrees);

    if (!start_arm_relative_position_degrees(motor_uart,
                                             ARM_MOTOR_ADDR_2,
                                             "arm motor2",
                                             motor2_delta_degrees,
                                             &wait_motor2)) {
        return false;
    }
    usleep(1000);

    if (!start_arm_relative_position_degrees(motor_uart,
                                             ARM_MOTOR_ADDR_3,
                                             "arm motor3",
                                             motor3_delta_degrees,
                                             &wait_motor3)) {
        return false;
    }

    if (wait_motor2 &&
        !wait_arm_position_arrived(motor_uart, ARM_MOTOR_ADDR_2,
                                   "arm motor2")) {
        return false;
    }
    if (wait_motor3 &&
        !wait_arm_position_arrived(motor_uart, ARM_MOTOR_ADDR_3,
                                   "arm motor3")) {
        return false;
    }

    if (!read_runtime_joint_angles(motor_uart, runtime)) {
        APP_LOG_ERROR("%s final joint angle read failed", stage_name);
        return false;
    }

    print_runtime_joint_angles(stage_name, runtime);
    print_runtime_endpoint_position(stage_name, runtime);
    return true;
}

static bool move_arm_to_startup_pose(ls2k0300_uart_t *motor_uart,
                                     ArmRuntime *runtime)
{
    point3d_t current_position;
    point3d_t target_position;
    double theta2_error_deg;
    double theta3_error_deg;
    double stage_z_theta2;
    double stage_z_theta3;

    if (motor_uart == NULL || runtime == NULL ||
        !runtime->motor_zero_valid) {
        return false;
    }

    if (std::fabs(ARM_STARTUP_TARGET_THETA1_RAD -
                  ARM_INITIAL_THETA1_RAD) > 1e-6) {
        APP_LOG_WARN("startup theta1 target differs from calibrated theta1; "
                     "motor1 realtime angle is not read yet");
        return false;
    }

    if (!read_runtime_joint_angles(motor_uart, runtime)) {
        APP_LOG_ERROR("startup pose read current joint angles failed");
        return false;
    }

    APP_LOG_INFO("startup target theta: theta1=%.2fdeg, theta2=%.2fdeg, "
                 "theta3=%.2fdeg",
                 ARM_STARTUP_TARGET_THETA1_RAD * 180.0 / M_PI,
                 ARM_STARTUP_TARGET_THETA2_RAD * 180.0 / M_PI,
                 ARM_STARTUP_TARGET_THETA3_RAD * 180.0 / M_PI);

    if (!arm_forward_kinematics(runtime->theta1,
                                runtime->theta2,
                                runtime->theta3,
                                &current_position) ||
        !arm_forward_kinematics(ARM_STARTUP_TARGET_THETA1_RAD,
                                ARM_STARTUP_TARGET_THETA2_RAD,
                                ARM_STARTUP_TARGET_THETA3_RAD,
                                &target_position)) {
        return false;
    }

    APP_LOG_INFO("startup stage order: z first, then x/final");
    APP_LOG_INFO("startup stage z target: keep x=%.2fmm, z %.2fmm -> %.2fmm",
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

    if (!move_arm_joints_to_target(motor_uart,
                                   runtime,
                                   "startup stage z measured",
                                   stage_z_theta2,
                                   stage_z_theta3)) {
        return false;
    }

    APP_LOG_INFO("startup stage x target: x %.2fmm -> %.2fmm, z=%.2fmm",
                 (double)current_position.x,
                 (double)target_position.x,
                 (double)target_position.z);
    if (!move_arm_joints_to_target(motor_uart,
                                   runtime,
                                   "startup target measured",
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
        APP_LOG_WARN("startup pose error too large: theta2_err=%.2fdeg, "
                     "theta3_err=%.2fdeg",
                     theta2_error_deg,
                     theta3_error_deg);
        return false;
    }

    return true;
}

static bool move_arm_to_startup_height(ls2k0300_uart_t *motor_uart,
                                       ArmRuntime *runtime,
                                       const char *stage_name)
{
    point3d_t current_position;
    point3d_t target_position;
    double stage_theta2;
    double stage_theta3;

    if (motor_uart == NULL || runtime == NULL || stage_name == NULL ||
        !runtime->motor_zero_valid) {
        return false;
    }

    if (!read_runtime_joint_angles(motor_uart, runtime)) {
        APP_LOG_ERROR("%s read current joint angles failed", stage_name);
        return false;
    }

    if (!arm_forward_kinematics(runtime->theta1,
                                runtime->theta2,
                                runtime->theta3,
                                &current_position) ||
        !arm_forward_kinematics(ARM_STARTUP_TARGET_THETA1_RAD,
                                ARM_STARTUP_TARGET_THETA2_RAD,
                                ARM_STARTUP_TARGET_THETA3_RAD,
                                &target_position)) {
        return false;
    }

    APP_LOG_INFO("%s height target: keep x=%.2fmm, z %.2fmm -> %.2fmm",
                 stage_name,
                 (double)current_position.x,
                 (double)current_position.z,
                 (double)target_position.z);

    if (!solve_startup_xz_to_joint_angles(current_position.x,
                                          target_position.z,
                                          runtime->theta2,
                                          runtime->theta3,
                                          &stage_theta2,
                                          &stage_theta3)) {
        return false;
    }

    return move_arm_joints_to_target(motor_uart,
                                     runtime,
                                     stage_name,
                                     stage_theta2,
                                     stage_theta3);
}

static void print_runtime_joint_angles(const char *prefix,
                                       const ArmRuntime *runtime)
{
    APP_LOG_INFO("%s theta1=%.2fdeg, theta2=%.2fdeg, theta3=%.2fdeg",
                 prefix,
                 runtime->theta1 * 180.0 / M_PI,
                 runtime->theta2 * 180.0 / M_PI,
                 runtime->theta3 * 180.0 / M_PI);
}

static bool compute_runtime_endpoint_position(const ArmRuntime *runtime,
                                              point3d_t *position,
                                              double *radius_base)
{
    double x;
    double y;
    double z;

    if (runtime == NULL || position == NULL || radius_base == NULL) {
        return false;
    }

    if (!arm_forward_kinematics(runtime->theta1,
                                runtime->theta2,
                                runtime->theta3,
                                position)) {
        return false;
    }

    x = position->x;
    y = position->y;
    z = position->z;
    *radius_base = std::sqrt(x * x + y * y + z * z);

    return true;
}

static void print_runtime_endpoint_position(const char *prefix,
                                            const ArmRuntime *runtime)
{
    point3d_t position;
    double radius_base;

    if (!compute_runtime_endpoint_position(runtime, &position, &radius_base)) {
        return;
    }

    APP_LOG_INFO("%s endpoint(arm origin): "
                 "x=%.2fmm, y=%.2fmm, z=%.2fmm, r_base=%.2fmm",
                 prefix,
                 (double)position.x,
                 (double)position.y,
                 (double)position.z,
                 radius_base);
}

static void print_runtime_endpoint_speed(const char *prefix,
                                         const point3d_t *position,
                                         double radius_base,
                                         double target_vx,
                                         double target_vy,
                                         double target_vz,
                                         double limited_vx,
                                         double limited_vy,
                                         double limited_vz,
                                         unsigned int limit_flags)
{
    APP_LOG_INFO("%s endpoint(arm origin): "
                 "x=%.2fmm, y=%.2fmm, z=%.2fmm, r_base=%.2fmm, "
                 "v=(%.2f, %.2f, %.2f)->(%.2f, %.2f, %.2f), "
                 "limit[low_z_need_x=%u,z90_x=%u,z100_x=%u,max_x=%u,max_z=%u]",
                 prefix,
                 (double)position->x,
                 (double)position->y,
                 (double)position->z,
                 radius_base,
                 target_vx,
                 target_vy,
                 target_vz,
                 limited_vx,
                 limited_vy,
                 limited_vz,
                 (limit_flags & ARM_LIMIT_FLAG_LOW_Z_NEED_X) ? 1U : 0U,
                 (limit_flags & ARM_LIMIT_FLAG_Z90_MAX_X) ? 1U : 0U,
                 (limit_flags & ARM_LIMIT_FLAG_Z100_MAX_X) ? 1U : 0U,
                 (limit_flags & ARM_LIMIT_FLAG_MAX_X) ? 1U : 0U,
                 (limit_flags & ARM_LIMIT_FLAG_MAX_Z) ? 1U : 0U);
}

static bool is_low_z_band(double z)
{
    return z >= ARM_LIMIT_LOW_Z_MIN_MM && z <= ARM_LIMIT_LOW_Z_MAX_MM;
}

static unsigned int limit_endpoint_velocity_by_position(
    const point3d_t *position,
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
    x = position->x;
    z = position->z;

    if (x >= ARM_LIMIT_MAX_X_MM && *vx > 0.0) {
        *vx = 0.0;
        flags |= ARM_LIMIT_FLAG_MAX_X;
    }

    if (z >= ARM_LIMIT_MAX_Z_MM && *vz > 0.0) {
        *vz = 0.0;
        flags |= ARM_LIMIT_FLAG_MAX_Z;
    }

    if (is_low_z_band(z)) {
        if (x <= ARM_LIMIT_LOW_Z_MIN_X_MM && *vx < 0.0) {
            *vx = 0.0;
            flags |= ARM_LIMIT_FLAG_LOW_Z_NEED_X;
        }
        if (x < ARM_LIMIT_LOW_Z_MIN_X_MM && std::fabs(*vz) > 1e-9) {
            *vz = 0.0;
            flags |= ARM_LIMIT_FLAG_LOW_Z_NEED_X;
        }
    } else if (x < ARM_LIMIT_LOW_Z_MIN_X_MM) {
        if (z < ARM_LIMIT_LOW_Z_MIN_MM && *vz > 0.0) {
            *vz = 0.0;
            flags |= ARM_LIMIT_FLAG_LOW_Z_NEED_X;
        } else if (z > ARM_LIMIT_LOW_Z_MAX_MM && *vz < 0.0) {
            *vz = 0.0;
            flags |= ARM_LIMIT_FLAG_LOW_Z_NEED_X;
        }
    }

    if (z >= ARM_LIMIT_Z100_MM) {
        if (x >= ARM_LIMIT_Z100_MAX_X_MM && *vx > 0.0) {
            *vx = 0.0;
            flags |= ARM_LIMIT_FLAG_Z100_MAX_X;
        }
        if (x > ARM_LIMIT_Z100_MAX_X_MM && *vz > 0.0) {
            *vz = 0.0;
            flags |= ARM_LIMIT_FLAG_Z100_MAX_X;
        }
    } else if (z >= ARM_LIMIT_Z90_MM) {
        if (x >= ARM_LIMIT_Z90_MAX_X_MM && *vx > 0.0) {
            *vx = 0.0;
            flags |= ARM_LIMIT_FLAG_Z90_MAX_X;
        }
        if (x > ARM_LIMIT_Z90_MAX_X_MM && *vz > 0.0) {
            *vz = 0.0;
            flags |= ARM_LIMIT_FLAG_Z90_MAX_X;
        }
    }

    return flags;
}

static void apply_motor_limit(JointMotorSpeeds *speeds)
{
    float max_rpm = std::max(speeds->velocity_1.rpm_abs,
                             std::max(speeds->velocity_2.rpm_abs,
                                      speeds->velocity_3.rpm_abs));
    if (max_rpm <= ARM_MAX_MOTOR_RPM || max_rpm <= 0.0f) {
        return;
    }

    float scale = ARM_MAX_MOTOR_RPM / max_rpm;
    speeds->velocity_1.rpm *= scale;
    speeds->velocity_2.rpm *= scale;
    speeds->velocity_3.rpm *= scale;
    speeds->velocity_1.rpm_abs = fabsf(speeds->velocity_1.rpm);
    speeds->velocity_2.rpm_abs = fabsf(speeds->velocity_2.rpm);
    speeds->velocity_3.rpm_abs = fabsf(speeds->velocity_3.rpm);
}

static void send_arm_speeds(ls2k0300_uart_t *motor_uart,
                            const JointMotorSpeeds *speeds)
{
    ZDT_X42_V2_Velocity_Control(motor_uart, ARM_MOTOR_ADDR_1,
                                speeds->velocity_1.dir,
                                ARM_VELOCITY_RAMP_RPMPS,
                                speeds->velocity_1.rpm_abs, 0);
    usleep(1000);
    ZDT_X42_V2_Velocity_Control(motor_uart, ARM_MOTOR_ADDR_2,
                                speeds->velocity_2.dir,
                                ARM_VELOCITY_RAMP_RPMPS,
                                speeds->velocity_2.rpm_abs, 0);
    usleep(1000);
    ZDT_X42_V2_Velocity_Control(motor_uart, ARM_MOTOR_ADDR_3,
                                speeds->velocity_3.dir,
                                ARM_VELOCITY_RAMP_RPMPS,
                                speeds->velocity_3.rpm_abs, 0);
    usleep(1000);
}

static void send_arm_stop_locked(ls2k0300_uart_t *motor_uart,
                                 ArmRuntime *runtime)
{
    JointMotorSpeeds zero;
    zero_speeds(&zero);
    send_arm_speeds(motor_uart, &zero);
    runtime->last_speeds = zero;
}

static void set_runtime_endpoint_speed(ArmRuntime *runtime,
                                       double vx,
                                       double vy,
                                       double vz,
                                       bool verbose)
{
    bool changed = std::fabs(runtime->target_vx - vx) > 1e-6 ||
                   std::fabs(runtime->target_vy - vy) > 1e-6 ||
                   std::fabs(runtime->target_vz - vz) > 1e-6 ||
                   !runtime->target_velocity_valid;

    runtime->target_vx = vx;
    runtime->target_vy = vy;
    runtime->target_vz = vz;
    runtime->stale_stop_sent = false;
    runtime->target_velocity_valid = true;
    runtime->target_velocity_dirty = verbose || changed;
    if (changed) {
        runtime->last_correction_us = monotonic_time_us();
        runtime->last_position_print_ms = 0;
        runtime->joint_target_valid = false;
    }
}

static bool read_line_nonblocking(ls2k0300_uart_t *uart, char *line,
                                  size_t line_size)
{
    static size_t pos = 0;

    if (line == NULL || line_size == 0U) {
        return false;
    }

    while (running) {
        uint8_t ch;
        ssize_t n = ls2k0300_uart_read(uart, &ch, 1);
        if (n <= 0) {
            if (n < 0 && errno != EINTR && errno != EAGAIN &&
                errno != EWOULDBLOCK) {
                pos = 0;
            }
            return false;
        }

        if (ch == '\r') {
            continue;
        }

        if (ch == '\n') {
            line[pos] = '\0';
            bool has_line = pos > 0U;
            pos = 0;
            if (has_line) {
                return true;
            }
            continue;
        }

        if (pos < line_size - 1U) {
            line[pos++] = (char)ch;
        } else {
            pos = 0;
        }
    }

    return false;
}

static bool parse_velocity_line(const char *line, double *vx, double *vy,
                                double *vz)
{
    double x;
    double y;
    double z;

    if (sscanf(line, "X:%lf,Y:%lf,Z:%lf", &x, &y, &z) == 3) {
        *vx = x;
        *vy = y;
        *vz = z;
        return true;
    }

    if (sscanf(line, "X:%lf,Y:%lf", &x, &z) == 2) {
        *vx = x;
        *vy = 0.0;
        *vz = z;
        return true;
    }

    return false;
}

static void correct_arm_speed_locked(ls2k0300_uart_t *motor_uart,
                                     ArmRuntime *runtime)
{
    int64_t now_us = monotonic_time_us();

    if (!runtime->target_velocity_valid) {
        return;
    }

    if (!runtime->motor_zero_valid) {
        if (calibrate_runtime_motor_zero(motor_uart, runtime)) {
            print_runtime_joint_angles("initial measured", runtime);
        } else {
            JointMotorSpeeds zero;

            APP_LOG_ERROR("startup motor zero read failed, stop motors");
            zero_speeds(&zero);
            send_arm_speeds(motor_uart, &zero);
            runtime->last_speeds = zero;
            return;
        }
    }

    if (!read_runtime_joint_angles(motor_uart, runtime)) {
        JointMotorSpeeds zero;

        APP_LOG_ERROR("read arm joint angles failed, stop motors");
        zero_speeds(&zero);
        send_arm_speeds(motor_uart, &zero);
        runtime->last_speeds = zero;
        return;
    }

    point3d_t endpoint_position;
    double endpoint_radius_base = 0.0;
    bool endpoint_valid = compute_runtime_endpoint_position(
        runtime, &endpoint_position, &endpoint_radius_base);
    double limited_vx = runtime->target_vx;
    double limited_vy = runtime->target_vy;
    double limited_vz = runtime->target_vz;
    unsigned int limit_flags = 0U;

    if (endpoint_valid) {
        limit_flags = limit_endpoint_velocity_by_position(
            &endpoint_position,
            &limited_vx,
            &limited_vy,
            &limited_vz);
    }

    int64_t now_ms = now_us / 1000LL;
#if ARM_VERBOSE_MOTION_LOG
    if (runtime->target_velocity_dirty ||
        runtime->last_position_print_ms <= 0 ||
        now_ms - runtime->last_position_print_ms >=
            ARM_POSITION_PRINT_PERIOD_MS) {
        if (endpoint_valid) {
            print_runtime_endpoint_speed("main2 realtime",
                                         &endpoint_position,
                                         endpoint_radius_base,
                                         runtime->target_vx,
                                         runtime->target_vy,
                                         runtime->target_vz,
                                         limited_vx,
                                         limited_vy,
                                         limited_vz,
                                         limit_flags);
        }
        runtime->last_position_print_ms = now_ms;
    }
#else
    (void)endpoint_radius_base;
    (void)limit_flags;
    runtime->last_position_print_ms = now_ms;
#endif

    double dt = 0.0;
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
    compute_motor_speeds(limited_vx,
                         limited_vy,
                         limited_vz,
                         runtime->theta1,
                         runtime->theta2,
                         runtime->theta3,
                         ARM_GEAR_RATIO_1,
                         ARM_GEAR_RATIO_2,
                         ARM_GEAR_RATIO_3,
                         &target_speeds);

    if (!target_speeds.valid) {
        APP_LOG_ERROR("arm velocity solve failed, stop motors");
        JointMotorSpeeds zero;
        zero_speeds(&zero);
        send_arm_speeds(motor_uart, &zero);
        runtime->last_speeds = zero;
        return;
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
    arm_motor_speed_from_joint_radps(&speeds.velocity_2,
                                     target_dtheta2 - correction2,
                                     ARM_GEAR_RATIO_2,
                                     1U);
    arm_motor_speed_from_joint_radps(&speeds.velocity_3,
                                     target_dtheta3 - correction3,
                                     ARM_GEAR_RATIO_3,
                                     0U);
    apply_motor_limit(&speeds);

    send_arm_speeds(motor_uart, &speeds);
    runtime->last_speeds = speeds;

#if ARM_VERBOSE_MOTION_LOG
    if (runtime->target_velocity_dirty) {
        APP_LOG_INFO("main2 measured theta: "
                     "theta1=%.2fdeg, theta2=%.2fdeg, theta3=%.2fdeg",
                     runtime->theta1 * 180.0 / M_PI,
                     runtime->theta2 * 180.0 / M_PI,
                     runtime->theta3 * 180.0 / M_PI);
        APP_LOG_INFO("main2 motor speed: "
                     "M1(dir=%u,rpm=%.2f,signed=%.2f), "
                     "M2(dir=%u,rpm=%.2f,signed=%.2f), "
                     "M3(dir=%u,rpm=%.2f,signed=%.2f)",
                     (unsigned int)speeds.velocity_1.dir,
                     speeds.velocity_1.rpm_abs,
                     speeds.velocity_1.rpm,
                     (unsigned int)speeds.velocity_2.dir,
                     speeds.velocity_2.rpm_abs,
                     speeds.velocity_2.rpm,
                     (unsigned int)speeds.velocity_3.dir,
                     speeds.velocity_3.rpm_abs,
                     speeds.velocity_3.rpm);
        APP_LOG_INFO("main2 joint target: theta2=%.2fdeg, theta3=%.2fdeg, "
                     "ahead_error2=%.2fdeg, ahead_error3=%.2fdeg, dt=%.3fs",
                     runtime->target_theta2 * 180.0 / M_PI,
                     runtime->target_theta3 * 180.0 / M_PI,
                     error2 * 180.0 / M_PI,
                     error3 * 180.0 / M_PI,
                     dt);
        runtime->target_velocity_dirty = false;
    }
#else
    runtime->target_velocity_dirty = false;
#endif
}

static void run_endpoint_speed_once(ls2k0300_uart_t *motor_uart,
                                    ArmRuntime *runtime,
                                    double vx,
                                    double vy,
                                    double vz,
                                    bool verbose)
{
    pthread_mutex_lock(&control_mutex);
    set_runtime_endpoint_speed(runtime, vx, vy, vz, verbose);
    correct_arm_speed_locked(motor_uart, runtime);
    pthread_mutex_unlock(&control_mutex);
}

static void stop_arm_motion(ls2k0300_uart_t *motor_uart, ArmRuntime *runtime)
{
    pthread_mutex_lock(&control_mutex);
    send_arm_stop_locked(motor_uart, runtime);
    runtime->target_velocity_valid = false;
    runtime->target_velocity_dirty = false;
    runtime->joint_target_valid = false;
    runtime->vision_velocity_valid = false;
    pthread_mutex_unlock(&control_mutex);
}

static void update_patrol_direction(const ArmRuntime *runtime,
                                    double *patrol_dir)
{
    point3d_t position;
    double radius_base;

    if (patrol_dir == NULL ||
        !compute_runtime_endpoint_position(runtime, &position, &radius_base)) {
        return;
    }

    (void)radius_base;
    if (is_low_z_band(position.z) &&
        position.x <= ARM_LIMIT_LOW_Z_MIN_X_MM + ARM_PATROL_X_TOLERANCE_MM) {
        *patrol_dir = 1.0;
        return;
    }

    if (position.x >= ARM_PATROL_MAX_X_MM - ARM_PATROL_X_TOLERANCE_MM) {
        *patrol_dir = -1.0;
    } else if (position.x <= ARM_PATROL_MIN_X_MM + ARM_PATROL_X_TOLERANCE_MM) {
        *patrol_dir = 1.0;
    }
}

static bool limit_inject_test_patrol_limit_reached(const ArmRuntime *runtime,
                                                   double patrol_dir)
{
    point3d_t position;
    double radius_base;
    double vx;
    double vy = 0.0;
    double vz = 0.0;
    unsigned int limit_flags;

    if (runtime == NULL ||
        !compute_runtime_endpoint_position(runtime, &position, &radius_base)) {
        return false;
    }

    (void)radius_base;
    vx = patrol_dir * ARM_PATROL_X_SPEED_MMPS;
    limit_flags = limit_endpoint_velocity_by_position(&position, &vx, &vy, &vz);
    if (limit_flags != 0U) {
        return true;
    }

    if (patrol_dir > 0.0 &&
        position.x >= ARM_PATROL_MAX_X_MM - ARM_PATROL_X_TOLERANCE_MM) {
        return true;
    }

    if (patrol_dir < 0.0 &&
        position.x <= ARM_PATROL_MIN_X_MM + ARM_PATROL_X_TOLERANCE_MM) {
        return true;
    }

    return false;
}

static bool target_lost_requested_locked(ArmRuntime *runtime)
{
    return runtime->target_lost_requested;
}

static void clear_injection_requests_locked(ArmRuntime *runtime)
{
    runtime->inject_start_requested = false;
    runtime->target_lost_requested = false;
}

static bool recover_after_injection_abort(All_control *inject_control,
                                          ls2k0300_uart_t *vision_uart,
                                          ls2k0300_uart_t *motor_uart,
                                          ArmRuntime *runtime,
                                          bool reset_base)
{
    bool height_ready = false;

    stop_arm_motion(motor_uart, runtime);
    inject_control->Motor_stop();
    injection_running = 1;
    inject_control->Inject_stop(INJECT_NEEDLE_DIRECTION);
    (void)inject_control->Inject_home_to_zero();

    if (reset_base) {
        (void)home_arm_motor_nearest(motor_uart, ARM_MOTOR_ADDR_1);
    }

    pthread_mutex_lock(&control_mutex);
    clear_injection_requests_locked(runtime);
    runtime->target_velocity_valid = false;
    runtime->target_velocity_dirty = false;
    runtime->joint_target_valid = false;
    runtime->vision_velocity_valid = false;
    runtime->last_rx_ms = 0;
    pthread_mutex_unlock(&control_mutex);

    if (running && !shutdown_requested) {
        height_ready = move_arm_to_startup_height(motor_uart,
                                                  runtime,
                                                  "recovery startup height");
        if (!height_ready) {
            APP_LOG_WARN("recovery startup height failed");
        }
    }

    stop_arm_motion(motor_uart, runtime);
    injection_running = 1;

    pthread_mutex_lock(&control_mutex);
    clear_injection_requests_locked(runtime);
    runtime->target_velocity_valid = false;
    runtime->target_velocity_dirty = false;
    runtime->joint_target_valid = false;
    runtime->vision_velocity_valid = false;
    runtime->last_rx_ms = 0;
    pthread_mutex_unlock(&control_mutex);

    if (!running || shutdown_requested) {
        return height_ready;
    }

    send_main1_command_repeat(vision_uart, cmd_detect_5x5, 3U, 20000);
    send_main1_command_repeat(vision_uart, cmd_detect_resume, 3U, 20000);

    return height_ready;
}

static bool wait_push_motor_arrived(All_control *inject_control,
                                    ArmRuntime *runtime)
{
    const timespec push_period = {0, ARM_CORRECTION_PERIOD_MS * 1000000L};
    int64_t start_ms = monotonic_time_ms();

    while (running && !shutdown_requested && injection_running) {
        int64_t now_ms = monotonic_time_ms();

        if (now_ms - start_ms > INJECT_PUSH_WAIT_TIMEOUT_MS) {
            inject_control->Motor_stop();
            return false;
        }

        pthread_mutex_lock(&control_mutex);
        bool target_lost = target_lost_requested_locked(runtime);
        pthread_mutex_unlock(&control_mutex);
        if (target_lost) {
            return false;
        }

        inject_control->Motor_control();
        if (inject_control->Is_arrived()) {
            inject_control->Motor_stop();
            return true;
        }

        nanosleep(&push_period, NULL);
    }

    inject_control->Motor_stop();
    return false;
}

static InjectSequenceResult run_injection_sequence(All_control *inject_control,
                                                   ls2k0300_uart_t *vision_uart,
                                                   ls2k0300_uart_t *motor_uart,
                                                   ArmRuntime *runtime)
{
    injection_running = 1;
    stop_arm_motion(motor_uart, runtime);

    pthread_mutex_lock(&control_mutex);
    runtime->inject_start_requested = false;
    pthread_mutex_unlock(&control_mutex);

    if (!inject_control->Inject_run_until_contact(INJECT_NEEDLE_DIRECTION,
                                                  INJECT_DEFAULT_SPEED_RPM)) {
        pthread_mutex_lock(&control_mutex);
        bool lost = target_lost_requested_locked(runtime);
        pthread_mutex_unlock(&control_mutex);
        return lost ? INJECT_SEQUENCE_TARGET_LOST :
               shutdown_requested ? INJECT_SEQUENCE_INTERRUPTED :
               INJECT_SEQUENCE_FAIL;
    }

    send_main1_command(vision_uart, cmd_detect_150x150);
    inject_control->Set_target_delta(INJECT_PUSH_VOLUME_ML);
    if (!wait_push_motor_arrived(inject_control, runtime)) {
        pthread_mutex_lock(&control_mutex);
        bool lost = target_lost_requested_locked(runtime);
        pthread_mutex_unlock(&control_mutex);
        return lost ? INJECT_SEQUENCE_TARGET_LOST :
               shutdown_requested ? INJECT_SEQUENCE_INTERRUPTED :
               INJECT_SEQUENCE_FAIL;
    }

    if (!inject_control->Inject_home_to_zero()) {
        pthread_mutex_lock(&control_mutex);
        bool lost = target_lost_requested_locked(runtime);
        pthread_mutex_unlock(&control_mutex);
        return lost ? INJECT_SEQUENCE_TARGET_LOST :
               shutdown_requested ? INJECT_SEQUENCE_INTERRUPTED :
               INJECT_SEQUENCE_FAIL;
    }

    return INJECT_SEQUENCE_SUCCESS;
}

static bool wait_shutdown_stage_x_reached(ls2k0300_uart_t *motor_uart,
                                          ArmRuntime *runtime,
                                          double target_x,
                                          int64_t shutdown_start_ms)
{
    point3d_t position;
    double radius_base;
    int64_t last_print_ms = 0;

    while (running) {
        int64_t now_ms = monotonic_time_ms();

        if (now_ms - shutdown_start_ms > ARM_SHUTDOWN_TIMEOUT_MS) {
            APP_LOG_WARN("shutdown stage x wait timeout");
            return false;
        }

        if (!read_runtime_joint_angles(motor_uart, runtime) ||
            !compute_runtime_endpoint_position(runtime,
                                               &position,
                                               &radius_base)) {
            usleep(ARM_CORRECTION_PERIOD_MS * 1000);
            continue;
        }
        (void)radius_base;

        double x_error = target_x - position.x;
        if (std::fabs(x_error) <= ARM_SHUTDOWN_STAGE_X_EPS_MM) {
            APP_LOG_INFO("shutdown stage x confirmed: x=%.2fmm, target=%.2fmm",
                         (double)position.x,
                         target_x);
            return true;
        }

        if (last_print_ms <= 0 ||
            now_ms - last_print_ms >= ARM_POSITION_PRINT_PERIOD_MS) {
            APP_LOG_INFO("shutdown stage x waiting: x=%.2fmm, target=%.2fmm, "
                         "err=%.2fmm",
                         (double)position.x,
                         target_x,
                         x_error);
            last_print_ms = now_ms;
        }

        usleep(ARM_CORRECTION_PERIOD_MS * 1000);
    }

    return false;
}

static bool run_shutdown_return_zero_by_angle(ls2k0300_uart_t *motor_uart,
                                              ArmRuntime *runtime,
                                              int64_t shutdown_start_ms)
{
    point3d_t current_position;
    point3d_t final_position;
    double stage_x_target;
    double stage_x_theta2;
    double stage_x_theta3;
    double theta2_error_deg;
    double theta3_error_deg;
    bool ok = false;

    if (motor_uart == NULL || runtime == NULL) {
        return false;
    }

    pthread_mutex_lock(&control_mutex);

    send_arm_stop_locked(motor_uart, runtime);
    runtime->target_velocity_valid = false;
    runtime->target_velocity_dirty = false;
    runtime->joint_target_valid = false;
    runtime->vision_velocity_valid = false;

    APP_LOG_INFO("shutdown return target by angle: x first, then z/final");

    if (monotonic_time_ms() - shutdown_start_ms > ARM_SHUTDOWN_TIMEOUT_MS) {
        APP_LOG_WARN("shutdown return target timeout before move");
        goto out;
    }

    if (!runtime->motor_zero_valid) {
        APP_LOG_ERROR("shutdown return target failed: motor zero invalid");
        goto out;
    }

    if (!read_runtime_joint_angles(motor_uart, runtime)) {
        APP_LOG_ERROR("shutdown return target read current joint angles failed");
        goto out;
    }

    if (!arm_forward_kinematics(runtime->theta1,
                                runtime->theta2,
                                runtime->theta3,
                                &current_position) ||
        !arm_forward_kinematics(ARM_SHUTDOWN_TARGET_THETA1_RAD,
                                ARM_SHUTDOWN_TARGET_THETA2_RAD,
                                ARM_SHUTDOWN_TARGET_THETA3_RAD,
                                &final_position)) {
        APP_LOG_ERROR("shutdown return target forward kinematics failed");
        goto out;
    }

    stage_x_target = final_position.x;

    APP_LOG_INFO("shutdown stage x target: x %.2fmm -> %.2fmm, "
                 "keep z=%.2fmm, final zero z=%.2fmm",
                 (double)current_position.x,
                 stage_x_target,
                 (double)current_position.z,
                 (double)final_position.z);

    if (!solve_startup_xz_to_joint_angles(stage_x_target,
                                          current_position.z,
                                          runtime->theta2,
                                          runtime->theta3,
                                          &stage_x_theta2,
                                          &stage_x_theta3)) {
        goto out;
    }

    if (!move_arm_joints_to_target(motor_uart,
                                   runtime,
                                   "shutdown stage x measured",
                                   stage_x_theta2,
                                   stage_x_theta3)) {
        goto out;
    }

    if (!wait_shutdown_stage_x_reached(motor_uart,
                                       runtime,
                                       stage_x_target,
                                       shutdown_start_ms)) {
        goto out;
    }

    if (monotonic_time_ms() - shutdown_start_ms > ARM_SHUTDOWN_TIMEOUT_MS) {
        APP_LOG_WARN("shutdown return target timeout after x stage");
        goto out;
    }

    APP_LOG_INFO("shutdown stage z/final target: theta2=%.2fdeg, "
                 "theta3=%.2fdeg",
                 ARM_SHUTDOWN_TARGET_THETA2_RAD * 180.0 / M_PI,
                 ARM_SHUTDOWN_TARGET_THETA3_RAD * 180.0 / M_PI);

    if (!move_arm_joints_to_target(motor_uart,
                                   runtime,
                                   "shutdown zero measured",
                                   ARM_SHUTDOWN_TARGET_THETA2_RAD,
                                   ARM_SHUTDOWN_TARGET_THETA3_RAD)) {
        goto out;
    }

    theta2_error_deg =
        (runtime->theta2 - ARM_SHUTDOWN_TARGET_THETA2_RAD) * 180.0 / M_PI;
    theta3_error_deg =
        (runtime->theta3 - ARM_SHUTDOWN_TARGET_THETA3_RAD) * 180.0 / M_PI;
    if (std::fabs(theta2_error_deg) > ARM_SHUTDOWN_JOINT_ARRIVE_EPS_DEG ||
        std::fabs(theta3_error_deg) > ARM_SHUTDOWN_JOINT_ARRIVE_EPS_DEG) {
        APP_LOG_WARN("shutdown zero error too large: theta2_err=%.2fdeg, "
                     "theta3_err=%.2fdeg",
                     theta2_error_deg,
                     theta3_error_deg);
        goto out;
    }

    ok = true;

out:
    send_arm_stop_locked(motor_uart, runtime);
    runtime->target_velocity_valid = false;
    runtime->target_velocity_dirty = false;
    runtime->joint_target_valid = false;
    runtime->vision_velocity_valid = false;
    pthread_mutex_unlock(&control_mutex);
    return ok;
}

static void *arm_control_thread(void *arg)
{
    ArmThreadContext *ctx = (ArmThreadContext *)arg;
    char line[128];

    while (running) {
        double vx;
        double vy;
        double vz;

        if (!read_line_nonblocking(ctx->vision_uart, line, sizeof(line))) {
            usleep(1000);
            continue;
        }

        if (strcmp(line, main1_handshake_cmd) == 0) {
            pthread_mutex_lock(&control_mutex);
            bool was_done = ctx->runtime->handshake_done;
            ctx->runtime->handshake_done = true;
            pthread_mutex_unlock(&control_mutex);
            if (!was_done) {
                APP_LOG_INFO("main2 handshake received");
            }
            continue;
        }

        if (strcmp(line, main1_exit_cmd) == 0) {
            APP_LOG_INFO("main2 received main1 exit request");
            pthread_mutex_lock(&control_mutex);
            ctx->runtime->target_velocity_valid = false;
            ctx->runtime->target_velocity_dirty = false;
            ctx->runtime->joint_target_valid = false;
            ctx->runtime->vision_velocity_valid = false;
            pthread_mutex_unlock(&control_mutex);
            shutdown_requested = 1;
            injection_running = 0;
            continue;
        }

        if (limit_inject_test_ignore_main1) {
            continue;
        }

        if (strcmp(line, cmd_inject_start) == 0) {
            pthread_mutex_lock(&control_mutex);
            ctx->runtime->inject_start_requested = true;
            ctx->runtime->target_velocity_valid = false;
            ctx->runtime->target_velocity_dirty = false;
            ctx->runtime->joint_target_valid = false;
            ctx->runtime->vision_velocity_valid = false;
            pthread_mutex_unlock(&control_mutex);
            APP_LOG_INFO("main2 received inject start");
            continue;
        }

        if (strcmp(line, cmd_target_lost) == 0) {
            injection_running = 0;
            pthread_mutex_lock(&control_mutex);
            ctx->runtime->target_lost_requested = true;
            ctx->runtime->target_velocity_valid = false;
            ctx->runtime->target_velocity_dirty = false;
            ctx->runtime->joint_target_valid = false;
            ctx->runtime->vision_velocity_valid = false;
            pthread_mutex_unlock(&control_mutex);
            APP_LOG_WARN("main2 received target lost");
            continue;
        }

        if (!parse_velocity_line(line, &vx, &vy, &vz)) {
            APP_LOG_WARN("invalid velocity line: %s", line);
            continue;
        }

        vx = clamp_double(vx * ARM_ENDPOINT_SPEED_SCALE,
                          -ARM_MAX_ENDPOINT_SPEED_MMPS,
                          ARM_MAX_ENDPOINT_SPEED_MMPS);
        vy = clamp_double(vy * ARM_ENDPOINT_SPEED_SCALE,
                          -ARM_MAX_ENDPOINT_SPEED_MMPS,
                          ARM_MAX_ENDPOINT_SPEED_MMPS);
        vz = clamp_double(vz * ARM_ENDPOINT_SPEED_SCALE,
                          -ARM_MAX_ENDPOINT_SPEED_MMPS,
                          ARM_MAX_ENDPOINT_SPEED_MMPS);
        vx *= ARM_ENDPOINT_X_DIRECTION;

        int64_t now_ms = monotonic_time_ms();
        pthread_mutex_lock(&control_mutex);
        ctx->runtime->handshake_done = true;
        ctx->runtime->target_vx = vx;
        ctx->runtime->target_vy = vy;
        ctx->runtime->target_vz = vz;
        ctx->runtime->last_rx_ms = now_ms;
        ctx->runtime->stale_stop_sent = false;
        ctx->runtime->target_velocity_valid = true;
        ctx->runtime->target_velocity_dirty = true;
        ctx->runtime->vision_velocity_valid = true;
        ctx->runtime->target_lost_requested = false;
        ctx->runtime->last_correction_us = monotonic_time_us();
        ctx->runtime->last_position_print_ms = 0;
        ctx->runtime->joint_target_valid = false;

#if ARM_VERBOSE_MOTION_LOG
        APP_LOG_INFO("main2 rx endpoint speed: vx=%.2f, vy=%.2f, vz=%.2f",
                     vx, vy, vz);
#endif
        pthread_mutex_unlock(&control_mutex);
    }

    return NULL;
}

int main(void)
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    ls2k0300_uart_t vision_uart;
    ls2k0300_uart_t motor_uart;
    memset(&vision_uart, 0, sizeof(vision_uart));
    memset(&motor_uart, 0, sizeof(motor_uart));

    if (ls2k0300_uart_init(&vision_uart, ARM_VISION_UART, B115200,
                           LS_UART_STOP1, LS_UART_DATA8,
                           LS_UART_PARITY_NONE,
                           LS_UART_MODE_NON_BLOCKING) != 0) {
        APP_LOG_ERROR("failed to init vision uart: %s", ARM_VISION_UART);
        return 1;
    }

    if (ls2k0300_uart_init(&motor_uart, ARM_MOTOR_UART, B921600,
                           LS_UART_STOP1, LS_UART_DATA8,
                           LS_UART_PARITY_NONE,
                           LS_UART_MODE_NON_BLOCKING) != 0) {
        APP_LOG_ERROR("failed to init motor uart: %s", ARM_MOTOR_UART);
        notify_main1_exit(&vision_uart);
        ls2k0300_uart_deinit(&vision_uart);
        return 1;
    }

    if (!home_arm_motors_nearest(&motor_uart)) {
        APP_LOG_ERROR("main2 startup nearest home failed, exit before control");
        JointMotorSpeeds zero;
        zero_speeds(&zero);
        send_arm_speeds(&motor_uart, &zero);
        notify_main1_exit(&vision_uart);
        ls2k0300_uart_deinit(&motor_uart);
        ls2k0300_uart_deinit(&vision_uart);
        return 1;
    }

    All_control inject_control;
    inject_control.Inject_attach_uart(&motor_uart);
    inject_control.Inject_set_running_flag(&injection_running);

    ArmRuntime runtime;
    memset(&runtime, 0, sizeof(runtime));
    runtime.theta1 = ARM_INITIAL_THETA1_RAD;
    runtime.theta2 = ARM_INITIAL_THETA2_RAD;
    runtime.theta3 = ARM_INITIAL_THETA3_RAD;
    runtime.stale_stop_sent = true;
    runtime.motor_zero_valid = false;
    runtime.target_velocity_valid = false;
    runtime.target_velocity_dirty = false;
    runtime.joint_target_valid = false;
    runtime.vision_velocity_valid = false;
    runtime.handshake_done = false;
    runtime.inject_start_requested = false;
    runtime.target_lost_requested = false;
    runtime.last_correction_us = 0;
    runtime.last_position_print_ms = 0;
    zero_speeds(&runtime.last_speeds);

    pthread_mutex_lock(&control_mutex);
    send_arm_stop_locked(&motor_uart, &runtime);
    bool startup_pose_ready = false;
    if (calibrate_runtime_motor_zero(&motor_uart, &runtime)) {
        print_runtime_joint_angles("initial measured", &runtime);
        print_runtime_endpoint_position("initial measured", &runtime);
        startup_pose_ready = move_arm_to_startup_pose(&motor_uart,
                                                      &runtime);
    } else {
        APP_LOG_ERROR("startup motor zero read failed");
    }
    pthread_mutex_unlock(&control_mutex);

    if (!startup_pose_ready) {
        APP_LOG_ERROR("startup pose move failed, exit before control");
        pthread_mutex_lock(&control_mutex);
        send_arm_stop_locked(&motor_uart, &runtime);
        pthread_mutex_unlock(&control_mutex);
        notify_main1_exit(&vision_uart);
        ls2k0300_uart_deinit(&motor_uart);
        ls2k0300_uart_deinit(&vision_uart);
        return 1;
    }

    APP_LOG_INFO("startup arm pose ready, init rotate motor4: home then 90deg");
    if (!inject_control.Rotate_control()) {
        APP_LOG_ERROR("rotate motor4 init failed, exit before control");
        pthread_mutex_lock(&control_mutex);
        send_arm_stop_locked(&motor_uart, &runtime);
        pthread_mutex_unlock(&control_mutex);
        inject_control.Rotate_stop(ROTATE_DEFAULT_DIRECTION);
        notify_main1_exit(&vision_uart);
        ls2k0300_uart_deinit(&motor_uart);
        ls2k0300_uart_deinit(&vision_uart);
        return 1;
    }
    APP_LOG_INFO("rotate motor4 init done, keep 90deg position");

    ArmThreadContext ctx;
    ctx.vision_uart = &vision_uart;
    ctx.motor_uart = &motor_uart;
    ctx.runtime = &runtime;

    pthread_t control_thread;
    if (pthread_create(&control_thread, NULL, arm_control_thread, &ctx) != 0) {
        APP_LOG_ERROR("failed to create arm control thread");
        notify_main1_exit(&vision_uart);
        ls2k0300_uart_deinit(&motor_uart);
        ls2k0300_uart_deinit(&vision_uart);
        return 1;
    }

    APP_LOG_INFO("main2 running: rx=%s, motor=%s, arm addr=%u/%u/%u",
                 ARM_VISION_UART, ARM_MOTOR_UART,
                 ARM_MOTOR_ADDR_1, ARM_MOTOR_ADDR_2, ARM_MOTOR_ADDR_3);
    APP_LOG_INFO("main2 control period: rx speed about 300ms, correction every %lldms",
                 ARM_CORRECTION_PERIOD_MS);

    ArmControlState state = ARM_STATE_WAIT_HANDSHAKE;
    ArmControlState last_logged_state = ARM_STATE_WAIT_HANDSHAKE;
    double patrol_dir = 1.0;
    int64_t post_patrol_start_ms = 0;
    int64_t shutdown_start_ms = 0;
    int64_t last_state_print_ms = 0;
    bool limit_inject_test_done = false;
    bool limit_inject_test_active = false;

    graceful_shutdown_ready = 1;

    while (running) {
        usleep(ARM_CORRECTION_PERIOD_MS * 1000);

        int64_t now_ms = monotonic_time_ms();

        if (shutdown_requested && state != ARM_STATE_SHUTDOWN_RETURN_SAFE) {
            injection_running = 0;
            stop_arm_motion(&motor_uart, &runtime);
            inject_control.Motor_stop();
            inject_control.Inject_stop(INJECT_NEEDLE_DIRECTION);
            send_main1_command(&vision_uart, cmd_detect_pause);
            APP_LOG_INFO("main2 state change: %s -> %s, shutdown requested",
                         arm_state_name(state),
                         arm_state_name(ARM_STATE_SHUTDOWN_RETURN_SAFE));
            state = ARM_STATE_SHUTDOWN_RETURN_SAFE;
            shutdown_start_ms = now_ms;
            APP_LOG_INFO("main2 shutdown requested, return zero by angle before exit");
        }

        bool handshake_done;
        bool inject_start_requested;
        bool vision_velocity_valid;
        int64_t last_rx_ms;

        pthread_mutex_lock(&control_mutex);
        handshake_done = runtime.handshake_done;
        inject_start_requested = runtime.inject_start_requested;
        vision_velocity_valid = runtime.vision_velocity_valid;
        last_rx_ms = runtime.last_rx_ms;
        pthread_mutex_unlock(&control_mutex);

        if (state != last_logged_state ||
            last_state_print_ms <= 0 ||
            now_ms - last_state_print_ms >= ARM_STATE_PRINT_PERIOD_MS) {
            APP_LOG_INFO("main2 state heartbeat: state=%s, handshake=%u, "
                         "vision_valid=%u, target_valid=%u, inject_req=%u, "
                         "target_lost=%u, last_rx_age=%lldms, patrol_dir=%.1f",
                         arm_state_name(state),
                         handshake_done ? 1U : 0U,
                         vision_velocity_valid ? 1U : 0U,
                         runtime.target_velocity_valid ? 1U : 0U,
                         inject_start_requested ? 1U : 0U,
                         runtime.target_lost_requested ? 1U : 0U,
                         last_rx_ms > 0 ? (long long)(now_ms - last_rx_ms) : -1LL,
                         patrol_dir);
            last_logged_state = state;
            last_state_print_ms = now_ms;
        }

        switch (state) {
        case ARM_STATE_WAIT_HANDSHAKE:
            if (handshake_done) {
                send_main1_command_repeat(&vision_uart,
                                          cmd_detect_resume,
                                          3U,
                                          20000);
                APP_LOG_INFO("main2 state change: %s -> %s, handshake done",
                             arm_state_name(state),
                             arm_state_name(ARM_STATE_PATROL_X));
                state = ARM_STATE_PATROL_X;
                APP_LOG_INFO("main2 state: patrol x");
            }
            break;

        case ARM_STATE_PATROL_X:
            if (!limit_inject_test_done) {
                if (limit_inject_test_patrol_limit_reached(&runtime,
                                                           patrol_dir)) {
                    limit_inject_test_done = true;
                    limit_inject_test_active = true;
                    stop_arm_motion(&motor_uart, &runtime);
                    pthread_mutex_lock(&control_mutex);
                    clear_injection_requests_locked(&runtime);
                    runtime.stale_stop_sent = true;
                    runtime.target_velocity_valid = false;
                    runtime.target_velocity_dirty = false;
                    runtime.joint_target_valid = false;
                    runtime.vision_velocity_valid = false;
                    pthread_mutex_unlock(&control_mutex);
                    APP_LOG_INFO("main2 limit inject test: patrol limit reached, force inject once");
                    APP_LOG_INFO("main2 state change: %s -> %s, limit inject test",
                                 arm_state_name(state),
                                 arm_state_name(ARM_STATE_INJECTION));
                    state = ARM_STATE_INJECTION;
                    break;
                }

                pthread_mutex_lock(&control_mutex);
                clear_injection_requests_locked(&runtime);
                runtime.stale_stop_sent = true;
                runtime.target_velocity_valid = false;
                runtime.target_velocity_dirty = false;
                runtime.joint_target_valid = false;
                runtime.vision_velocity_valid = false;
                pthread_mutex_unlock(&control_mutex);

                update_patrol_direction(&runtime, &patrol_dir);
                run_endpoint_speed_once(&motor_uart, &runtime,
                                        patrol_dir * ARM_PATROL_X_SPEED_MMPS,
                                        0.0, 0.0, false);
                break;
            }

            if (inject_start_requested) {
                APP_LOG_INFO("main2 state change: %s -> %s, inject request",
                             arm_state_name(state),
                             arm_state_name(ARM_STATE_INJECTION));
                state = ARM_STATE_INJECTION;
                break;
            }
            if (vision_velocity_valid) {
                APP_LOG_INFO("main2 state change: %s -> %s, vision velocity valid",
                             arm_state_name(state),
                             arm_state_name(ARM_STATE_TRACK_TARGET));
                state = ARM_STATE_TRACK_TARGET;
                APP_LOG_INFO("main2 state: track target");
                pthread_mutex_lock(&control_mutex);
                correct_arm_speed_locked(&motor_uart, &runtime);
                pthread_mutex_unlock(&control_mutex);
                break;
            }
            update_patrol_direction(&runtime, &patrol_dir);
            run_endpoint_speed_once(&motor_uart, &runtime,
                                    patrol_dir * ARM_PATROL_X_SPEED_MMPS,
                                    0.0, 0.0, false);
            update_patrol_direction(&runtime, &patrol_dir);
            break;

        case ARM_STATE_TRACK_TARGET:
            if (inject_start_requested) {
                APP_LOG_INFO("main2 state change: %s -> %s, inject request",
                             arm_state_name(state),
                             arm_state_name(ARM_STATE_INJECTION));
                state = ARM_STATE_INJECTION;
                break;
            }
            if (last_rx_ms > 0 &&
                now_ms - last_rx_ms > ARM_STALE_STOP_TIMEOUT_MS) {
                stop_arm_motion(&motor_uart, &runtime);
                pthread_mutex_lock(&control_mutex);
                runtime.stale_stop_sent = true;
                runtime.target_velocity_valid = false;
                runtime.target_velocity_dirty = false;
                runtime.joint_target_valid = false;
                runtime.vision_velocity_valid = false;
                pthread_mutex_unlock(&control_mutex);
                APP_LOG_INFO("main2 state change: %s -> %s, vision timeout",
                             arm_state_name(state),
                             arm_state_name(ARM_STATE_PATROL_X));
                state = ARM_STATE_PATROL_X;
                APP_LOG_WARN("vision velocity timeout, back to patrol");
                break;
            }
            pthread_mutex_lock(&control_mutex);
            correct_arm_speed_locked(&motor_uart, &runtime);
            pthread_mutex_unlock(&control_mutex);
            break;

        case ARM_STATE_INJECTION: {
            send_main1_command(&vision_uart, cmd_detect_150x150);
            InjectSequenceResult result =
                run_injection_sequence(&inject_control,
                                       &vision_uart,
                                       &motor_uart,
                                       &runtime);
            if (limit_inject_test_active) {
                limit_inject_test_active = false;
                limit_inject_test_ignore_main1 = 0;
                APP_LOG_INFO("main2 limit inject test done, resume main1 control");
            }
            if (result == INJECT_SEQUENCE_SUCCESS) {
                pthread_mutex_lock(&control_mutex);
                clear_injection_requests_locked(&runtime);
                pthread_mutex_unlock(&control_mutex);
                send_main1_command(&vision_uart, cmd_inject_done);
                send_main1_command(&vision_uart, cmd_detect_pause);
                post_patrol_start_ms = monotonic_time_ms();
                APP_LOG_INFO("main2 state change: %s -> %s, inject success",
                             arm_state_name(state),
                             arm_state_name(ARM_STATE_POST_INJECT_PATROL));
                state = ARM_STATE_POST_INJECT_PATROL;
                APP_LOG_INFO("inject sequence done, post patrol 2s");
            } else if (result == INJECT_SEQUENCE_TARGET_LOST) {
                bool recovered =
                    recover_after_injection_abort(&inject_control,
                                                 &vision_uart,
                                                 &motor_uart,
                                                 &runtime,
                                                 true);
                APP_LOG_INFO("main2 state change: %s -> %s, target lost recovered=%u",
                             arm_state_name(state),
                             arm_state_name(ARM_STATE_PATROL_X),
                             recovered ? 1U : 0U);
                state = ARM_STATE_PATROL_X;
                APP_LOG_WARN("inject aborted by target lost%s",
                             recovered ? "" : ", recovery height not ready");
            } else if (result == INJECT_SEQUENCE_INTERRUPTED ||
                       shutdown_requested) {
                APP_LOG_INFO("main2 state change: %s -> %s, inject interrupted",
                             arm_state_name(state),
                             arm_state_name(ARM_STATE_SHUTDOWN_RETURN_SAFE));
                state = ARM_STATE_SHUTDOWN_RETURN_SAFE;
                shutdown_start_ms = monotonic_time_ms();
            } else {
                bool recovered =
                    recover_after_injection_abort(&inject_control,
                                                 &vision_uart,
                                                 &motor_uart,
                                                 &runtime,
                                                 false);
                APP_LOG_INFO("main2 state change: %s -> %s, inject fail recovered=%u",
                             arm_state_name(state),
                             arm_state_name(ARM_STATE_PATROL_X),
                             recovered ? 1U : 0U);
                state = ARM_STATE_PATROL_X;
                APP_LOG_WARN("inject failed, retry with 5x5 detect window%s",
                             recovered ? "" : ", recovery height not ready");
            }
            break;
        }

        case ARM_STATE_POST_INJECT_PATROL:
            update_patrol_direction(&runtime, &patrol_dir);
            run_endpoint_speed_once(&motor_uart, &runtime,
                                    patrol_dir * ARM_PATROL_X_SPEED_MMPS,
                                    0.0, 0.0, false);
            update_patrol_direction(&runtime, &patrol_dir);
            if (now_ms - post_patrol_start_ms >= ARM_POST_INJECT_PATROL_MS) {
                send_main1_command_repeat(&vision_uart,
                                          cmd_detect_5x5,
                                          3U,
                                          20000);
                send_main1_command_repeat(&vision_uart,
                                          cmd_detect_resume,
                                          3U,
                                          20000);
                APP_LOG_INFO("main2 state change: %s -> %s, post patrol done",
                             arm_state_name(state),
                             arm_state_name(ARM_STATE_PATROL_X));
                state = ARM_STATE_PATROL_X;
                APP_LOG_INFO("post patrol done, resume detect");
            }
            break;

        case ARM_STATE_SHUTDOWN_RETURN_SAFE:
            if (!run_shutdown_return_zero_by_angle(&motor_uart,
                                                   &runtime,
                                                   shutdown_start_ms)) {
                APP_LOG_WARN("shutdown angle zero return failed, exit after stop");
            }
            running = 0;
            break;

        default:
            APP_LOG_INFO("main2 state change: %s -> %s, default fallback",
                         arm_state_name(state),
                         arm_state_name(ARM_STATE_PATROL_X));
            state = ARM_STATE_PATROL_X;
            break;
        }
    }

    pthread_join(control_thread, NULL);

    pthread_mutex_lock(&control_mutex);
    send_arm_stop_locked(&motor_uart, &runtime);
    pthread_mutex_unlock(&control_mutex);
    inject_control.Motor_stop();
    inject_control.Inject_stop(INJECT_NEEDLE_DIRECTION);

    notify_main1_exit(&vision_uart);
    ls2k0300_uart_deinit(&motor_uart);
    ls2k0300_uart_deinit(&vision_uart);
    pthread_mutex_destroy(&control_mutex);

    return 0;
}
