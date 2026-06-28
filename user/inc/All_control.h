#ifndef _ALL_CONTROL_H_
#define _ALL_CONTROL_H_

#include <cstddef>
#include <cstdint>
#include <signal.h>
#include <sys/types.h>

#include "My_inc.h"

/* 编码器与物理量换算 */
#define COUNT_PER_1ML       66752.0f /* 1ml 对应的编码器计数：1 * 8 * 7 * 298 * 4 */

/* 位置环与速度目标限制 */
#define MAX_SPEED           0.14f    /* 位置环输出的最大目标速度，单位 ml/s */
#define MIN_TARGET_SPEED    0.005f   /* 非零目标速度下限，用于克服低速死区，单位 ml/s */
#define MAX_DUTY            10000.0f /* PWM 占空比满量程，范围 0~10000 */
#define POSITION_LIMIT_ML   5.0f     /* 单次目标位置增量限幅，单位 ml */
#define POSITION_LOOP_DIV   10U      /* 位置环分频：每 10 次 Motor_control 调用计算一次位置环 */

/* 速度测量滤波与停止判定 */
#define LOW_PASS            0.355f   /* 速度一阶低通系数：本次值权重，约等于 33ms 周期、0.06s 时间常数 */
#define DEAD_POS            0.0015f  /* 目标前提前停止死区，进入该距离后目标速度置 0，单位 ml */
#define POSITION_ERROR_EPS  0.0001f  /* 位置误差零点阈值，避免编码器抖动导致误判，单位 ml */
#define TARGET_SPEED_EPS    0.0001f  /* 目标速度零点阈值，单位 ml/s */
#define ZERO_SPEED_EPS      0.0001f  /* 实测速度零点阈值，单位 ml/s */
#define ZERO_SPEED_CYCLES   3U       /* 连续多少个周期速度为 0，认为电机处于静止启动状态 */

/* 速度环分段 PID 参数，按目标速度绝对值选择 */
#define SPEED_PID_LOW_LIMIT  0.01f    /* 低速段上限，|target_speed| < 0.01 ml/s */
#define SPEED_PID_MID_LIMIT  0.10f    /* 中速段上限，0.01~0.10 ml/s */
#define SPEED_PID_LOW_KP     68000.0f /* 低速段速度环 Kp */
#define SPEED_PID_MID_KP     62000.0f /* 中速段速度环 Kp */
#define SPEED_PID_HIGH_KP    38000.0f /* 高速段速度环 Kp */
#define SPEED_PID_LOW_KI     7000.0f  /* 低速段速度环 Ki */
#define SPEED_PID_DEFAULT_KI 7000.0f  /* 中速和高速段速度环 Ki */

/* 低速启动辅助 */
#define LOW_SPEED_PRELOAD_LIMIT 0.05f   /* 低于该目标速度且静止时，允许预置启动占空比，单位 ml/s */
#define LOW_SPEED_PRELOAD_DUTY  4000.0f /* 低速静止启动时预置到速度 PID 内部输出的占空比 */


#define UART_INJECTION_PIN  UART4 /* 注入电机通信串口，目前使用 /dev/ttyS4 */
#define INJECTION_MOTOR_ADD  1     /* 注入电机地址 */

/* 28 电机注入动作流程参数 */
#define INJECT_CHECK_BYTE                       0x6BU   /* X42 串口协议固定校验字节 */
#define INJECT_READ_BUS_CURRENT_CODE           0x26U   /* 读取总线电流返回功能码 */
#define INJECT_MOTOR_STATUS_CODE               0x3AU   /* 读取电机状态标志返回功能码 */
#define INJECT_ORIGIN_STATUS_CODE              0x3BU   /* 读取回零状态标志返回功能码 */
#define INJECT_POSITION_CODE                   0xFBU   /* 直通限速位置模式返回功能码 */
#define INJECT_COMMAND_RESPONSE_SIZE           4U      /* 普通命令应答长度：地址+功能码+状态+校验 */
#define INJECT_CURRENT_RESPONSE_SIZE           5U      /* 电流读取应答长度：地址+功能码+高字节+低字节+校验 */
#define INJECT_VELOCITY_RAMP_RPM_PER_S         100U    /* 恒速运动速度斜率，单位 RPM/s */
#define INJECT_QUERY_PERIOD_MS                 10L     /* 接触检测时读取电流的轮询周期，单位 ms */
#define INJECT_STATUS_PRINT_INTERVAL           10UL    /* 接触检测调试打印间隔，单位采样点 */
#define INJECT_CURRENT_READ_TIMEOUT_MS         120     /* 读取总线电流单次等待超时，单位 ms */
#define INJECT_STATUS_READ_TIMEOUT_MS          200     /* 读取状态标志单次等待超时，单位 ms */
#define INJECT_POSITION_RESPONSE_TIMEOUT_MS    200     /* 位置指令应答等待超时，单位 ms */
#define INJECT_POSITION_WAIT_TIMEOUT_MS        15000LL /* 位置运动到位总等待超时，单位 ms */
#define INJECT_POSITION_STATUS_POLL_MS         20L     /* 位置运动到位状态轮询周期，单位 ms */
#define INJECT_POSITION_STABLE_DONE_POLLS      3U      /* 连续检测到到位状态的次数，防止瞬时误判 */
#define INJECT_POSITION_ARRIVED_MASK           0x02U   /* 电机状态标志中表示到位的 bit1 */
#define INJECT_CURRENT_MAX_CONSECUTIVE_ERRORS  20U     /* 连续电流读取失败上限，超过则退出接触检测 */
#define INJECT_MOVING_AVERAGE_SIZE             50U     /* 总线电流滑动平均窗口大小 */
#define INJECT_CONTACT_DELTA_THRESHOLD         3.0f    /* 接触阈值：滑动平均电流减基线超过该值触发 contact */
#define INJECT_ORIGIN_MODE_ABSOLUTE_ZERO       0x04U   /* 回零模式：回到已设置的绝对位置零点 */
#define INJECT_ORIGIN_STATUS_READY_MASK        0x03U   /* 回零状态中 Enc_Rdy 和 Cal_Rdy 的掩码 */
#define INJECT_ORIGIN_STATUS_READY             0x03U   /* 编码器和校准均就绪 */
#define INJECT_ORIGIN_STATE_MASK               0x0CU   /* 回零状态中 Org_SF/Org_CF 两位掩码 */
#define INJECT_ORIGIN_STATE_BUSY               0x04U   /* 正在回零：Org_SF=1, Org_CF=0 */
#define INJECT_ORIGIN_STATE_FAILED             0x08U   /* 回零失败：Org_SF=0, Org_CF=1 */
#define INJECT_ORIGIN_STATE_DONE               0x00U   /* 回零完成或默认完成状态：Org_SF=0, Org_CF=0 */
#define INJECT_ORIGIN_FIRST_POLL_DELAY_MS      1L      /* 触发回零后第一次读取状态前的等待时间，单位 ms */
#define INJECT_ORIGIN_STATUS_POLL_MS           10L     /* 回零状态轮询周期，单位 ms */
#define INJECT_ORIGIN_WAIT_TIMEOUT_MS          15000LL /* 回零总等待超时，单位 ms */
#define INJECT_ORIGIN_STABLE_DONE_POLLS        3U      /* 连续检测到回零完成的次数，防止刚启动误判 */
#define INJECT_POSITION_MODE_REL_CURRENT       0x02U   /* 位置模式：相对当前实时位置运动 */

class All_control{

public:
    All_control();
    ~All_control();
    void Motor_control(void);
    void Set_target_delta(float delta_ml);
    void Trigger_home(void);
    void Set_current_as_zero(void);
    void Save_current_position(void);
    bool Is_arrived(void) const;
    bool Is_home_finished(void) const;
    bool Is_contact(void) const;
    bool Is_inject_position_arrived(void) const;

    void Inject_control(void);
    void Inject_set_speed(uint8_t dir, uint16_t speed);
    bool Inject_home_to_zero(void);
    bool Inject_run_until_contact(uint8_t dir, float speed_rpm);
    bool Inject_move_relative(int32_t relative_units,
                              uint8_t default_direction,
                              float speed_rpm);
    void Inject_stop(uint8_t dir);
    void Inject_set_running_flag(const volatile sig_atomic_t *running_flag);
    float Inject_current_average(void) const;
    float Inject_current_baseline_delta(void) const;


private:
    typedef enum speed_pid_segment {
        SPEED_PID_SEG_LOW = 0,
        SPEED_PID_SEG_MID,
        SPEED_PID_SEG_HIGH,
        SPEED_PID_SEG_INVALID,
    } speed_pid_segment_t;

    void Motor_control_init(void);

    bool Handle_deadzone_and_direction(float current_position_ml, float raw_target_speed);
    void Set_speed_pid_segment(float target_speed);
    EncoderData encoder_data;
    float setspeed; //ml/s  [-0.14,0.14]
    float setposition; //目标位置，单位 ml
    long long origin_count;
    Pid position_pid;
    Pid speed_pid;
    double timespec_diff_sec(const timespec *now, const timespec *last);
    timespec last_time;
    uint8_t period_count;
    uint8_t zero_speed_count;
    uint8_t speed_sample_count;
    bool position_error_ready;
    bool position_cross_stopped;
    bool position_arrived; //电机到位标志：进入目标死区或越过目标停止后置 true
    bool homing_active; //回零过程中为 true
    bool homing_finished; //回零结束标志
    bool zero_start_preloaded;
    bool speed_filter_ready;
    int motion_direction;
    float last_position_error;
    float filtered_speed;
    speed_pid_segment_t speed_pid_segment;
    Motor motor;
    float median3(float a, float b, float c);
    float low_pass_speed(float input, float last_output, float delta);
    float sample_speed[3] = {0};

    void Motor_control_deinit(void);


    bool Ensure_inject_uart_ready(void);
    bool Inject_control_init(void);
    ls2k0300_uart_t uart_injection;
    uint8_t rec_buf[5];
    uint16_t sample_current[2];
    float current_average_window[INJECT_MOVING_AVERAGE_SIZE];
    float current_average_sum;
    float current_average_value;
    float current_baseline;
    float current_baseline_delta;
    uint16_t current_average_index;
    uint16_t current_average_count;
    uint16_t current_error_count;
    unsigned long current_sample_index;
    bool current_filter_ready;
    bool current_baseline_ready;
    bool contact;
    bool inject_position_arrived;
    bool injection_uart_ready;
    const volatile sig_atomic_t *inject_running_flag;
    void Inject_sleep_ms(long milliseconds) const;
    bool Inject_should_continue(void) const;
    int64_t Inject_monotonic_time_ms(void) const;
    void Inject_print_bytes(const char *prefix, const uint8_t *data,
                            ssize_t size) const;
    ssize_t Inject_read_exact_timeout(uint8_t *buffer, size_t size,
                                      int timeout_ms);
    void Inject_reset_contact_state(void);
    float Inject_update_moving_average(float sample);
    bool Inject_origin_status_done(uint8_t status) const;
    bool Inject_read_origin_status(uint8_t *status);
    bool Inject_read_bus_current(uint16_t *current);
    bool Inject_read_motor_status(uint8_t *status);
    bool Inject_wait_position_ack(uint8_t *command_status);
    bool Inject_wait_position_arrived(void);
    bool Inject_set_velocity(uint8_t dir, float speed_rpm);
    void Inject_control_deinit(void);


};



#endif /* _ALL_CONTROL_H_.h */
