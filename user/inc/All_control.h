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


#define UART_INJECTION_PIN  UART4 /* 注入和旋转电机通信串口，目前使用 /dev/ttyS4 */
#define INJECTION_MOTOR_ADD  5     /* 注入电机地址 */
#define ROTATION_MOTOR_ADD   4     /* 旋转电机地址 */

/* 28 电机注入动作流程参数 */
#define INJECT_CHECK_BYTE                       0x6BU   /* X42 串口协议固定校验字节 */
#define INJECT_READ_BUS_CURRENT_CODE           0x26U   /* 读取总线电流返回功能码 */
#define INJECT_MOTOR_STATUS_CODE               0x3AU   /* 读取电机状态标志返回功能码 */
#define INJECT_ORIGIN_STATUS_CODE              0x3BU   /* 读取回零状态标志返回功能码 */
#define INJECT_ORIGIN_COMMAND_CODE             0x9AU   /* 触发回零命令应答功能码 */
#define INJECT_POSITION_CODE                   0xFBU   /* 直通限速位置模式返回功能码 */
#define INJECT_COMMAND_RESPONSE_SIZE           4U      /* 普通命令应答长度：地址+功能码+状态+校验 */
#define INJECT_CURRENT_RESPONSE_SIZE           5U      /* 电流读取应答长度：地址+功能码+高字节+低字节+校验 */
#define INJECT_DEFAULT_SPEED_RPM               100.0f  /* 注入和旋转电机默认速度，单位 RPM */
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
#define INJECT_COMMAND_STATUS_OK               0x02U   /* 命令接收正确，动作仍可能未完成 */
#define INJECT_COMMAND_STATUS_ALREADY_DONE     0x12U   /* 回零时已在零点/限位触发，电机不动 */
#define INJECT_COMMAND_STATUS_DONE             0x9FU   /* 动作执行完成的主动返回状态 */
#define INJECT_COMMAND_STATUS_PARAM_ERROR      0xE2U   /* 命令参数错误或条件不满足 */
#define INJECT_COMMAND_STATUS_FORMAT_ERROR     0xEEU   /* 命令格式错误 */
#define INJECT_ORIGIN_FIRST_POLL_DELAY_MS      1L      /* 触发回零后第一次读取状态前的等待时间，单位 ms */
#define INJECT_ORIGIN_STATUS_POLL_MS           10L     /* 回零状态轮询周期，单位 ms */
#define INJECT_ORIGIN_WAIT_TIMEOUT_MS          15000LL /* 回零总等待超时，单位 ms */
#define INJECT_ORIGIN_STABLE_DONE_POLLS        3U      /* 连续检测到回零完成的次数，防止刚启动误判 */
#define INJECT_POSITION_MODE_REL_CURRENT       0x02U   /* 位置模式：相对当前实时位置运动 */
#define ROTATE_ORIGIN_MODE_NEAREST_ZERO        0x00U   /* 旋转电机回零模式：单圈就近回零 */
#define ROTATE_ORIGIN_NEAREST_DONE_STATE       0x08U   /* 单圈就近回零兼容完成状态：部分驱动器完成后返回 0x0B */
#define ROTATE_DEFAULT_DIRECTION               0U      /* 旋转电机初始化相对转动方向 */
#define ROTATE_INIT_RELATIVE_DEGREES           90.0f   /* 旋转电机初始化后相对转动角度，单位度 */

class All_control{

public:
    /**
     * @brief 构造控制对象，初始化主电机位置控制状态。
     * @note 注入电机串口采用懒初始化，第一次调用 Inject_* 时才打开串口。
     */
    All_control();

    /**
     * @brief 析构控制对象，停止主电机并释放注入电机串口。
     */
    ~All_control();

    /**
     * @brief 主电机闭环控制周期函数。
     * @note 需要由外部按固定周期调用，用编码器位置和速度 PID 输出 PWM。
     */
    void Motor_control(void);

    /**
     * @brief 立即停止主推药电机，并清掉位置/速度环输出。
     * @note 用于注射中断、异常退出等需要快速停止推药的场景。
     */
    void Motor_stop(void);

    /**
     * @brief 设置主电机相对当前位置的目标位移。
     * @param delta_ml 相对位移，单位 ml，内部会按 POSITION_LIMIT_ML 限幅。
     */
    void Set_target_delta(float delta_ml);

    /**
     * @brief 触发主电机回到保存的零点位置。
     * @note 根据保存的编码器偏移和当前编码器值生成回零目标。
     */
    void Trigger_home(void);

    /**
     * @brief 将主电机当前位置设置为零点并保存。
     * @note 会停止主电机、复位编码器计数并写入零点保存值。
     */
    void Set_current_as_zero(void);

    /**
     * @brief 保存主电机当前相对零点的位置。
     */
    void Save_current_position(void);

    /**
     * @brief 查询主电机位置控制是否到达目标。
     * @return true 表示已到达或进入停止死区，false 表示仍在运动。
     */
    bool Is_arrived(void) const;

    /**
     * @brief 查询主电机回零是否结束。
     * @return true 表示回零结束，false 表示回零仍在进行。
     */
    bool Is_home_finished(void) const;

    /**
     * @brief 获取主推药电机当前相对零点位置。
     * @return 当前相对零点位置，单位 ml；编码器读取失败时返回 NAN。
     */
    float Motor_position_ml(void) const;

    /**
     * @brief 获取主推药电机当前目标位置。
     * @return 目标位置，单位 ml。
     */
    float Motor_target_position_ml(void) const;

    /**
     * @brief 获取主推药电机当前目标速度。
     * @return 目标速度，单位 ml/s。
     */
    float Motor_target_speed_mlps(void) const;

    /**
     * @brief 获取主推药电机滤波后的实测速度。
     * @return 实测速度，单位 ml/s。
     */
    float Motor_filtered_speed_mlps(void) const;

    /**
     * @brief 查询注入电机接触检测标志。
     * @return true 表示总线电流变化已超过接触阈值。
     */
    bool Is_contact(void) const;

    /**
     * @brief 查询注入电机接触后的相对位置运动是否到位。
     * @return true 表示最近一次 Inject_move_relative 已到位。
     */
    bool Is_inject_position_arrived(void) const;

    /**
     * @brief 查询旋转电机最近一次相对位置运动是否到位。
     * @return true 表示最近一次 Rotate_move_relative 已到位。
     */
    bool Is_rotate_position_arrived(void) const;

    /**
     * @brief 单次读取注入电机总线电流并更新 contact 状态。
     * @note 适合外部自行调度采样时调用；完整阻塞流程使用 Inject_run_until_contact。
     */
    void Inject_control(void);

    /**
     * @brief 设置注入电机恒速运行。
     * @param dir 方向，0/1 对应驱动器 CW/CCW。
     * @param speed 速度，单位 RPM。
     */
    void Inject_set_speed(uint8_t dir, uint16_t speed);

    /**
     * @brief 触发注入电机回到已设置的绝对零点并等待完成。
     * @return true 表示回零完成，false 表示超时、失败或被中断。
     */
    bool Inject_home_to_zero(void);

    /**
     * @brief 执行旋转电机初始化动作。
     * @note 流程：单圈就近回零，成功后方向 0 相对转动 90 度。
     * @return true 表示完整流程执行成功，false 表示通信失败、超时或被中断。
     */
    bool Rotate_control(void);

    /**
     * @brief 触发旋转电机单圈就近回零并等待完成。
     * @return true 表示回零完成，false 表示超时、失败或被中断。
     */
    bool Rotate_home_nearest(void);

    /**
     * @brief 旋转电机执行相对当前位置的角度运动并等待到位。
     * @param angle_degrees 相对角度，单位度；负数表示反向运动。
     * @param dir 正角度使用的默认方向，0/1 对应 CW/CCW。
     * @param speed_rpm 位置模式最大速度，单位 RPM。
     * @return true 表示指令成功且到位，false 表示指令失败、到位超时或被中断。
     */
    bool Rotate_move_relative(float angle_degrees, uint8_t dir, float speed_rpm);

    /**
     * @brief 停止旋转电机速度模式运动。
     * @param dir 停止命令沿用的方向位，0/1 对应驱动器 CW/CCW。
     */
    void Rotate_stop(uint8_t dir);

    /**
     * @brief 注入电机按恒定速度运行，直到电流阈值触发接触。
     * @param dir 方向，0/1 对应驱动器 CW/CCW。
     * @param speed_rpm 恒定速度，单位 RPM。
     * @return true 表示检测到接触并已停机，false 表示初始化失败、读电流失败或被中断。
     */
    bool Inject_run_until_contact(uint8_t dir, float speed_rpm);

    /**
     * @brief 注入电机执行相对当前位置的位置运动并等待到位。
     * @param relative_units 相对位置单位，1 表示 0.1 度；负数表示反向运动。
     * @param default_direction 正数 relative_units 使用的默认方向，0/1 对应 CW/CCW。
     * @param speed_rpm 位置模式最大速度，单位 RPM。
     * @return true 表示指令成功且到位，false 表示指令失败、到位超时或被中断。
     */
    bool Inject_move_relative(int32_t relative_units,
                              uint8_t default_direction,
                              float speed_rpm);

    /**
     * @brief 停止注入电机速度模式运动。
     * @param dir 停止命令沿用的方向位，0/1 对应驱动器 CW/CCW。
     */
    void Inject_stop(uint8_t dir);

    /**
     * @brief 设置外部运行标志，用于 Ctrl+C 等中断时退出阻塞流程。
     * @param running_flag 指向外部 volatile sig_atomic_t 标志；为 NULL 时不启用中断检查。
     */
    void Inject_set_running_flag(const volatile sig_atomic_t *running_flag);

    /**
     * @brief 绑定外部已初始化的 X42 控制串口。
     * @note main2 已经持有 UART4 时调用，避免 All_control 再次打开同一串口。
     *       绑定后析构不会 deinit 这个外部串口。
     * @param uart 外部串口句柄，必须在 All_control 生命周期内保持有效。
     */
    void Inject_attach_uart(ls2k0300_uart_t *uart);

    /**
     * @brief 获取注入电机最近一次计算出的总线电流滑动平均值。
     * @return 当前滑动平均电流原始值。
     */
    float Inject_current_average(void) const;

    /**
     * @brief 获取注入电机当前滑动平均电流相对基线的差值。
     * @return moving_average - baseline 的结果。
     */
    float Inject_current_baseline_delta(void) const;


private:
    typedef enum speed_pid_segment {
        SPEED_PID_SEG_LOW = 0,
        SPEED_PID_SEG_MID,
        SPEED_PID_SEG_HIGH,
        SPEED_PID_SEG_INVALID,
    } speed_pid_segment_t;

    /**
     * @brief 初始化主电机位置控制和编码器状态。
     */
    void Motor_control_init(void);

    /**
     * @brief 处理主电机目标死区、越过目标停止和禁止反向输出。
     * @param current_position_ml 当前相对零点位置，单位 ml。
     * @param raw_target_speed 位置环输出的原始目标速度，单位 ml/s。
     * @return true 表示应停机，false 表示可以继续按 setspeed 输出。
     */
    bool Handle_deadzone_and_direction(float current_position_ml,
                                       float raw_target_speed);

    /**
     * @brief 根据目标速度绝对值切换主电机速度环 PID 参数段。
     * @param target_speed 目标速度，单位 ml/s。
     */
    void Set_speed_pid_segment(float target_speed);
    EncoderData encoder_data;
    float setspeed; //ml/s  [-0.14,0.14]
    float setposition; //目标位置，单位 ml
    long long origin_count;
    Pid position_pid;
    Pid speed_pid;
    /**
     * @brief 计算两个 timespec 的时间差。
     * @param now 当前时间。
     * @param last 上一次时间。
     * @return 时间差，单位 s。
     */
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
    /**
     * @brief 计算三个浮点数的中值，用于速度采样抗尖峰。
     * @return 三个输入中的中位数。
     */
    float median3(float a, float b, float c);

    /**
     * @brief 一阶低通滤波。
     * @param input 当前输入值。
     * @param last_output 上一次滤波输出。
     * @param delta 滤波系数，内部限制到 0~1。
     * @return 本次滤波输出。
     */
    float low_pass_speed(float input, float last_output, float delta);
    float sample_speed[3] = {0};

    /**
     * @brief 释放主电机控制资源并停止主电机输出。
     */
    void Motor_control_deinit(void);


    /**
     * @brief 确保注入电机串口已初始化。
     * @return true 表示串口可用，false 表示初始化失败。
     */
    bool Ensure_inject_uart_ready(void);

    /**
     * @brief 初始化注入电机串口和接触检测状态。
     * @return true 表示初始化成功，false 表示初始化失败。
     */
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
    bool rotate_position_arrived;
    bool injection_uart_ready;
    bool injection_uart_external;
    const volatile sig_atomic_t *inject_running_flag;
    /**
     * @brief 注入流程中使用的可中断毫秒延时。
     * @param milliseconds 延时时间，单位 ms。
     */
    void Inject_sleep_ms(long milliseconds) const;

    /**
     * @brief 查询注入流程是否应继续运行。
     * @return true 表示未收到外部中断，false 表示应退出当前阻塞流程。
     */
    bool Inject_should_continue(void) const;

    /**
     * @brief 获取单调时钟时间。
     * @return 当前单调时钟时间，单位 ms。
     */
    int64_t Inject_monotonic_time_ms(void) const;

    /**
     * @brief 打印串口接收数据，便于调试非法应答。
     * @param prefix 打印前缀。
     * @param data 数据缓冲区。
     * @param size 数据长度；小于等于 0 时只打印长度。
     */
    void Inject_print_bytes(const char *prefix, const uint8_t *data,
                            ssize_t size) const;

    /**
     * @brief 在超时时间内读取指定长度的串口数据。
     * @param buffer 接收缓冲区。
     * @param size 期望读取字节数。
     * @param timeout_ms 总超时时间，单位 ms。
     * @return 实际读取字节数；失败或被中断返回 -1。
     */
    ssize_t Inject_read_exact_timeout(uint8_t *buffer, size_t size,
                                      int timeout_ms);

    /**
     * @brief 重置注入电机接触检测滤波、基线和 contact 标志。
     */
    void Inject_reset_contact_state(void);

    /**
     * @brief 更新总线电流滑动平均。
     * @param sample 本次滤波后的电流采样原始值。
     * @return 当前滑动平均值。
     */
    float Inject_update_moving_average(float sample);

    /**
     * @brief 判断回零状态字是否表示回零完成且编码器、校准就绪。
     * @param status 回零状态标志字节。
     * @return true 表示回零完成。
     */
    bool Inject_origin_status_done(uint8_t status) const;

    /**
     * @brief 触发指定 X42 电机回零并等待完成。
     * @param addr 电机地址。
     * @param name 打印用电机名称。
     * @param origin_mode 回零模式。
     * @return true 表示回零完成。
     */
    bool X42_home_and_wait(uint8_t addr, const char *name, uint8_t origin_mode);

    /**
     * @brief 读取指定 X42 电机回零状态。
     * @param addr 电机地址。
     * @param name 打印用电机名称。
     * @param status 输出回零状态字节。
     * @return true 表示读取并校验成功。
     */
    bool X42_read_origin_status(uint8_t addr, const char *name, uint8_t *status);

    /**
     * @brief 读取指定 X42 电机系统状态。
     * @param addr 电机地址。
     * @param name 打印用电机名称。
     * @param status 输出系统状态字节。
     * @return true 表示读取并校验成功。
     */
    bool X42_read_motor_status(uint8_t addr, const char *name, uint8_t *status);

    /**
     * @brief 等待指定 X42 电机位置指令应答。
     * @param addr 电机地址。
     * @param name 打印用电机名称。
     * @param command_status 输出指令返回状态；0 表示未收到有效应答。
     * @return true 表示未收到错误状态。
     */
    bool X42_wait_position_ack(uint8_t addr, const char *name,
                               uint8_t *command_status);

    /**
     * @brief 等待指定 X42 电机位置运动到位。
     * @param addr 电机地址。
     * @param name 打印用电机名称。
     * @param arrived 输出到位标志。
     * @return true 表示连续检测到到位状态。
     */
    bool X42_wait_position_arrived(uint8_t addr, const char *name,
                                   bool *arrived);

    /**
     * @brief 读取注入电机回零状态标志。
     * @param status 输出回零状态标志字节。
     * @return true 表示读取并校验成功。
     */
    bool Inject_read_origin_status(uint8_t *status);

    /**
     * @brief 读取注入电机总线电流。
     * @param current 输出总线电流原始值。
     * @return true 表示读取并校验成功。
     */
    bool Inject_read_bus_current(uint16_t *current);

    /**
     * @brief 读取注入电机系统状态标志。
     * @param status 输出状态标志字节。
     * @return true 表示读取并校验成功。
     */
    bool Inject_read_motor_status(uint8_t *status);

    /**
     * @brief 等待注入电机位置模式指令应答。
     * @param command_status 输出指令返回状态；0 表示未收到有效应答。
     * @return true 表示未收到错误状态，false 表示驱动器返回错误。
     */
    bool Inject_wait_position_ack(uint8_t *command_status);

    /**
     * @brief 等待注入电机位置运动到位。
     * @return true 表示连续检测到到位状态，false 表示超时或被中断。
     */
    bool Inject_wait_position_arrived(void);

    /**
     * @brief 发送注入电机速度模式命令。
     * @param dir 方向，0/1 对应驱动器 CW/CCW。
     * @param speed_rpm 速度，单位 RPM。
     * @return true 表示命令发送后串口状态正常。
     */
    bool Inject_set_velocity(uint8_t dir, float speed_rpm);

    /**
     * @brief 释放注入电机串口资源，并在释放前发送 0 速命令。
     */
    void Inject_control_deinit(void);


};



#endif /* _ALL_CONTROL_H_.h */
