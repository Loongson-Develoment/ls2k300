#include "My_inc.h"
#include "AppLog.h"

#include <chrono>
#include <cmath>
#include <csignal>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <time.h>
#include <vector>
#include <pthread.h>


static volatile std::sig_atomic_t running = 1;
static volatile std::sig_atomic_t main2_handshake_ready = 0;
static const struct timespec period = {0, 330 * 1000 * 1000}; // 330ms
static const char main1_handshake_cmd[] = "MAIN1_HELLO\n";
static const char main1_handshake_ack[] = "MAIN2_ACK";
static const char main1_exit_cmd[] = "MAIN1_EXIT\n";
static const char main2_exit_cmd[] = "MAIN2_EXIT";
static const char cmd_inject_start[] = "CMD:INJECT_START\n";
static const char cmd_target_lost[] = "CMD:TARGET_LOST\n";
static const char cmd_detect_pause[] = "CMD:DETECT_PAUSE";
static const char cmd_detect_resume[] = "CMD:DETECT_RESUME";
static const char cmd_detect_5x5[] = "CMD:DETECT_5X5";
static const char cmd_detect_150x150[] = "CMD:DETECT_150X150";
static const char cmd_detect_10x10[] = "CMD:DETECT_10X10";
static const char cmd_inject_done[] = "CMD:INJECT_DONE";
static const char cmd_inject_fail[] = "CMD:INJECT_FAIL";
static DetectResult max_result;
static bool has_max_result = false;
static pthread_mutex_t max_result_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint16_t x_pixel_offset = 0;
static uint16_t y_pixel_offset = 0;

/* Red color block detector is disabled; main1 now uses the ncnn neck model. */
#define MAIN1_USE_COLOR_BLOCK 0
#define COLOR_BLOCK_RED_HUE_LOW_1 0
#define COLOR_BLOCK_RED_HUE_HIGH_1 10
#define COLOR_BLOCK_RED_HUE_LOW_2 170
#define COLOR_BLOCK_RED_HUE_HIGH_2 179
#define COLOR_BLOCK_MIN_SATURATION 80
#define COLOR_BLOCK_MIN_VALUE 80
#define COLOR_BLOCK_MIN_AREA 200.0

#define MAIN1_CENTER_HALF_5X5_PX 5.0f
#define MAIN1_CENTER_HALF_INJECT_PX 75.0f
#define MAIN1_STATE_PRINT_PERIOD_MS 1000LL
#define MAIN1_VERBOSE_DETECT_LOG 0
#define MAIN1_VERBOSE_SPEED_LOG 0

typedef enum {
    MAIN1_STATE_TRACK_TARGET = 0,
    MAIN1_STATE_INJECTION_MONITOR,
    MAIN1_STATE_PAUSED,
} Main1State;

static const char *main1_state_name(Main1State state)
{
    switch (state) {
    case MAIN1_STATE_TRACK_TARGET:
        return "TRACK_TARGET";
    case MAIN1_STATE_INJECTION_MONITOR:
        return "INJECTION_MONITOR";
    case MAIN1_STATE_PAUSED:
        return "PAUSED";
    default:
        return "UNKNOWN";
    }
}

static void clear_detection_cache(void)
{
    pthread_mutex_lock(&max_result_mutex);
    has_max_result = false;
    pthread_mutex_unlock(&max_result_mutex);
}

void signal_handler(int signum) {
    (void)signum;
    running = 0;
}

static double timespec_diff_ms(const struct timespec& start, const struct timespec& end)
{
    return (end.tv_sec - start.tv_sec) * 1000.0 +
           (end.tv_nsec - start.tv_nsec) / 1000000.0;
}

static bool detect_color_block(const cv::Mat& frame, DetectResult *result)
{
    if (frame.empty() || result == NULL) {
        return false;
    }

    cv::Mat hsv;
    cv::Mat mask_low;
    cv::Mat mask_high;
    cv::Mat mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::inRange(hsv,
                cv::Scalar(COLOR_BLOCK_RED_HUE_LOW_1,
                           COLOR_BLOCK_MIN_SATURATION,
                           COLOR_BLOCK_MIN_VALUE),
                cv::Scalar(COLOR_BLOCK_RED_HUE_HIGH_1, 255, 255),
                mask_low);
    cv::inRange(hsv,
                cv::Scalar(COLOR_BLOCK_RED_HUE_LOW_2,
                           COLOR_BLOCK_MIN_SATURATION,
                           COLOR_BLOCK_MIN_VALUE),
                cv::Scalar(COLOR_BLOCK_RED_HUE_HIGH_2, 255, 255),
                mask_high);
    cv::bitwise_or(mask_low, mask_high, mask);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
                                               cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    double best_area = 0.0;
    cv::Rect best_rect;
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area > best_area) {
            best_area = area;
            best_rect = cv::boundingRect(contours[i]);
        }
    }

    if (best_area < COLOR_BLOCK_MIN_AREA) {
        return false;
    }

    result->center_x = best_rect.x + best_rect.width * 0.5f;
    result->center_y = best_rect.y + best_rect.height * 0.5f;
    result->width = (float)best_rect.width;
    result->height = (float)best_rect.height;
    result->score = (float)best_area;

    return true;
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

static bool read_line_timeout(ls2k0300_uart_t *uart, char *line,
                              size_t line_size, int timeout_ms)
{
    size_t pos = 0;
    int64_t deadline_ms = monotonic_time_ms() + timeout_ms;

    if (line == NULL || line_size == 0U) {
        return false;
    }

    while (running) {
        int64_t now_ms = monotonic_time_ms();
        int remaining_ms = (int)(deadline_ms - now_ms);
        if (remaining_ms <= 0) {
            return false;
        }

        uint8_t ch;
        ssize_t n = ls2k0300_uart_read(uart, &ch, 1);
        if (n <= 0) {
            if (n < 0 && errno == EINTR) {
                continue;
            }
            if (n == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
                usleep(1000);
                continue;
            }
            return false;
        }

        if (ch == '\r') {
            continue;
        }

        if (ch == '\n') {
            line[pos] = '\0';
            return pos > 0U;
        }

        if (pos < line_size - 1U) {
            line[pos++] = (char)ch;
        } else {
            pos = 0;
        }
    }

    return false;
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

static void notify_main2_exit(ls2k0300_uart_t *uart)
{
    if (uart == NULL) {
        return;
    }

    APP_LOG_INFO("notify main2 exit");
    for (int i = 0; i < 10; ++i) {
        if (!uart_write_all(uart, main1_exit_cmd)) {
            APP_LOG_WARN("send main2 exit failed on try %d", i + 1);
        }
        usleep(20000);
    }
}

static void uart_send_line(ls2k0300_uart_t *uart, const char *line)
{
    if (uart == NULL || line == NULL) {
        return;
    }

    (void)uart_write_all(uart, line);
}

static void send_endpoint_speed(ls2k0300_uart_t *uart,
                                float vx,
                                float vy,
                                float vz)
{
    char sendbuffer[64];
    int send_len = snprintf(sendbuffer, sizeof(sendbuffer),
                            "X:%.2f,Y:%.2f,Z:%.2f\n", vx, vy, vz);
    if (send_len > 0) {
        if (!uart_write_all(uart, sendbuffer)) {
            APP_LOG_ERROR("could not send endpoint speed");
            running = 0;
            return;
        }
#if MAIN1_VERBOSE_SPEED_LOG
        APP_LOG_INFO("main1 tx endpoint speed: vx=%.2f, vy=%.2f, vz=%.2f",
                     vx, vy, vz);
#endif
    }
}

static bool result_inside_center_window(const DetectResult& result,
                                        float half_window_px)
{
    float center_x = CAMERA_WIDTH / 2.0f + (float)x_pixel_offset;
    float center_y = CAMERA_HEIGHT / 2.0f + (float)y_pixel_offset;
    float dx = result.center_x - center_x;
    float dy = result.center_y - center_y;

    return std::fabs(dx) <= half_window_px &&
           std::fabs(dy) <= half_window_px;
}

static bool wait_main2_handshake(ls2k0300_uart_t *uart)
{
    for (unsigned int i = 0U; running && i < 10U; ++i) {
        if (!uart_write_all(uart, main1_handshake_cmd)) {
            APP_LOG_ERROR("could not send main2 handshake");
            return false;
        }
        usleep(20000);
    }

    APP_LOG_INFO("main1 handshake announced, start detection without ack");
    return running != 0;
}


void *transmit_function(void *arg) {
    (void)arg;
    DetectResult transmit_result;
    ls2k0300_uart_t uart1;
    memset(&uart1, 0, sizeof(uart1));
    if (ls2k0300_uart_init(&uart1, UART1, B115200, LS_UART_STOP1, LS_UART_DATA8, LS_UART_PARITY_NONE, LS_UART_MODE_NON_BLOCKING) != 0) {
        APP_LOG_ERROR("could not init uart");
        return NULL;
    }

    if (!wait_main2_handshake(&uart1)) {
        running = 0;
        notify_main2_exit(&uart1);
        ls2k0300_uart_deinit(&uart1);
        return NULL;
    }
    clear_detection_cache();
    main2_handshake_ready = 1;

    Pid pid_x(5.0f, 0.0f, 0.15f,100);
    Pid pid_y(5.0f, 0.0f, 0.15f,100);
    // Pid pid_z(0.0f, 0.0f, 0.0f);
    Main1State state = MAIN1_STATE_TRACK_TARGET;
    float active_center_half_px = MAIN1_CENTER_HALF_5X5_PX;
    int64_t last_state_print_ms = 0;
    APP_LOG_INFO("main1 state: track target after handshake");
    
    while (running) 
    {
        bool local_has_result = false;

        pthread_mutex_lock(&max_result_mutex);
        if (has_max_result) {
            transmit_result = max_result;
            local_has_result = true;
        }
        pthread_mutex_unlock(&max_result_mutex);

        char rx_line[64];
        while (read_line_nonblocking(&uart1, rx_line, sizeof(rx_line))) {
            APP_LOG_INFO("main1 rx command: %s", rx_line);
            if (strcmp(rx_line, main2_exit_cmd) == 0) {
                APP_LOG_INFO("main2 requested exit");
                running = 0;
                break;
            }
            if (strcmp(rx_line, cmd_detect_pause) == 0) {
                state = MAIN1_STATE_PAUSED;
                pid_x.Set_output(0.0f);
                pid_y.Set_output(0.0f);
                APP_LOG_INFO("main1 state: paused");
                continue;
            }
            if (strcmp(rx_line, cmd_detect_resume) == 0) {
                active_center_half_px = MAIN1_CENTER_HALF_5X5_PX;
                state = MAIN1_STATE_TRACK_TARGET;
                pid_x.Set_output(0.0f);
                pid_y.Set_output(0.0f);
                APP_LOG_INFO("main1 state: track target, window 5x5");
                continue;
            }
            if (strcmp(rx_line, cmd_detect_5x5) == 0) {
                active_center_half_px = MAIN1_CENTER_HALF_5X5_PX;
                state = MAIN1_STATE_TRACK_TARGET;
                pid_x.Set_output(0.0f);
                pid_y.Set_output(0.0f);
                APP_LOG_INFO("detect window set to 5x5");
                continue;
            }
            if (strcmp(rx_line, cmd_detect_150x150) == 0 ||
                strcmp(rx_line, cmd_detect_10x10) == 0) {
                active_center_half_px = MAIN1_CENTER_HALF_INJECT_PX;
                state = MAIN1_STATE_INJECTION_MONITOR;
                pid_x.Set_output(0.0f);
                pid_y.Set_output(0.0f);
                APP_LOG_INFO("detect window set to 150x150, state injection monitor");
                continue;
            }
            if (strcmp(rx_line, cmd_inject_done) == 0 ||
                strcmp(rx_line, cmd_inject_fail) == 0) {
                pid_x.Set_output(0.0f);
                pid_y.Set_output(0.0f);
                APP_LOG_INFO("main1 inject result: %s", rx_line);
                continue;
            }
            if (strcmp(rx_line, main1_handshake_ack) != 0) {
                APP_LOG_WARN("unexpected main2 rx: %s", rx_line);
            }
        }

        if (!running) {
            break;
        }

        int64_t now_ms = monotonic_time_ms();
        if (last_state_print_ms <= 0 ||
            now_ms - last_state_print_ms >= MAIN1_STATE_PRINT_PERIOD_MS) {
            float center_x = CAMERA_WIDTH / 2.0f + (float)x_pixel_offset;
            float center_y = CAMERA_HEIGHT / 2.0f + (float)y_pixel_offset;
            float dx = local_has_result ?
                       transmit_result.center_x - center_x : 0.0f;
            float dy = local_has_result ?
                       transmit_result.center_y - center_y : 0.0f;
            bool centered = local_has_result &&
                            result_inside_center_window(transmit_result,
                                                        active_center_half_px);
            APP_LOG_INFO("main1 state heartbeat: state=%s, detected=%u, "
                         "centered=%u, dx=%.1f, dy=%.1f, window=%.1f",
                         main1_state_name(state),
                         local_has_result ? 1U : 0U,
                         centered ? 1U : 0U,
                         dx,
                         dy,
                         active_center_half_px);
            last_state_print_ms = now_ms;
        }

        switch (state) {
        case MAIN1_STATE_TRACK_TARGET:
            if (local_has_result) {
                if (result_inside_center_window(transmit_result,
                                                active_center_half_px)) {
                    send_endpoint_speed(&uart1, 0.0f, 0.0f, 0.0f);
                    uart_send_line(&uart1, cmd_inject_start);
                    state = MAIN1_STATE_INJECTION_MONITOR;
                    pid_x.Set_output(0.0f);
                    pid_y.Set_output(0.0f);
                    APP_LOG_INFO("target centered, inject start");
                } else {
                    float vx = pid_x.calculate(CAMERA_WIDTH / 2 +
                                                   x_pixel_offset,
                                               transmit_result.center_x);
                    float vz = pid_y.calculate(CAMERA_HEIGHT / 2 +
                                                   y_pixel_offset,
                                               transmit_result.center_y);
                    send_endpoint_speed(&uart1, vx, 0.0f, vz);
                }
            } else {
                pid_x.Set_output(0.0f);
                pid_y.Set_output(0.0f);
            }
            break;

        case MAIN1_STATE_INJECTION_MONITOR:
            if (local_has_result &&
                !result_inside_center_window(transmit_result,
                                             MAIN1_CENTER_HALF_INJECT_PX)) {
                send_endpoint_speed(&uart1, 0.0f, 0.0f, 0.0f);
                uart_send_line(&uart1, cmd_target_lost);
                state = MAIN1_STATE_TRACK_TARGET;
                active_center_half_px = MAIN1_CENTER_HALF_5X5_PX;
                pid_x.Set_output(0.0f);
                pid_y.Set_output(0.0f);
                APP_LOG_WARN("target moved out of 150x150, abort inject");
            }
            break;

        case MAIN1_STATE_PAUSED:
        default:
            pid_x.Set_output(0.0f);
            pid_y.Set_output(0.0f);
            break;
        }

        nanosleep(&period, NULL);
    }
    notify_main2_exit(&uart1);
    ls2k0300_uart_deinit(&uart1);
    return NULL;
}

int main()
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

#if !MAIN1_USE_COLOR_BLOCK
    NcnnDetector detector;
#endif

    Camera camera;
    if (!camera.Open()) {
        return 1;
    }

    icst7735_t lcd;
    memset(&lcd, 0, sizeof(lcd));
    if (icst7735_init(&lcd, LS_SPI2, 80000000U, LCD_DC_PIN, CD_RST_PIN, LCD_BL_PIN, 160, 128, 0, 0) != 0) {
        APP_LOG_ERROR("could not init lcd");
        return 1;
    }

#if !MAIN1_USE_COLOR_BLOCK
    if (!detector.IsLoaded()) 
    {
        APP_LOG_ERROR("could not load ncnn model: %s",
                      detector.LastError().c_str());
        icst7735_deinit(&lcd);
        return 1;
    }
    APP_LOG_INFO("use ncnn cow neck detector, color block detector disabled");
#else
    APP_LOG_INFO("use color block detector, ncnn detector disabled");
#endif

    pthread_t transmit_thread;
    if (pthread_create(&transmit_thread, NULL, transmit_function, NULL) != 0) {
        APP_LOG_ERROR("could not create transmit thread");
        icst7735_deinit(&lcd);
        return 1;
    }

    struct timespec start_time, end1_time, end2_time;
    int64_t last_detect_state_print_ms = 0;

    while (running) {
        cv::Mat frame;
        if (!camera.ReadFrame(frame)) {
            continue;
        }

        if (!main2_handshake_ready) {
            int64_t now_ms = monotonic_time_ms();
            clear_detection_cache();
            if (last_detect_state_print_ms <= 0 ||
                now_ms - last_detect_state_print_ms >=
                    MAIN1_STATE_PRINT_PERIOD_MS) {
                APP_LOG_INFO("main1 detect heartbeat: waiting handshake");
                last_detect_state_print_ms = now_ms;
            }
            (void)icst7735_show_camera_bgr888(&lcd,
                                              frame.data,
                                              (uint16_t)frame.cols,
                                              (uint16_t)frame.rows,
                                              (uint32_t)frame.step);
            continue;
        }

#if MAIN1_USE_COLOR_BLOCK
        clock_gettime(CLOCK_MONOTONIC, &start_time);

        DetectResult current_result;
        bool found_color_block = detect_color_block(frame, &current_result);

        clock_gettime(CLOCK_MONOTONIC, &end1_time);

        if (found_color_block) {
            pthread_mutex_lock(&max_result_mutex);
            max_result = current_result;
            has_max_result = true;
            pthread_mutex_unlock(&max_result_mutex);

#if MAIN1_VERBOSE_DETECT_LOG
            APP_LOG_INFO("color block center=(%.1f,%.1f), area=%.1f",
                         current_result.center_x,
                         current_result.center_y,
                         current_result.score);
#endif

            cv::rectangle(frame,
                          cv::Rect((int)(current_result.center_x -
                                         current_result.width / 2.0f),
                                   (int)(current_result.center_y -
                                         current_result.height / 2.0f),
                                   (int)current_result.width,
                                   (int)current_result.height),
                          cv::Scalar(0, 255, 0), 4);
        } else {
            pthread_mutex_lock(&max_result_mutex);
            has_max_result = false;
            pthread_mutex_unlock(&max_result_mutex);
        }

#if MAIN1_VERBOSE_DETECT_LOG
        APP_LOG_INFO("色块检测耗时：%.3fms",
                     timespec_diff_ms(start_time, end1_time));
#endif
#else
        if (detector.IsLoaded()) {
            bool detect_time_valid = false;

            clock_gettime(CLOCK_MONOTONIC, &start_time);

            ncnn::Mat out = detector.Reasoning(frame);

            clock_gettime(CLOCK_MONOTONIC, &end1_time);
            
            std::vector<DetectResult> results;
            if (!detector.Detect(out, frame.cols, frame.rows, results)) {
                APP_LOG_WARN("ncnn detect failed: %s",
                             detector.LastError().c_str());
            } else {
                clock_gettime(CLOCK_MONOTONIC, &end2_time);
                detect_time_valid = true;

                APP_LOG_INFO("detections: %zu", results.size());

                if (results.empty()) {
                    pthread_mutex_lock(&max_result_mutex);
                    has_max_result = false;
                    pthread_mutex_unlock(&max_result_mutex);
                } else {
                    DetectResult current_result = detector.Max_score(results);

                    pthread_mutex_lock(&max_result_mutex);
                    max_result = current_result;
                    has_max_result = true;
                    pthread_mutex_unlock(&max_result_mutex);

                    APP_LOG_INFO("score=%.3f", current_result.score);

                    cv::rectangle(frame,
                    cv::Rect(current_result.center_x - current_result.width / 2,
                            current_result.center_y - current_result.height / 2,
                            current_result.width,
                            current_result.height),
                    cv::Scalar(0, 255, 0), 4);
                }
            }

            APP_LOG_INFO("推理耗时：%.3fms",
                         timespec_diff_ms(start_time, end1_time));
            if (detect_time_valid) {
                APP_LOG_INFO("检测耗时：%.3fms",
                             timespec_diff_ms(end1_time, end2_time));
            }
        }
#endif

        int64_t now_ms = monotonic_time_ms();
        if (last_detect_state_print_ms <= 0 ||
            now_ms - last_detect_state_print_ms >=
                MAIN1_STATE_PRINT_PERIOD_MS) {
            DetectResult heartbeat_result;
            bool heartbeat_detected;

            pthread_mutex_lock(&max_result_mutex);
            heartbeat_result = max_result;
            heartbeat_detected = has_max_result;
            pthread_mutex_unlock(&max_result_mutex);

            float center_x = CAMERA_WIDTH / 2.0f + (float)x_pixel_offset;
            float center_y = CAMERA_HEIGHT / 2.0f + (float)y_pixel_offset;
            float dx = heartbeat_detected ?
                       heartbeat_result.center_x - center_x : 0.0f;
            float dy = heartbeat_detected ?
                       heartbeat_result.center_y - center_y : 0.0f;

            APP_LOG_INFO("main1 detect heartbeat: detected=%u, dx=%.1f, "
                         "dy=%.1f",
                         heartbeat_detected ? 1U : 0U,
                         dx,
                         dy);
            last_detect_state_print_ms = now_ms;
        }

        (void)icst7735_show_camera_bgr888(&lcd, frame.data, (uint16_t)frame.cols, (uint16_t)frame.rows, (uint32_t)frame.step);

    }
    pthread_join(transmit_thread, NULL);
    icst7735_deinit(&lcd);
    return 0;
}
