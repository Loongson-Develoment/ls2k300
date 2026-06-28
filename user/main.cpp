#include "My_inc.h"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <time.h>
#include <vector>
#include <pthread.h>


static volatile std::sig_atomic_t running = 1;
static const struct timespec period = {0, 330 * 1000 * 1000}; // 330ms
static DetectResult max_result;
static bool has_max_result = false;
static pthread_mutex_t max_result_mutex = PTHREAD_MUTEX_INITIALIZER;

void signal_handler(int signum) {
    (void)signum;
    running = 0;
}


void *transmit_function(void *arg) {
    (void)arg;
    DetectResult transmit_result;
    ls2k0300_uart_t uart1;
    memset(&uart1, 0, sizeof(uart1));
    if (ls2k0300_uart_init(&uart1, UART1, B115200, LS_UART_STOP1, LS_UART_DATA8, LS_UART_PARITY_NONE, LS_UART_MODE_BLOCKING) != 0) {
        std::cerr << "Error: Could not init uart" << std::endl;
        return NULL;
    }

    Pid pid_x(0.5f, 0.0f, 0.0f,100);
    Pid pid_y(0.5f, 0.0f, 0.0f,100);
    // Pid pid_z(0.0f, 0.0f, 0.0f);
    
    while (running) 
    {
        bool local_has_result = false;

        pthread_mutex_lock(&max_result_mutex);
        if (has_max_result) {
            transmit_result = max_result;
            local_has_result = true;
        }
        pthread_mutex_unlock(&max_result_mutex);

        if (local_has_result)
        {
            float speed_x = pid_x.calculate(CAMERA_WIDTH / 2, transmit_result.center_x);
            float speed_y = pid_y.calculate(CAMERA_HEIGHT / 2, transmit_result.center_y);
            // float speed_z = pid_z.calculate(0.0f, transmit_result->score);
            char sendbuffer[64];
            int send_len = snprintf(sendbuffer, sizeof(sendbuffer), "X:%.2f,Y:%.2f\n", speed_x, speed_y);
            if (send_len > 0) {
                ls2k0300_uart_write(&uart1, (const uint8_t*)sendbuffer, (size_t)send_len);
            }

        }
        nanosleep(&period, NULL);
    }
    ls2k0300_uart_deinit(&uart1);
    return NULL;
}

int main()
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    NcnnDetector detector;

    Camera camera;
    if (!camera.Open()) {
        return 1;
    }

    icst7735_t lcd;
    memset(&lcd, 0, sizeof(lcd));
    if (icst7735_init(&lcd, LS_SPI2, 80000000U, LCD_DC_PIN, CD_RST_PIN, LCD_BL_PIN, 160, 128, 0, 0) != 0) {
        std::cerr << "Error: Could not init lcd" << std::endl;
        return 1;
    }

    if (!detector.IsLoaded()) 
    {
        std::cerr << "Error: Could not load ncnn model: " << detector.LastError() << std::endl;
        icst7735_deinit(&lcd);
        return 1;
    }

    pthread_t transmit_thread;
    if (pthread_create(&transmit_thread, NULL, transmit_function, NULL) != 0) {
        std::cerr << "Error: Could not create transmit thread" << std::endl;
        icst7735_deinit(&lcd);
        return 1;
    }

    while (running) {
        cv::Mat frame;
        if (!camera.ReadFrame(frame)) {
            continue;
        }

        if (detector.IsLoaded()) {
            ncnn::Mat out = detector.Reasoning(frame);

            std::vector<DetectResult> results;
            if (!detector.Detect(out, frame.cols, frame.rows, results)) {
                std::cerr << "[WARN] ncnn detect failed: "
                          << detector.LastError()
                          << std::endl;
            } else {
                std::cout << "[INFO] detections: " << results.size() << std::endl;

                if (results.empty()) {
                    pthread_mutex_lock(&max_result_mutex);
                    has_max_result = false;
                    pthread_mutex_unlock(&max_result_mutex);
                    (void)icst7735_show_camera_bgr888(&lcd, frame.data, (uint16_t)frame.cols, (uint16_t)frame.rows, (uint32_t)frame.step);
                    continue;
                }

                DetectResult current_result = detector.Max_score(results);

                pthread_mutex_lock(&max_result_mutex);
                max_result = current_result;
                has_max_result = true;
                pthread_mutex_unlock(&max_result_mutex);

                std::cout << " score=" << current_result.score << std::endl;

                cv::rectangle(frame, 
                cv::Rect(current_result.center_x - current_result.width / 2,
                        current_result.center_y - current_result.height / 2,
                        current_result.width,
                        current_result.height),
                cv::Scalar(0, 255, 0), 4);
            }
        }

        (void)icst7735_show_camera_bgr888(&lcd, frame.data, (uint16_t)frame.cols, (uint16_t)frame.rows, (uint32_t)frame.step);

    }
    pthread_join(transmit_thread, NULL);
    icst7735_deinit(&lcd);
    return 0;
}
