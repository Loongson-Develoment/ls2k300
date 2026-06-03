#include "opencv2/opencv.hpp"
#include "LS2K0300_DRV_INC.h"
#include "NcnnDetector.h"

#include <unistd.h>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <time.h>
#include <vector>

#define LCD_DC_PIN   PIN_74
#define CD_RST_PIN  PIN_50
#define LCD_BL_PIN   PIN_75

#define CAMERA_WIDTH   640
#define CAMERA_HEIGHT  480
#define CAMERA_FPS     120
#define CAMERA_BUFFERS 1
#define SHOW_YUYV_GRAY 1

#define NCNN_MODEL_PARAM_PATH "./model.ncnn.param"
#define NCNN_MODEL_BIN_PATH   "./model.ncnn.bin"

static volatile std::sig_atomic_t running = 1;

void signal_handler(int signum) {
    (void)signum;
    running = 0;
}

static bool file_exists(const char *path)
{
    return path != 0 && access(path, F_OK) == 0;
}

static double duration_ms(const std::chrono::steady_clock::time_point& start,
                          const std::chrono::steady_clock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

int main()
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    NcnnDetector detector;
    if (file_exists(NCNN_MODEL_PARAM_PATH) && file_exists(NCNN_MODEL_BIN_PATH)) {
        if (detector.Load(NCNN_MODEL_PARAM_PATH, NCNN_MODEL_BIN_PATH)) {
            std::cout << "[INFO] ncnn model loaded: "
                      << NCNN_MODEL_PARAM_PATH
                      << " / "
                      << NCNN_MODEL_BIN_PATH
                      << std::endl;
        } else {
            std::cerr << "[WARN] ncnn model load failed: "
                      << detector.LastError()
                      << std::endl;
        }
    } else {
        std::cout << "[INFO] ncnn model files not found, skip detector init"
                  << std::endl;
    }

    cv::VideoCapture cap("/dev/video0", cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera" << std::endl;
        return -1;
    }
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    cap.set(cv::CAP_PROP_CONVERT_RGB, 0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
    cap.set(cv::CAP_PROP_FPS, CAMERA_FPS);
    cap.set(cv::CAP_PROP_BUFFERSIZE, CAMERA_BUFFERS);
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    cap.set(cv::CAP_PROP_EXPOSURE, 50);

    icst7735_t lcd;
    memset(&lcd, 0, sizeof(lcd));
    icst7735_init(&lcd, LS_SPI2, 40000000U, LCD_DC_PIN, CD_RST_PIN, LCD_BL_PIN, 160, 128, 0, 0);

    while (running) {
        const std::chrono::steady_clock::time_point frame_start = std::chrono::steady_clock::now();
        cv::Mat frame;
        if (!cap.read(frame)) {
            std::cerr << "Error: Could not read frame from camera" << std::endl;
            break;
        }
        const std::chrono::steady_clock::time_point capture_end = std::chrono::steady_clock::now();

        cv::Mat bgr_frame;
        cv::cvtColor(frame, bgr_frame, cv::COLOR_YUV2BGR_YUYV);
        const std::chrono::steady_clock::time_point convert_end = std::chrono::steady_clock::now();

        std::vector<NcnnDetection> detections;
        NcnnDetectTiming detect_timing;
        bool detect_ok = false;

        if (detector.IsLoaded()) {
            detect_ok = detector.Detect(bgr_frame, &detections, &detect_timing);
            if (!detect_ok) {
                std::cerr << "[WARN] ncnn detect failed: "
                          << detector.LastError()
                          << std::endl;
            } else {
                std::cout << "[INFO] detections: " << detections.size() << std::endl;
            }
        }
        const std::chrono::steady_clock::time_point detect_end = std::chrono::steady_clock::now();

        for (size_t i = 0; i < detections.size(); ++i) {
            const cv::Rect& box = detections[i].box;
            const float score = detections[i].score;
            std::cout << "[INFO] detection[" << i << "]"
                      << " score=" << std::fixed << std::setprecision(3) << score
                      << " x=" << box.x
                      << " y=" << box.y
                      << " w=" << box.width
                      << " h=" << box.height
                      << std::endl;
            cv::rectangle(bgr_frame, box, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

            char text[64];
            std::snprintf(text, sizeof(text), "injection %.2f", score);
            int baseline = 0;
            cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.55, 1, &baseline);
            int text_x = box.x;
            int text_y = box.y > text_size.height + 4 ? box.y - 4 : box.y + text_size.height + 4;

            cv::rectangle(
                bgr_frame,
                cv::Rect(text_x, text_y - text_size.height - 2, text_size.width + 4, text_size.height + 6),
                cv::Scalar(0, 255, 0),
                cv::FILLED
            );
            cv::putText(
                bgr_frame,
                text,
                cv::Point(text_x + 2, text_y),
                cv::FONT_HERSHEY_SIMPLEX,
                0.55,
                cv::Scalar(0, 0, 0),
                1,
                cv::LINE_AA
            );
        }
        const std::chrono::steady_clock::time_point draw_end = std::chrono::steady_clock::now();

        (void)icst7735_show_camera_bgr888(&lcd, bgr_frame.data, (uint16_t)bgr_frame.cols, (uint16_t)bgr_frame.rows, (uint32_t)bgr_frame.step);
        const std::chrono::steady_clock::time_point lcd_end = std::chrono::steady_clock::now();

        const double capture_ms = duration_ms(frame_start, capture_end);
        const double convert_ms = duration_ms(capture_end, convert_end);
        const double detect_ms = duration_ms(convert_end, detect_end);
        const double draw_ms = duration_ms(detect_end, draw_end);
        const double lcd_ms = duration_ms(draw_end, lcd_end);
        const double total_ms = duration_ms(frame_start, lcd_end);

        std::cout << "[INFO] timing"
                  << " capture=" << std::fixed << std::setprecision(2) << capture_ms << "ms"
                  << " convert=" << convert_ms << "ms"
                  << " detect=" << detect_ms << "ms";

        if (detector.IsLoaded() && detect_ok) {
            std::cout << " {prep=" << detect_timing.preprocess_ms
                      << "ms infer=" << detect_timing.inference_ms
                      << "ms post=" << detect_timing.postprocess_ms
                      << "ms total=" << detect_timing.total_ms
                      << "ms}";
        }

        std::cout << " draw=" << draw_ms << "ms"
                  << " lcd=" << lcd_ms << "ms"
                  << " frame=" << total_ms << "ms"
                  << std::endl;

    }

    icst7735_deinit(&lcd);
    return 0;
}
