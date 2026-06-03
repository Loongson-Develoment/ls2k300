#include "opencv2/opencv.hpp"
#include "LS2K0300_DRV_INC.h"

#include <unistd.h>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/select.h>

#ifndef RECORD_WITH_TARGET
#define RECORD_WITH_TARGET 1
#endif

#define LCD_DC_PIN   PIN_50
#define LCD_RST_PIN  PIN_51
#define LCD_BL_PIN   PIN_74

#define CAMERA_WIDTH    640
#define CAMERA_HEIGHT   480
#define CAMERA_FPS      30
#define CAMERA_BUFFERS  2

#define SAVE_DIR            "/home/root/video"
#define COUNTDOWN_SECONDS   3
#define RECORD_SECONDS      30

#if RECORD_WITH_TARGET
static const char* kVideoPrefix = "with_target";
static const int kRecordLoops = 15;
#else
static const char* kVideoPrefix = "without_target";
static const int kRecordLoops = 8;
#endif

static volatile std::sig_atomic_t g_running = 1;

static void signal_handler(int signum)
{
    (void)signum;
    g_running = 0;
}

static bool ensure_save_dir()
{
    const std::string cmd = std::string("mkdir -p ") + SAVE_DIR;
    return std::system(cmd.c_str()) == 0;
}

static std::string make_video_path(int clip_index)
{
    char path_buf[256];
    std::snprintf(path_buf, sizeof(path_buf), "%s/%s_%03d.avi", SAVE_DIR, kVideoPrefix, clip_index + 1);
    return std::string(path_buf);
}

static void remove_partial_video(const std::string& path)
{
    if (path.empty()) {
        return;
    }

    if (std::remove(path.c_str()) == 0) {
        std::cout << "[INFO] removed partial video: " << path << std::endl;
    }
}

static void draw_overlay(cv::Mat* image, const std::string& line1, const std::string& line2)
{
    if (image == 0 || image->empty()) {
        return;
    }

    cv::rectangle(*image, cv::Rect(0, 0, image->cols, 54), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(*image, line1, cv::Point(8, 20), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::putText(*image, line2, cv::Point(8, 44), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
}

static void show_frame(icst7735_t* lcd, const cv::Mat& bgr_frame, const std::string& line1, const std::string& line2)
{
    if (lcd == 0 || bgr_frame.empty()) {
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

static bool read_terminal_command_nonblocking(std::string* cmd)
{
    if (cmd == 0) {
        return false;
    }

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    const int ready = select(STDIN_FILENO + 1, &readfds, 0, 0, &timeout);
    if (ready > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
        return (bool)std::getline(std::cin, *cmd);
    }

    return false;
}

static bool wait_for_start_command(cv::VideoCapture* cap, icst7735_t* lcd, int clip_index)
{
    if (cap == 0 || lcd == 0) {
        return false;
    }

    std::cout << "[INFO] clip " << (clip_index + 1)
              << ": input 1 then Enter to start recording, input 0 to stop" << std::endl;

    while (g_running) {
        cv::Mat frame;
        if (!cap->read(frame)) {
            std::cerr << "[ERROR] could not read frame while waiting for start command" << std::endl;
            g_running = 0;
            return false;
        }

        cv::Mat bgr_frame;
        cv::cvtColor(frame, bgr_frame, cv::COLOR_YUV2BGR_YUYV);

        std::ostringstream line1;
        std::ostringstream line2;
        line1 << kVideoPrefix << " " << (clip_index + 1) << "/" << kRecordLoops;
        line2 << "1 start 0 stop";
        show_frame(lcd, bgr_frame, line1.str(), line2.str());

        std::string cmd;
        if (read_terminal_command_nonblocking(&cmd)) {
            if (cmd == "1") {
                return true;
            }

            if (cmd == "0" || cmd == "q" || cmd == "quit") {
                g_running = 0;
                return false;
            }

            std::cout << "[INFO] ignore command: " << cmd
                      << " (use 1 to start, 0 to stop)" << std::endl;
        }
    }

    return false;
}

int main()
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    if (!ensure_save_dir()) {
        std::cerr << "[ERROR] could not create save dir: " << SAVE_DIR << std::endl;
        return 1;
    }

    cv::VideoCapture cap("/dev/video0", cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "[ERROR] could not open camera" << std::endl;
        return 1;
    }

    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    cap.set(cv::CAP_PROP_CONVERT_RGB, 0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
    cap.set(cv::CAP_PROP_FPS, CAMERA_FPS);
    cap.set(cv::CAP_PROP_BUFFERSIZE, CAMERA_BUFFERS);

    icst7735_t lcd;
    std::memset(&lcd, 0, sizeof(lcd));
    if (icst7735_init(&lcd, LS_SPI2, 40000000U, LCD_DC_PIN, LCD_RST_PIN, LCD_BL_PIN, 160, 128, 0, 0) != 0) {
        std::cerr << "[ERROR] could not init lcd" << std::endl;
        return 1;
    }

    std::cout << "[INFO] save dir: " << SAVE_DIR << std::endl;
    std::cout << "[INFO] prefix: " << kVideoPrefix << std::endl;
    std::cout << "[INFO] clips : " << kRecordLoops << std::endl;
    std::cout << "[INFO] seconds per clip: " << RECORD_SECONDS << std::endl;

    for (int clip_index = 0; clip_index < kRecordLoops && g_running; ++clip_index) {
        const std::string output_path = make_video_path(clip_index);

        std::cout << "[INFO] ready clip " << (clip_index + 1) << "/" << kRecordLoops
                  << " -> " << output_path << std::endl;

        if (!wait_for_start_command(&cap, &lcd, clip_index)) {
            break;
        }

        for (int countdown = COUNTDOWN_SECONDS; countdown > 0 && g_running; --countdown) {
            const auto countdown_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
            while (g_running && std::chrono::steady_clock::now() < countdown_deadline) {
                cv::Mat frame;
                if (!cap.read(frame)) {
                    std::cerr << "[ERROR] could not read frame during countdown" << std::endl;
                    g_running = 0;
                    break;
                }

                cv::Mat bgr_frame;
                cv::cvtColor(frame, bgr_frame, cv::COLOR_YUV2BGR_YUYV);

                std::ostringstream line1;
                std::ostringstream line2;
                line1 << kVideoPrefix << " " << (clip_index + 1) << "/" << kRecordLoops;
                line2 << "start in " << countdown << " s";
                show_frame(&lcd, bgr_frame, line1.str(), line2.str());

                std::string cmd;
                if (read_terminal_command_nonblocking(&cmd) && (cmd == "0" || cmd == "q" || cmd == "quit")) {
                    g_running = 0;
                    break;
                }
            }
        }

        if (!g_running) {
            break;
        }

        cv::VideoWriter writer(output_path,
                               cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                               CAMERA_FPS,
                               cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT));

        if (!writer.isOpened()) {
            std::cerr << "[ERROR] could not open writer: " << output_path << std::endl;
            break;
        }

        const auto record_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(RECORD_SECONDS);
        bool clip_completed = true;
        while (g_running && std::chrono::steady_clock::now() < record_deadline) {
            cv::Mat frame;
            if (!cap.read(frame)) {
                std::cerr << "[ERROR] could not read frame during recording" << std::endl;
                g_running = 0;
                clip_completed = false;
                break;
            }

            cv::Mat bgr_frame;
            cv::cvtColor(frame, bgr_frame, cv::COLOR_YUV2BGR_YUYV);

            writer.write(bgr_frame);

            const auto now = std::chrono::steady_clock::now();
            const auto remaining = std::chrono::duration_cast<std::chrono::seconds>(record_deadline - now).count();

            std::ostringstream line1;
            std::ostringstream line2;
            line1 << "REC " << (clip_index + 1) << "/" << kRecordLoops;
            line2 << kVideoPrefix << " " << (remaining > 0 ? remaining : 0) << " s 0 stop";
            show_frame(&lcd, bgr_frame, line1.str(), line2.str());

            std::string cmd;
            if (read_terminal_command_nonblocking(&cmd) && (cmd == "0" || cmd == "q" || cmd == "quit")) {
                g_running = 0;
                clip_completed = false;
                break;
            }
        }

        writer.release();
        if (!g_running && clip_completed) {
            clip_completed = false;
        }

        if (clip_completed) {
            std::cout << "[INFO] saved: " << output_path << std::endl;
        } else {
            remove_partial_video(output_path);
        }
    }

    icst7735_deinit(&lcd);
    return 0;
}
