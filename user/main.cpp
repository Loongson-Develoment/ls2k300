#include <opencv2/opencv.hpp>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <vector>

#include "LS2K0300_DRV_INC.h"

#define APP_USE_NATIVE_V4L2  (1)
#define APP_USE_YUYV         (0)
#define APP_DECODE_MJPEG     (0)
#define APP_LCD_ENABLE       (0)

#define APP_LCD_WIDTH      ICST7735_CFG_WIDTH
#define APP_LCD_HEIGHT     ICST7735_CFG_HEIGHT
#define APP_LCD_X_OFFSET   ICST7735_CFG_X_OFFSET
#define APP_LCD_Y_OFFSET   ICST7735_CFG_Y_OFFSET
#define APP_LCD_DC_PIN     PIN_50
#define APP_LCD_RST_PIN    PIN_51
#define APP_LCD_BL_PIN     PIN_74
#define APP_LCD_SPI_PORT   LS_SPI2
#define APP_LCD_SPI_HZ     (80000000U)

#define APP_CAM_ID         (0)
#define APP_CAM_DEVICE     "/dev/video0"
#define APP_CAM_WIDTH      (APP_LCD_WIDTH)
#define APP_CAM_HEIGHT     (APP_LCD_HEIGHT)
#define APP_CAM_FPS        ((APP_USE_YUYV != 0) ? 30 : 120)
#define APP_CAM_BUFFERS    (4U)

static inline uint8_t clamp_u8(int v)
{
    if (v < 0) {
        return 0U;
    }
    if (v > 255) {
        return 255U;
    }
    return static_cast<uint8_t>(v);
}

static inline uint16_t color565_from_yuv(int y, int u, int v,
                                         const int y_lut[256],
                                         const int u_b_lut[256],
                                         const int u_g_lut[256],
                                         const int v_r_lut[256],
                                         const int v_g_lut[256])
{
    const int yy = y_lut[y];
    const uint8_t r = clamp_u8((yy + v_r_lut[v] + 128) >> 8);
    const uint8_t g = clamp_u8((yy + u_g_lut[u] + v_g_lut[v] + 128) >> 8);
    const uint8_t b = clamp_u8((yy + u_b_lut[u] + 128) >> 8);

    return static_cast<uint16_t>(((b & 0xF8U) << 8) |
                                 ((g & 0xFCU) << 3) |
                                 ((r & 0xF8U) >> 3));
}

static void init_luts(uint16_t lut_b[256],
                      uint16_t lut_g[256],
                      uint16_t lut_r[256],
                      int y_lut[256],
                      int u_b_lut[256],
                      int u_g_lut[256],
                      int v_r_lut[256],
                      int v_g_lut[256])
{
    for (int i = 0; i < 256; ++i) {
        lut_b[i] = static_cast<uint16_t>((i & 0xF8) << 8);
        lut_g[i] = static_cast<uint16_t>((i & 0xFC) << 3);
        lut_r[i] = static_cast<uint16_t>((i & 0xF8) >> 3);

        y_lut[i] = 298 * (i - 16);
        if (y_lut[i] < 0) {
            y_lut[i] = 0;
        }
        u_b_lut[i] = 516 * (i - 128);
        u_g_lut[i] = -100 * (i - 128);
        v_r_lut[i] = 409 * (i - 128);
        v_g_lut[i] = -208 * (i - 128);
    }
}

static void build_maps(std::vector<int>& x_map,
                       std::vector<int>& y_map,
                       int src_w,
                       int src_h)
{
    for (int x = 0; x < APP_LCD_WIDTH; ++x) {
        x_map[x] = x * src_w / APP_LCD_WIDTH;
    }
    for (int y = 0; y < APP_LCD_HEIGHT; ++y) {
        y_map[y] = y * src_h / APP_LCD_HEIGHT;
    }
}

static void pack_bgr888_to_lcd(const cv::Mat& frame,
                               std::vector<uint8_t>& lcd_buf,
                               std::vector<int>& x_map,
                               std::vector<int>& y_map,
                               int& mapped_src_w,
                               int& mapped_src_h,
                               const uint16_t lut_b[256],
                               const uint16_t lut_g[256],
                               const uint16_t lut_r[256])
{
    const int src_w = frame.cols;
    const int src_h = frame.rows;

    if (src_w == APP_LCD_WIDTH && src_h == APP_LCD_HEIGHT) {
        uint8_t* dst = lcd_buf.data();

        for (int y = 0; y < APP_LCD_HEIGHT; ++y) {
            const uint8_t* src = frame.ptr<uint8_t>(y);

            for (int x = 0; x < APP_LCD_WIDTH; ++x) {
                const uint16_t c = static_cast<uint16_t>(lut_b[src[0]] | lut_g[src[1]] | lut_r[src[2]]);
                dst[0] = static_cast<uint8_t>(c >> 8);
                dst[1] = static_cast<uint8_t>(c & 0xFFU);
                src += 3;
                dst += 2;
            }
        }
        return;
    }

    if (mapped_src_w != src_w || mapped_src_h != src_h) {
        build_maps(x_map, y_map, src_w, src_h);
        mapped_src_w = src_w;
        mapped_src_h = src_h;
    }

    for (int y = 0; y < APP_LCD_HEIGHT; ++y) {
        const uint8_t* src_row = frame.ptr<uint8_t>(y_map[y]);
        uint8_t* dst = lcd_buf.data() + (size_t)y * APP_LCD_WIDTH * 2U;

        for (int x = 0; x < APP_LCD_WIDTH; ++x) {
            const uint8_t* src = src_row + (size_t)x_map[x] * 3U;
            const uint16_t c = static_cast<uint16_t>(lut_b[src[0]] | lut_g[src[1]] | lut_r[src[2]]);
            dst[0] = static_cast<uint8_t>(c >> 8);
            dst[1] = static_cast<uint8_t>(c & 0xFFU);
            dst += 2;
        }
    }
}

static void pack_yuyv_to_lcd(const uint8_t* yuyv,
                             int src_w,
                             int src_h,
                             std::vector<uint8_t>& lcd_buf,
                             std::vector<int>& x_map,
                             std::vector<int>& y_map,
                             int& mapped_src_w,
                             int& mapped_src_h,
                             const int y_lut[256],
                             const int u_b_lut[256],
                             const int u_g_lut[256],
                             const int v_r_lut[256],
                             const int v_g_lut[256])
{
    if (mapped_src_w != src_w || mapped_src_h != src_h) {
        build_maps(x_map, y_map, src_w, src_h);
        mapped_src_w = src_w;
        mapped_src_h = src_h;
    }

    for (int y = 0; y < APP_LCD_HEIGHT; ++y) {
        const uint8_t* src_row = yuyv + (size_t)y_map[y] * src_w * 2U;
        uint8_t* dst = lcd_buf.data() + (size_t)y * APP_LCD_WIDTH * 2U;

        for (int x = 0; x < APP_LCD_WIDTH; ++x) {
            const int sx = x_map[x];
            const uint8_t* p = src_row + (size_t)(sx & ~1) * 2U;
            const int yy = ((sx & 1) == 0) ? p[0] : p[2];
            const int uu = p[1];
            const int vv = p[3];
            const uint16_t c = color565_from_yuv(yy, uu, vv,
                                                 y_lut,
                                                 u_b_lut,
                                                 u_g_lut,
                                                 v_r_lut,
                                                 v_g_lut);

            dst[0] = static_cast<uint8_t>(c >> 8);
            dst[1] = static_cast<uint8_t>(c & 0xFFU);
            dst += 2;
        }
    }
}

int main(void)
{
    icst7735_t lcd;
    ls2k0300_v4l2_t cam;
    cv::Mat frame;

    static std::vector<uint8_t> lcd_buf(APP_LCD_WIDTH * APP_LCD_HEIGHT * 2U);
    static uint16_t lut_b[256];
    static uint16_t lut_g[256];
    static uint16_t lut_r[256];
    static int y_lut[256];
    static int u_b_lut[256];
    static int u_g_lut[256];
    static int v_r_lut[256];
    static int v_g_lut[256];

    std::vector<int> x_map(APP_LCD_WIDTH);
    std::vector<int> y_map(APP_LCD_HEIGHT);
    int mapped_src_w = 0;
    int mapped_src_h = 0;

    int stat_cnt = 0;
    double sum_grab = 0.0;
    double sum_dec  = 0.0;
    double sum_pack = 0.0;
    double sum_lcd  = 0.0;
    double sum_all  = 0.0;
    double sum_cam_ts = 0.0;
    uint32_t prev_sequence = 0U;
    struct timeval prev_timestamp;
    int have_prev_timestamp = 0;
    uint32_t sum_seq_delta = 0U;
    uint32_t seq_stat_cnt = 0U;

    std::memset(&lcd, 0, sizeof(lcd));
    std::memset(&cam, 0, sizeof(cam));

    init_luts(lut_b, lut_g, lut_r, y_lut, u_b_lut, u_g_lut, v_r_lut, v_g_lut);

    const uint32_t request_fourcc = (APP_USE_YUYV != 0) ? V4L2_PIX_FMT_YUYV : V4L2_PIX_FMT_MJPEG;
    if (ls2k0300_v4l2_init(&cam,
                           APP_CAM_DEVICE,
                           APP_CAM_WIDTH,
                           APP_CAM_HEIGHT,
                           request_fourcc,
                           APP_CAM_FPS,
                           APP_CAM_BUFFERS) != 0) {
        std::printf("[FAIL] open %s failed\n", APP_CAM_DEVICE);
        return 1;
    }

    char fcc[5];
    ls2k0300_v4l2_fourcc_to_string(cam.pixelformat, fcc);
    const double actual_fps = (cam.fps_num != 0U) ? ((double)cam.fps_den / (double)cam.fps_num) : 0.0;

    std::printf("[INFO] cam backend : native v4l2 mmap\n");
    std::printf("[INFO] cam request : %d x %d @ %d\n", APP_CAM_WIDTH, APP_CAM_HEIGHT, APP_CAM_FPS);
    std::printf("[INFO] cam actual  : %u x %u @ %.2f\n", cam.width, cam.height, actual_fps);
    std::printf("[INFO] cam fourcc  : %s\n", fcc);

    if (icst7735_init(&lcd,
                      APP_LCD_SPI_PORT,
                      APP_LCD_SPI_HZ,
                      APP_LCD_DC_PIN,
                      APP_LCD_RST_PIN,
                      APP_LCD_BL_PIN,
                      APP_LCD_WIDTH,
                      APP_LCD_HEIGHT,
                      APP_LCD_X_OFFSET,
                      APP_LCD_Y_OFFSET) != 0) {
        std::printf("[FAIL] icst7735 init failed\n");
        ls2k0300_v4l2_deinit(&cam);
        return 1;
    }

    std::printf("[INFO] camera -> ST7735 running\n");
    std::printf("[INFO] lcd spi actual: %u Hz\n", ls2k0300_spi_get_speed(&lcd.spi));

    while (1) {
        ls2k0300_v4l2_frame_t raw;
        int queued = 0;
        int have_pixels = 0;
        int raw_yuyv = 0;
        const uint8_t* yuyv_data = NULL;
        int yuyv_w = 0;
        int yuyv_h = 0;

        auto t0 = std::chrono::steady_clock::now();

        const int dq_ret = ls2k0300_v4l2_dequeue(&cam, &raw, 2000);
        if (dq_ret == 1) {
            std::printf("[WARN] v4l2 dequeue timeout\n");
            continue;
        }
        if (dq_ret != 0) {
            std::printf("[WARN] v4l2 dequeue failed\n");
            continue;
        }
        auto t1 = std::chrono::steady_clock::now();

        if (have_prev_timestamp) {
            const double cam_ts_ms =
                (double)(raw.timestamp.tv_sec - prev_timestamp.tv_sec) * 1000.0 +
                (double)(raw.timestamp.tv_usec - prev_timestamp.tv_usec) / 1000.0;
            sum_cam_ts += cam_ts_ms;
            sum_seq_delta += raw.sequence - prev_sequence;
            seq_stat_cnt++;
        }
        prev_timestamp = raw.timestamp;
        prev_sequence = raw.sequence;
        have_prev_timestamp = 1;

        if (cam.pixelformat == V4L2_PIX_FMT_MJPEG || cam.pixelformat == V4L2_PIX_FMT_JPEG) {
#if (APP_DECODE_MJPEG != 0)
            cv::Mat jpeg_view(1, static_cast<int>(raw.bytesused), CV_8UC1, const_cast<uint8_t*>(raw.data));
            frame = cv::imdecode(jpeg_view, cv::IMREAD_COLOR);
            if (ls2k0300_v4l2_enqueue(&cam, &raw) != 0) {
                std::printf("[WARN] v4l2 enqueue failed\n");
                continue;
            }
            queued = 1;

            if (frame.empty() || frame.type() != CV_8UC3) {
                std::printf("[WARN] mjpeg decode failed bytes=%zu type=%d\n", raw.bytesused, frame.type());
                continue;
            }
            have_pixels = 1;
#else
            if (ls2k0300_v4l2_enqueue(&cam, &raw) != 0) {
                std::printf("[WARN] v4l2 enqueue failed\n");
                continue;
            }
            queued = 1;
#endif
        } else if (cam.pixelformat == V4L2_PIX_FMT_YUYV) {
            raw_yuyv = 1;
            yuyv_data = raw.data;
            yuyv_w = static_cast<int>(cam.width);
            yuyv_h = static_cast<int>(cam.height);
            have_pixels = 1;
        } else {
            std::printf("[WARN] unsupported v4l2 fourcc=%s bytes=%zu\n", fcc, raw.bytesused);
            (void)ls2k0300_v4l2_enqueue(&cam, &raw);
            continue;
        }
        auto t2 = std::chrono::steady_clock::now();

        if (have_pixels) {
            if (raw_yuyv) {
                pack_yuyv_to_lcd(yuyv_data,
                                 yuyv_w,
                                 yuyv_h,
                                 lcd_buf,
                                 x_map,
                                 y_map,
                                 mapped_src_w,
                                 mapped_src_h,
                                 y_lut,
                                 u_b_lut,
                                 u_g_lut,
                                 v_r_lut,
                                 v_g_lut);
            } else {
                pack_bgr888_to_lcd(frame,
                                   lcd_buf,
                                   x_map,
                                   y_map,
                                   mapped_src_w,
                                   mapped_src_h,
                                   lut_b,
                                   lut_g,
                                   lut_r);
            }
        }
        auto t3 = std::chrono::steady_clock::now();

        if (!queued) {
            if (ls2k0300_v4l2_enqueue(&cam, &raw) != 0) {
                std::printf("[WARN] v4l2 enqueue failed\n");
                continue;
            }
            queued = 1;
        }

#if (APP_LCD_ENABLE != 0)
        if (have_pixels) {
            if (icst7735_draw_rgb565_bytes(&lcd,
                                           0,
                                           0,
                                           APP_LCD_WIDTH,
                                           APP_LCD_HEIGHT,
                                           lcd_buf.data()) != 0) {
                std::printf("[WARN] show frame failed\n");
                continue;
            }
        }
#endif
        auto t4 = std::chrono::steady_clock::now();

        double grab_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double dec_ms  = std::chrono::duration<double, std::milli>(t2 - t1).count();
        double pack_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();
        double lcd_ms  = std::chrono::duration<double, std::milli>(t4 - t3).count();
        double all_ms  = std::chrono::duration<double, std::milli>(t4 - t0).count();

        sum_grab += grab_ms;
        sum_dec  += dec_ms;
        sum_pack += pack_ms;
        sum_lcd  += lcd_ms;
        sum_all  += all_ms;

        if (++stat_cnt >= 30) {
            double avg_grab = sum_grab / stat_cnt;
            double avg_dec  = sum_dec  / stat_cnt;
            double avg_pack = sum_pack / stat_cnt;
            double avg_lcd  = sum_lcd  / stat_cnt;
            double avg_all  = sum_all  / stat_cnt;
            double avg_cam_ts = (seq_stat_cnt > 0U) ? (sum_cam_ts / seq_stat_cnt) : 0.0;
            double avg_seq_delta = (seq_stat_cnt > 0U) ? ((double)sum_seq_delta / seq_stat_cnt) : 0.0;

            std::printf("[INFO] grab=%.2fms dec=%.2fms pack=%.2fms lcd=%.2fms total=%.2fms fps=%.2f cam_ts=%.2fms seq_delta=%.2f\n",
                        avg_grab,
                        avg_dec,
                        avg_pack,
                        avg_lcd,
                        avg_all,
                        1000.0 / avg_all,
                        avg_cam_ts,
                        avg_seq_delta);

            stat_cnt = 0;
            sum_grab = 0.0;
            sum_dec  = 0.0;
            sum_pack = 0.0;
            sum_lcd  = 0.0;
            sum_all  = 0.0;
            sum_cam_ts = 0.0;
            sum_seq_delta = 0U;
            seq_stat_cnt = 0U;
        }
    }

    icst7735_deinit(&lcd);
    ls2k0300_v4l2_deinit(&cam);
    return 0;
}
