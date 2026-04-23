#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "ICSSD1306.h"

/********************************************************************************
 * @brief   示例屏幕分辨率.
 ********************************************************************************/
#define EXAMPLE_OLED_WIDTH   ICSSD1306_WIDTH_DEFAULT
#define EXAMPLE_OLED_HEIGHT  ICSSD1306_HEIGHT_DEFAULT

/********************************************************************************
 * @brief   示例 I2C 端口配置.
 * @note    以 I2C1-m0 为例，默认引脚为 PIN_50(SCL)、PIN_51(SDA).
 ********************************************************************************/
#define EXAMPLE_I2C_PORT       LS_HW_I2C1
#define EXAMPLE_I2C_MUX        LS_HW_I2C_MUX_M0
#define EXAMPLE_I2C_ADDR       (0x3CU)
#define EXAMPLE_I2C_SPEED_HZ   (400000U)

/********************************************************************************
 * @brief   测试图像缓冲区（BGR888）.
 ********************************************************************************/
static uint8_t g_cam_bgr888[EXAMPLE_OLED_WIDTH * EXAMPLE_OLED_HEIGHT * 3U];

/********************************************************************************
 * @brief   生成一帧用于演示的伪摄像头 BGR888 图像.
 * @param   buf    : 输出图像缓冲区.
 * @param   width  : 图像宽度.
 * @param   height : 图像高度.
 * @return  none.
 ********************************************************************************/
static void build_test_camera_frame(uint8_t *buf, uint16_t width, uint16_t height)
{
    uint32_t x;
    uint32_t y;

    if (buf == NULL || width == 0U || height == 0U) {
        return;
    }

    for (y = 0U; y < (uint32_t)height; ++y) {
        for (x = 0U; x < (uint32_t)width; ++x) {
            uint8_t r = (uint8_t)((x * 255U) / (uint32_t)(width - 1U));
            uint8_t g = (uint8_t)((y * 255U) / (uint32_t)(height - 1U));
            uint8_t b = (uint8_t)(((x + y) * 255U) / (uint32_t)((width - 1U) + (height - 1U)));
            size_t idx = ((size_t)y * (size_t)width + (size_t)x) * 3U;

            buf[idx + 0U] = b;
            buf[idx + 1U] = g;
            buf[idx + 2U] = r;
        }
    }
}

/********************************************************************************
 * @brief   绘制一个 8x8 棋盘格测试图案（仅写缓冲区）.
 * @param   oled : 驱动句柄.
 * @return  none.
 ********************************************************************************/
static void draw_checkerboard(icssd1306_t *oled)
{
    uint16_t x;
    uint16_t y;

    if (oled == NULL) {
        return;
    }

    for (y = 0U; y < EXAMPLE_OLED_HEIGHT; ++y) {
        for (x = 0U; x < EXAMPLE_OLED_WIDTH; ++x) {
            uint8_t block_x = (uint8_t)(x / 8U);
            uint8_t block_y = (uint8_t)(y / 8U);
            icssd1306_color_t color = (((block_x + block_y) & 0x01U) != 0U)
                                       ? ICSSD1306_COLOR_WHITE
                                       : ICSSD1306_COLOR_BLACK;
            (void)icssd1306_draw_pixel(oled, x, y, color);
        }
    }
}

/********************************************************************************
 * @brief   SSD1306 I2C 示例程序入口.
 * @return  成功返回 0，失败返回 1.
 ********************************************************************************/
int main(void)
{
    icssd1306_t oled;

    memset(&oled, 0, sizeof(oled));

    if (icssd1306_init(&oled,
                       EXAMPLE_I2C_PORT,
                       EXAMPLE_I2C_MUX,
                       EXAMPLE_I2C_ADDR,
                       EXAMPLE_I2C_SPEED_HZ,
                       EXAMPLE_OLED_WIDTH,
                       EXAMPLE_OLED_HEIGHT) != 0) {
        printf("[FAIL] icssd1306 init failed\\n");
        return 1;
    }

    printf("[INFO] icssd1306 init ok\\n");

    if (icssd1306_fill_screen(&oled, ICSSD1306_COLOR_BLACK) != 0) {
        printf("[FAIL] clear screen failed\\n");
        icssd1306_deinit(&oled);
        return 1;
    }

    draw_checkerboard(&oled);
    if (icssd1306_update_screen(&oled) != 0) {
        printf("[FAIL] checkerboard draw failed\\n");
        icssd1306_deinit(&oled);
        return 1;
    }
    printf("[INFO] checkerboard displayed\\n");
    sleep(1);

    build_test_camera_frame(g_cam_bgr888, EXAMPLE_OLED_WIDTH, EXAMPLE_OLED_HEIGHT);
    if (icssd1306_show_camera_gray_bgr888(&oled,
                                          g_cam_bgr888,
                                          EXAMPLE_OLED_WIDTH,
                                          EXAMPLE_OLED_HEIGHT,
                                          EXAMPLE_OLED_WIDTH * 3U,
                                          128U) != 0) {
        printf("[FAIL] gray camera frame display failed\\n");
        icssd1306_deinit(&oled);
        return 1;
    }
    printf("[INFO] gray frame displayed\\n");
    sleep(2);

    (void)icssd1306_invert_display(&oled, 1);
    sleep(1);
    (void)icssd1306_invert_display(&oled, 0);

    icssd1306_deinit(&oled);
    printf("[PASS] example_ssd1306 done\\n");
    return 0;
}
