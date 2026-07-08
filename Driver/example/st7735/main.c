#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "ICST7735.h"

/********************************************************************************
 * @brief   示例显示分辨率.
 ********************************************************************************/
#define EXAMPLE_LCD_WIDTH   ICST7735_WIDTH_DEFAULT
#define EXAMPLE_LCD_HEIGHT  ICST7735_HEIGHT_DEFAULT

/********************************************************************************
 * @brief   屏幕控制引脚定义.
 * @note    SPI2 的 SCLK/MOSI/MISO/CS 由 LS2K0300_SPI 驱动按端口固定配置：
 *          SCLK=PIN_64, MISO=PIN_65, MOSI=PIN_66, CS=PIN_67(内部自动控制).
 *          这里仅额外指定 DC/RST/BL 三个 GPIO.
 ********************************************************************************/
#define EXAMPLE_LCD_RST_PIN  PIN_51
#define EXAMPLE_LCD_DC_PIN   PIN_50
#define EXAMPLE_LCD_BL_PIN   PIN_74

// #define EXAMPLE_LCD_RST_PIN  PIN_50
// #define EXAMPLE_LCD_DC_PIN   PIN_74
// #define EXAMPLE_LCD_BL_PIN   PIN_51

/********************************************************************************
 * @brief   SPI 端口与时钟配置.
 ********************************************************************************/
#define EXAMPLE_SPI_PORT      LS_SPI2
#define EXAMPLE_SPI_SPEED_HZ  (80000000U)

/********************************************************************************
 * @brief   测试图像缓冲区（BGR888）.
 ********************************************************************************/
static uint8_t g_cam_bgr888[EXAMPLE_LCD_WIDTH * EXAMPLE_LCD_HEIGHT * 3U];

/********************************************************************************
 * @brief   生成一帧用于演示的伪摄像头 BGR888 图像.
 * @param   buf    : 输出图像缓冲区.
 * @param   width  : 图像宽度.
 * @param   height : 图像高度.
 * @return  none.
 * @example build_test_camera_frame(g_cam_bgr888, 128, 160);
 ********************************************************************************/
static void build_test_camera_frame(uint8_t *buf, uint16_t width, uint16_t height)
{
    uint32_t x;
    uint32_t y;

    if (buf == NULL || width == 0U || height == 0U) {
        return;
    }

    for (y = 0U; y < (uint32_t)height; y++) {
        for (x = 0U; x < (uint32_t)width; x++) {
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
 * @brief   ST7735 示例程序入口.
 * @return  成功返回 0，失败返回 1.
 * @note    流程：初始化 -> 红绿蓝纯色测试 -> 彩色图显示 -> 灰度图显示 -> 反初始化.
 ********************************************************************************/
int main(void)
{
    icst7735_t lcd;

    memset(&lcd, 0, sizeof(lcd));

    if (icst7735_init(&lcd,
                      EXAMPLE_SPI_PORT,
                      EXAMPLE_SPI_SPEED_HZ,
                      EXAMPLE_LCD_DC_PIN,
                      EXAMPLE_LCD_RST_PIN,
                      EXAMPLE_LCD_BL_PIN,
                      EXAMPLE_LCD_WIDTH,
                      EXAMPLE_LCD_HEIGHT,
                      0U,
                      0U) != 0) {
        printf("[FAIL] icst7735 init failed\n");
        return 1;
    }

    printf("[INFO] icst7735 init ok\n");

    (void)icst7735_fill_color(&lcd, ICST7735_COLOR565(255U, 0U, 0U));
    sleep(1);
    (void)icst7735_fill_color(&lcd, ICST7735_COLOR565(0U, 255U, 0U));
    sleep(1);
    (void)icst7735_fill_color(&lcd, ICST7735_COLOR565(0U, 0U, 255U));
    sleep(1);

    build_test_camera_frame(g_cam_bgr888, EXAMPLE_LCD_WIDTH, EXAMPLE_LCD_HEIGHT);

    if (icst7735_show_camera_bgr888(&lcd,
                                    g_cam_bgr888,
                                    EXAMPLE_LCD_WIDTH,
                                    EXAMPLE_LCD_HEIGHT,
                                    EXAMPLE_LCD_WIDTH * 3U) != 0) {
        printf("[FAIL] show color camera frame failed\n");
        icst7735_deinit(&lcd);
        return 1;
    }
    printf("[INFO] color frame displayed\n");
    sleep(2);

    if (icst7735_show_camera_gray_bgr888(&lcd,
                                         g_cam_bgr888,
                                         EXAMPLE_LCD_WIDTH,
                                         EXAMPLE_LCD_HEIGHT,
                                         EXAMPLE_LCD_WIDTH * 3U) != 0) {
        printf("[FAIL] show gray camera frame failed\n");
        icst7735_deinit(&lcd);
        return 1;
    }
    printf("[INFO] gray frame displayed\n");
    sleep(2);

    icst7735_deinit(&lcd);
    printf("[PASS] example_st7735 done\n");
    return 0;
}
