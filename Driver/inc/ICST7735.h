#ifndef __ICST7735_H
#define __ICST7735_H

#include <stddef.h>
#include <stdint.h>

#include "LS2K0300_SPI.h"
#include "LS2K0300_GPIO.h"

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************************
 * @brief   ST7735 默认面板宽度（像素）.
 ********************************************************************************/
#define ICST7735_WIDTH_DEFAULT   (128U)

/********************************************************************************
 * @brief   ST7735 默认面板高度（像素）.
 ********************************************************************************/
#define ICST7735_HEIGHT_DEFAULT  (160U)

/********************************************************************************
 * @brief   将 8bit RGB 分量转换为 RGB565.
 * @param   r : 红色分量（0~255）.
 * @param   g : 绿色分量（0~255）.
 * @param   b : 蓝色分量（0~255）.
 * @return  RGB565 打包颜色值.
 * @example uint16_t c = ICST7735_COLOR565(255, 0, 0);
 ********************************************************************************/
#define ICST7735_COLOR565(r, g, b) \
    (uint16_t)((((uint16_t)((r) & 0xF8U)) << 8) | (((uint16_t)((g) & 0xFCU)) << 3) | ((uint16_t)(b) >> 3))

/********************************************************************************
 * @brief   屏幕旋转模式枚举.
 ********************************************************************************/
typedef enum {
    ICST7735_ROTATION_0 = 0,
    ICST7735_ROTATION_90,
    ICST7735_ROTATION_180,
    ICST7735_ROTATION_270,
    ICST7735_ROTATION_INVALID
} icst7735_rotation_t;

/********************************************************************************
 * @brief   ST7735 驱动句柄结构体.
 * @note    SPI 与 GPIO 句柄会在以下接口中初始化：
 *          icst7735_init().
 ********************************************************************************/
typedef struct {
    ls2k0300_spi_t  spi;
    ls2k0300_gpio_t dc_gpio;
    ls2k0300_gpio_t rst_gpio;
    ls2k0300_gpio_t bl_gpio;

    uint16_t native_width;
    uint16_t native_height;
    uint16_t width;
    uint16_t height;
    uint16_t x_offset;
    uint16_t y_offset;

    icst7735_rotation_t rotation;

    int has_reset_pin;
    int has_backlight_pin;
    int initialized;
} icst7735_t;

/********************************************************************************
 * @brief   初始化 ST7735 驱动与面板.
 * @param   lcd          : 驱动句柄.
 * @param   spi_port     : SPI 控制器端口.
 * @param   spi_speed_hz : SPI 时钟频率（Hz）.
 * @param   dc_pin       : D/C 控制引脚.
 * @param   rst_pin      : 复位引脚，未连接可传 PIN_INVALID.
 * @param   bl_pin       : 背光引脚，未连接可传 PIN_INVALID.
 * @param   width        : 逻辑显示宽度.
 * @param   height       : 逻辑显示高度.
 * @param   x_offset     : 控制器显存 X 起始偏移.
 * @param   y_offset     : 控制器显存 Y 起始偏移.
 * @return  成功返回 0，失败返回 -1.
 * @example icst7735_t lcd;
 *          icst7735_init(&lcd, LS_SPI2, 20000000U, PIN_10, PIN_11, PIN_12, 128, 160, 0, 0);
 ********************************************************************************/
int icst7735_init(icst7735_t *lcd,
                  ls_spi_port_t spi_port,
                  uint32_t spi_speed_hz,
                  gpio_pin_t dc_pin,
                  gpio_pin_t rst_pin,
                  gpio_pin_t bl_pin,
                  uint16_t width,
                  uint16_t height,
                  uint16_t x_offset,
                  uint16_t y_offset);

/********************************************************************************
 * @brief   释放 ST7735 驱动资源.
 * @param   lcd : 驱动句柄.
 * @return  none.
 * @example icst7735_deinit(&lcd);
 ********************************************************************************/
void icst7735_deinit(icst7735_t *lcd);

/********************************************************************************
 * @brief   执行面板硬件复位序列.
 * @param   lcd : 驱动句柄.
 * @return  成功返回 0，失败返回 -1.
 * @example icst7735_hw_reset(&lcd);
 ********************************************************************************/
int icst7735_hw_reset(icst7735_t *lcd);

/********************************************************************************
 * @brief   控制面板背光 GPIO.
 * @param   lcd : 驱动句柄.
 * @param   on  : 0 表示关闭，非 0 表示开启.
 * @return  成功返回 0，失败返回 -1.
 * @example icst7735_set_backlight(&lcd, 1);
 ********************************************************************************/
int icst7735_set_backlight(icst7735_t *lcd, int on);

/********************************************************************************
 * @brief   设置屏幕旋转方向.
 * @param   lcd      : 驱动句柄.
 * @param   rotation : 旋转模式.
 * @return  成功返回 0，失败返回 -1.
 * @example icst7735_set_rotation(&lcd, ICST7735_ROTATION_90);
 ********************************************************************************/
int icst7735_set_rotation(icst7735_t *lcd, icst7735_rotation_t rotation);

/********************************************************************************
 * @brief   设置当前绘制窗口.
 * @param   lcd : 驱动句柄.
 * @param   x0  : 窗口起始 X.
 * @param   y0  : 窗口起始 Y.
 * @param   x1  : 窗口结束 X.
 * @param   y1  : 窗口结束 Y.
 * @return  成功返回 0，失败返回 -1.
 * @example icst7735_set_window(&lcd, 0, 0, 127, 159);
 ********************************************************************************/
int icst7735_set_window(icst7735_t *lcd, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

/********************************************************************************
 * @brief   绘制一个 RGB565 像素点.
 * @param   lcd   : 驱动句柄.
 * @param   x     : X 坐标.
 * @param   y     : Y 坐标.
 * @param   color : RGB565 颜色值.
 * @return  成功返回 0，失败返回 -1.
 * @example icst7735_draw_pixel(&lcd, 10, 20, ICST7735_COLOR565(255, 255, 255));
 ********************************************************************************/
int icst7735_draw_pixel(icst7735_t *lcd, uint16_t x, uint16_t y, uint16_t color);

/********************************************************************************
 * @brief   使用单一 RGB565 颜色值填充全屏.
 * @param   lcd   : 驱动句柄.
 * @param   color : RGB565 颜色值.
 * @return  成功返回 0，失败返回 -1.
 * @example icst7735_fill_color(&lcd, ICST7735_COLOR565(0, 0, 0));
 ********************************************************************************/
int icst7735_fill_color(icst7735_t *lcd, uint16_t color);

/********************************************************************************
 * @brief   在目标区域绘制 RGB565 图像块.
 * @param   lcd    : 驱动句柄.
 * @param   x      : 目标起始 X.
 * @param   y      : 目标起始 Y.
 * @param   w      : 图像块宽度.
 * @param   h      : 图像块高度.
 * @param   pixels : RGB565 像素缓冲区，按行连续存储.
 * @return  成功返回 0，失败返回 -1.
 * @example icst7735_draw_rgb565(&lcd, 0, 0, 128, 160, frame565);
 ********************************************************************************/
int icst7735_draw_rgb565(icst7735_t *lcd, uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *pixels);

/********************************************************************************
 * @brief   将 BGR888 摄像头图像以彩色方式全屏显示.
 * @param   lcd                : 驱动句柄.
 * @param   frame_bgr888       : 输入 BGR888 图像缓冲区.
 * @param   frame_width        : 源图像宽度.
 * @param   frame_height       : 源图像高度.
 * @param   frame_stride_bytes : 源图像行步长（字节），传 0 表示 width*3.
 * @return  成功返回 0，失败返回 -1.
 * @note    源图像将使用最近邻缩放到屏幕尺寸.
 * @example icst7735_show_camera_bgr888(&lcd, frame, 640, 480, 0);
 ********************************************************************************/
int icst7735_show_camera_bgr888(icst7735_t *lcd,
                                const uint8_t *frame_bgr888,
                                uint16_t frame_width,
                                uint16_t frame_height,
                                uint32_t frame_stride_bytes);

/********************************************************************************
 * @brief   将 BGR888 摄像头图像以灰度方式全屏显示.
 * @param   lcd                : 驱动句柄.
 * @param   frame_bgr888       : 输入 BGR888 图像缓冲区.
 * @param   frame_width        : 源图像宽度.
 * @param   frame_height       : 源图像高度.
 * @param   frame_stride_bytes : 源图像行步长（字节），传 0 表示 width*3.
 * @return  成功返回 0，失败返回 -1.
 * @note    灰度转换采用整数亮度近似公式.
 * @example icst7735_show_camera_gray_bgr888(&lcd, frame, 640, 480, 0);
 ********************************************************************************/
int icst7735_show_camera_gray_bgr888(icst7735_t *lcd,
                                     const uint8_t *frame_bgr888,
                                     uint16_t frame_width,
                                     uint16_t frame_height,
                                     uint32_t frame_stride_bytes);

/********************************************************************************
 * @brief   打开或关闭面板显示.
 * @param   lcd : 驱动句柄.
 * @param   on  : 0 表示关闭，非 0 表示开启.
 * @return  成功返回 0，失败返回 -1.
 * @example icst7735_display_on(&lcd, 1);
 ********************************************************************************/
int icst7735_display_on(icst7735_t *lcd, int on);

/********************************************************************************
 * @brief   使能或关闭显示反色.
 * @param   lcd    : 驱动句柄.
 * @param   enable : 0 为正常显示，非 0 为反色显示.
 * @return  成功返回 0，失败返回 -1.
 * @example icst7735_invert_display(&lcd, 0);
 ********************************************************************************/
int icst7735_invert_display(icst7735_t *lcd, int enable);

#ifdef __cplusplus
}
#endif

#endif
