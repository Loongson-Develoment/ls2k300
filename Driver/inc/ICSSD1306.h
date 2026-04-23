#ifndef __ICSSD1306_H
#define __ICSSD1306_H

#include <stddef.h>
#include <stdint.h>

#include "LS2K0300_HW_I2C.h"

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************************
 * @brief   SSD1306 默认屏幕宽度（像素）.
 ********************************************************************************/
#define ICSSD1306_WIDTH_DEFAULT      (128U)

/********************************************************************************
 * @brief   SSD1306 默认屏幕高度（像素）.
 ********************************************************************************/
#define ICSSD1306_HEIGHT_DEFAULT     (64U)

/********************************************************************************
 * @brief   SSD1306 驱动支持的最大宽度（像素）.
 ********************************************************************************/
#define ICSSD1306_WIDTH_MAX          (128U)

/********************************************************************************
 * @brief   SSD1306 驱动支持的最大高度（像素）.
 ********************************************************************************/
#define ICSSD1306_HEIGHT_MAX         (64U)

/********************************************************************************
 * @brief   SSD1306 缓冲区最大字节数（1bit 灰度）.
 ********************************************************************************/
#define ICSSD1306_BUFFER_MAX_SIZE    ((ICSSD1306_WIDTH_MAX * ICSSD1306_HEIGHT_MAX) / 8U)

/********************************************************************************
 * @brief   SSD1306 像素颜色枚举.
 ********************************************************************************/
typedef enum {
    ICSSD1306_COLOR_BLACK = 0,
    ICSSD1306_COLOR_WHITE,
    ICSSD1306_COLOR_INVERSE,
} icssd1306_color_t;

/********************************************************************************
 * @brief   SSD1306 驱动句柄结构体.
 * @note    I2C 句柄会在以下接口中初始化：
 *          icssd1306_init().
 ********************************************************************************/
typedef struct {
    ls2k0300_hw_i2c_t i2c;

    uint16_t width;
    uint16_t height;
    uint8_t  pages;
    uint8_t  i2c_addr;

    int      initialized;
    uint8_t  buffer[ICSSD1306_BUFFER_MAX_SIZE];
} icssd1306_t;

/********************************************************************************
 * @brief   初始化 SSD1306（硬件 I2C）.
 * @param   oled     : 驱动句柄.
 * @param   port     : 硬件 I2C 端口.
 * @param   mux      : I2C 引脚复用组（m0/m1）.
 * @param   addr     : SSD1306 7bit 地址（常见 0x3C/0x3D）.
 * @param   speed_hz : I2C 总线速率（推荐 400000）.
 * @param   width    : 屏幕宽度（<=128）.
 * @param   height   : 屏幕高度（<=64，且需 8 对齐）.
 * @return  成功返回 0，失败返回 -1.
 * @example icssd1306_t oled;
 *          icssd1306_init(&oled, LS_HW_I2C1, LS_HW_I2C_MUX_M0, 0x3C, 400000, 128, 64);
 ********************************************************************************/
int icssd1306_init(icssd1306_t *oled,
                   ls_hw_i2c_port_t port,
                   ls_hw_i2c_mux_t mux,
                   uint8_t addr,
                   uint32_t speed_hz,
                   uint16_t width,
                   uint16_t height);

/********************************************************************************
 * @brief   释放 SSD1306 驱动资源.
 * @param   oled : 驱动句柄.
 * @return  none.
 * @example icssd1306_deinit(&oled);
 ********************************************************************************/
void icssd1306_deinit(icssd1306_t *oled);

/********************************************************************************
 * @brief   打开或关闭屏幕显示.
 * @param   oled : 驱动句柄.
 * @param   on   : 0 表示关闭，非 0 表示开启.
 * @return  成功返回 0，失败返回 -1.
 * @example icssd1306_display_on(&oled, 1);
 ********************************************************************************/
int icssd1306_display_on(icssd1306_t *oled, int on);

/********************************************************************************
 * @brief   开启或关闭反色显示.
 * @param   oled   : 驱动句柄.
 * @param   enable : 0 为正常显示，非 0 为反色显示.
 * @return  成功返回 0，失败返回 -1.
 * @example icssd1306_invert_display(&oled, 1);
 ********************************************************************************/
int icssd1306_invert_display(icssd1306_t *oled, int enable);

/********************************************************************************
 * @brief   设置屏幕对比度.
 * @param   oled     : 驱动句柄.
 * @param   contrast : 对比度值（0~255）.
 * @return  成功返回 0，失败返回 -1.
 * @example icssd1306_set_contrast(&oled, 0x7F);
 ********************************************************************************/
int icssd1306_set_contrast(icssd1306_t *oled, uint8_t contrast);

/********************************************************************************
 * @brief   清空/填充显示缓冲区（仅修改 RAM 缓冲，不立即刷屏）.
 * @param   oled  : 驱动句柄.
 * @param   color : 颜色（BLACK/WHITE/INVERSE）.
 * @return  成功返回 0，失败返回 -1.
 * @example icssd1306_clear_buffer(&oled, ICSSD1306_COLOR_BLACK);
 ********************************************************************************/
int icssd1306_clear_buffer(icssd1306_t *oled, icssd1306_color_t color);

/********************************************************************************
 * @brief   绘制单个像素到显示缓冲区（不立即刷屏）.
 * @param   oled  : 驱动句柄.
 * @param   x     : 像素 X 坐标.
 * @param   y     : 像素 Y 坐标.
 * @param   color : 颜色（BLACK/WHITE/INVERSE）.
 * @return  成功返回 0，失败返回 -1.
 * @example icssd1306_draw_pixel(&oled, 10, 12, ICSSD1306_COLOR_WHITE);
 ********************************************************************************/
int icssd1306_draw_pixel(icssd1306_t *oled, uint16_t x, uint16_t y, icssd1306_color_t color);

/********************************************************************************
 * @brief   将显示缓冲区刷新到屏幕.
 * @param   oled : 驱动句柄.
 * @return  成功返回 0，失败返回 -1.
 * @example icssd1306_update_screen(&oled);
 ********************************************************************************/
int icssd1306_update_screen(icssd1306_t *oled);

/********************************************************************************
 * @brief   全屏填充指定颜色并立即刷新.
 * @param   oled  : 驱动句柄.
 * @param   color : 颜色（BLACK/WHITE/INVERSE）.
 * @return  成功返回 0，失败返回 -1.
 * @example icssd1306_fill_screen(&oled, ICSSD1306_COLOR_BLACK);
 ********************************************************************************/
int icssd1306_fill_screen(icssd1306_t *oled, icssd1306_color_t color);

/********************************************************************************
 * @brief   将 BGR888 摄像头图像转灰度后二值化显示到 SSD1306.
 * @param   oled               : 驱动句柄.
 * @param   frame_bgr888       : 输入 BGR888 图像缓冲区.
 * @param   frame_width        : 源图像宽度.
 * @param   frame_height       : 源图像高度.
 * @param   frame_stride_bytes : 源图像行步长（字节），传 0 表示 width*3.
 * @param   threshold          : 二值化阈值（0~255）.
 * @return  成功返回 0，失败返回 -1.
 * @note    源图像采用最近邻缩放到屏幕分辨率.
 * @example icssd1306_show_camera_gray_bgr888(&oled, frame, 640, 480, 0, 128);
 ********************************************************************************/
int icssd1306_show_camera_gray_bgr888(icssd1306_t *oled,
                                      const uint8_t *frame_bgr888,
                                      uint16_t frame_width,
                                      uint16_t frame_height,
                                      uint32_t frame_stride_bytes,
                                      uint8_t threshold);

#ifdef __cplusplus
}
#endif

#endif
