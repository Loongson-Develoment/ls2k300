#include "ICSSD1306.h"

#include <string.h>

#define ICSSD1306_I2C_CTRL_CMD      (0x00U)
#define ICSSD1306_I2C_CTRL_DATA     (0x40U)
#define ICSSD1306_TX_CHUNK_BYTES    (16U)

#define ICSSD1306_CMD_SETCONTRAST           (0x81U)
#define ICSSD1306_CMD_DISPLAYALLON_RESUME   (0xA4U)
#define ICSSD1306_CMD_NORMALDISPLAY         (0xA6U)
#define ICSSD1306_CMD_INVERTDISPLAY         (0xA7U)
#define ICSSD1306_CMD_DISPLAYOFF            (0xAEU)
#define ICSSD1306_CMD_DISPLAYON             (0xAFU)
#define ICSSD1306_CMD_SETDISPLAYOFFSET      (0xD3U)
#define ICSSD1306_CMD_SETCOMPINS            (0xDAU)
#define ICSSD1306_CMD_SETVCOMDETECT         (0xDBU)
#define ICSSD1306_CMD_SETDISPLAYCLOCKDIV    (0xD5U)
#define ICSSD1306_CMD_SETPRECHARGE          (0xD9U)
#define ICSSD1306_CMD_SETMULTIPLEX          (0xA8U)
#define ICSSD1306_CMD_SETSTARTLINE          (0x40U)
#define ICSSD1306_CMD_CHARGEPUMP            (0x8DU)
#define ICSSD1306_CMD_MEMORYMODE            (0x20U)
#define ICSSD1306_CMD_SEGREMAP              (0xA0U)
#define ICSSD1306_CMD_COMSCANDEC            (0xC8U)
#define ICSSD1306_CMD_COLUMNADDR            (0x21U)
#define ICSSD1306_CMD_PAGEADDR              (0x22U)

/********************************************************************************
 * @brief   发送单字节命令.
 * @param   oled : 驱动句柄.
 * @param   cmd  : 命令字节.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
static int icssd1306_write_cmd(icssd1306_t *oled, uint8_t cmd)
{
    if (oled == NULL) {
        return -1;
    }

    return (ls2k0300_hw_i2c_write_byte(&oled->i2c, ICSSD1306_I2C_CTRL_CMD, cmd) == 0U) ? 0 : -1;
}

/********************************************************************************
 * @brief   连续发送命令序列.
 * @param   oled : 驱动句柄.
 * @param   cmd  : 命令缓冲区.
 * @param   len  : 命令字节数.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
static int icssd1306_write_cmds(icssd1306_t *oled, const uint8_t *cmd, uint16_t len)
{
    if (oled == NULL) {
        return -1;
    }

    if (len == 0U) {
        return 0;
    }

    if (cmd == NULL) {
        return -1;
    }

    return (ls2k0300_hw_i2c_write_n_byte(&oled->i2c, ICSSD1306_I2C_CTRL_CMD, cmd, len) == 0U) ? 0 : -1;
}

/********************************************************************************
 * @brief   分块发送显示数据.
 * @param   oled : 驱动句柄.
 * @param   data : 数据缓冲区.
 * @param   len  : 数据长度（字节）.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
static int icssd1306_write_data(icssd1306_t *oled, const uint8_t *data, size_t len)
{
    size_t offset;

    if (oled == NULL) {
        return -1;
    }

    if (len == 0U) {
        return 0;
    }

    if (data == NULL) {
        return -1;
    }

    for (offset = 0U; offset < len; offset += ICSSD1306_TX_CHUNK_BYTES) {
        size_t remain = len - offset;
        uint16_t chunk = (uint16_t)((remain > ICSSD1306_TX_CHUNK_BYTES) ? ICSSD1306_TX_CHUNK_BYTES : remain);

        if (ls2k0300_hw_i2c_write_n_byte(&oled->i2c,
                                         ICSSD1306_I2C_CTRL_DATA,
                                         &data[offset],
                                         chunk) != 0U) {
            return -1;
        }
    }

    return 0;
}

/********************************************************************************
 * @brief   执行 SSD1306 上电初始化命令序列.
 * @param   oled : 驱动句柄.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
static int icssd1306_init_sequence(icssd1306_t *oled)
{
    uint8_t compins;
    uint8_t contrast;
    uint8_t init_cmds[26];
    uint16_t idx = 0U;

    if (oled == NULL || oled->height == 0U) {
        return -1;
    }

    compins = (oled->height == 64U) ? 0x12U : 0x02U;
    contrast = (oled->height == 64U) ? 0x7FU : 0x8FU;

    init_cmds[idx++] = ICSSD1306_CMD_DISPLAYOFF;
    init_cmds[idx++] = ICSSD1306_CMD_SETDISPLAYCLOCKDIV;
    init_cmds[idx++] = 0x80U;
    init_cmds[idx++] = ICSSD1306_CMD_SETMULTIPLEX;
    init_cmds[idx++] = (uint8_t)(oled->height - 1U);
    init_cmds[idx++] = ICSSD1306_CMD_SETDISPLAYOFFSET;
    init_cmds[idx++] = 0x00U;
    init_cmds[idx++] = ICSSD1306_CMD_SETSTARTLINE | 0x00U;
    init_cmds[idx++] = ICSSD1306_CMD_CHARGEPUMP;
    init_cmds[idx++] = 0x14U;
    init_cmds[idx++] = ICSSD1306_CMD_MEMORYMODE;
    init_cmds[idx++] = 0x00U;
    init_cmds[idx++] = ICSSD1306_CMD_SEGREMAP | 0x01U;
    init_cmds[idx++] = ICSSD1306_CMD_COMSCANDEC;
    init_cmds[idx++] = ICSSD1306_CMD_SETCOMPINS;
    init_cmds[idx++] = compins;
    init_cmds[idx++] = ICSSD1306_CMD_SETCONTRAST;
    init_cmds[idx++] = contrast;
    init_cmds[idx++] = ICSSD1306_CMD_SETPRECHARGE;
    init_cmds[idx++] = 0xF1U;
    init_cmds[idx++] = ICSSD1306_CMD_SETVCOMDETECT;
    init_cmds[idx++] = 0x40U;
    init_cmds[idx++] = ICSSD1306_CMD_DISPLAYALLON_RESUME;
    init_cmds[idx++] = ICSSD1306_CMD_NORMALDISPLAY;
    init_cmds[idx++] = ICSSD1306_CMD_DISPLAYON;

    return icssd1306_write_cmds(oled, init_cmds, idx);
}

/********************************************************************************
 * @brief   初始化 SSD1306（硬件 I2C）.
 * @param   oled     : 驱动句柄.
 * @param   port     : 硬件 I2C 端口.
 * @param   mux      : I2C 引脚复用组（m0/m1）.
 * @param   addr     : SSD1306 7bit 地址.
 * @param   speed_hz : I2C 总线速率.
 * @param   width    : 屏幕宽度.
 * @param   height   : 屏幕高度.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icssd1306_init(icssd1306_t *oled,
                   ls_hw_i2c_port_t port,
                   ls_hw_i2c_mux_t mux,
                   uint8_t addr,
                   uint32_t speed_hz,
                   uint16_t width,
                   uint16_t height)
{
    if (oled == NULL ||
        port >= LS_HW_I2C_INVALID ||
        mux >= LS_HW_I2C_MUX_INVALID ||
        addr > 0x7FU ||
        width == 0U ||
        width > ICSSD1306_WIDTH_MAX ||
        height == 0U ||
        height > ICSSD1306_HEIGHT_MAX ||
        (height % 8U) != 0U) {
        return -1;
    }

    memset(oled, 0, sizeof(*oled));
    oled->width = width;
    oled->height = height;
    oled->pages = (uint8_t)(height / 8U);
    oled->i2c_addr = addr;

    if (speed_hz == 0U) {
        speed_hz = 400000U;
    }

    if (ls2k0300_hw_i2c_init_ex(&oled->i2c, port, mux, addr, speed_hz) != 0) {
        return -1;
    }

    oled->initialized = 1;

    if (icssd1306_init_sequence(oled) != 0) {
        icssd1306_deinit(oled);
        return -1;
    }

    if (icssd1306_fill_screen(oled, ICSSD1306_COLOR_BLACK) != 0) {
        icssd1306_deinit(oled);
        return -1;
    }

    return 0;
}

/********************************************************************************
 * @brief   释放 SSD1306 驱动资源.
 * @param   oled : 驱动句柄.
 * @return  none.
 ********************************************************************************/
void icssd1306_deinit(icssd1306_t *oled)
{
    if (oled == NULL) {
        return;
    }

    if (oled->initialized != 0) {
        (void)icssd1306_display_on(oled, 0);
    }

    if (oled->i2c.initialized != 0) {
        ls2k0300_hw_i2c_deinit(&oled->i2c);
    }

    memset(oled, 0, sizeof(*oled));
}

/********************************************************************************
 * @brief   打开或关闭屏幕显示.
 * @param   oled : 驱动句柄.
 * @param   on   : 0 表示关闭，非 0 表示开启.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icssd1306_display_on(icssd1306_t *oled, int on)
{
    if (oled == NULL || oled->initialized == 0) {
        return -1;
    }

    return icssd1306_write_cmd(oled, (on != 0) ? ICSSD1306_CMD_DISPLAYON : ICSSD1306_CMD_DISPLAYOFF);
}

/********************************************************************************
 * @brief   开启或关闭反色显示.
 * @param   oled   : 驱动句柄.
 * @param   enable : 0 为正常显示，非 0 为反色显示.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icssd1306_invert_display(icssd1306_t *oled, int enable)
{
    if (oled == NULL || oled->initialized == 0) {
        return -1;
    }

    return icssd1306_write_cmd(oled, (enable != 0) ? ICSSD1306_CMD_INVERTDISPLAY : ICSSD1306_CMD_NORMALDISPLAY);
}

/********************************************************************************
 * @brief   设置屏幕对比度.
 * @param   oled     : 驱动句柄.
 * @param   contrast : 对比度值.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icssd1306_set_contrast(icssd1306_t *oled, uint8_t contrast)
{
    uint8_t cmd[2];

    if (oled == NULL || oled->initialized == 0) {
        return -1;
    }

    cmd[0] = ICSSD1306_CMD_SETCONTRAST;
    cmd[1] = contrast;

    return icssd1306_write_cmds(oled, cmd, (uint16_t)sizeof(cmd));
}

/********************************************************************************
 * @brief   清空/填充显示缓冲区（仅修改 RAM 缓冲，不立即刷屏）.
 * @param   oled  : 驱动句柄.
 * @param   color : 颜色（BLACK/WHITE/INVERSE）.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icssd1306_clear_buffer(icssd1306_t *oled, icssd1306_color_t color)
{
    uint16_t bytes;

    if (oled == NULL || oled->initialized == 0) {
        return -1;
    }

    bytes = (uint16_t)((uint16_t)oled->width * (uint16_t)oled->pages);

    if (color == ICSSD1306_COLOR_WHITE) {
        memset(oled->buffer, 0xFF, bytes);
        return 0;
    }

    if (color == ICSSD1306_COLOR_BLACK) {
        memset(oled->buffer, 0x00, bytes);
        return 0;
    }

    if (color == ICSSD1306_COLOR_INVERSE) {
        uint16_t i;
        for (i = 0U; i < bytes; ++i) {
            oled->buffer[i] = (uint8_t)(~oled->buffer[i]);
        }
        return 0;
    }

    return -1;
}

/********************************************************************************
 * @brief   绘制单个像素到显示缓冲区（不立即刷屏）.
 * @param   oled  : 驱动句柄.
 * @param   x     : 像素 X 坐标.
 * @param   y     : 像素 Y 坐标.
 * @param   color : 颜色（BLACK/WHITE/INVERSE）.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icssd1306_draw_pixel(icssd1306_t *oled, uint16_t x, uint16_t y, icssd1306_color_t color)
{
    size_t index;
    uint8_t bit_mask;

    if (oled == NULL || oled->initialized == 0) {
        return -1;
    }

    if (x >= oled->width || y >= oled->height) {
        return -1;
    }

    index = (size_t)x + ((size_t)(y / 8U) * (size_t)oled->width);
    bit_mask = (uint8_t)(1U << (y % 8U));

    if (color == ICSSD1306_COLOR_WHITE) {
        oled->buffer[index] |= bit_mask;
    } else if (color == ICSSD1306_COLOR_BLACK) {
        oled->buffer[index] &= (uint8_t)(~bit_mask);
    } else if (color == ICSSD1306_COLOR_INVERSE) {
        oled->buffer[index] ^= bit_mask;
    } else {
        return -1;
    }

    return 0;
}

/********************************************************************************
 * @brief   将显示缓冲区刷新到屏幕.
 * @param   oled : 驱动句柄.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icssd1306_update_screen(icssd1306_t *oled)
{
    uint8_t set_addr_cmd[6];
    uint16_t bytes;

    if (oled == NULL || oled->initialized == 0) {
        return -1;
    }

    bytes = (uint16_t)((uint16_t)oled->width * (uint16_t)oled->pages);

    set_addr_cmd[0] = ICSSD1306_CMD_COLUMNADDR;
    set_addr_cmd[1] = 0x00U;
    set_addr_cmd[2] = (uint8_t)(oled->width - 1U);
    set_addr_cmd[3] = ICSSD1306_CMD_PAGEADDR;
    set_addr_cmd[4] = 0x00U;
    set_addr_cmd[5] = (uint8_t)(oled->pages - 1U);

    if (icssd1306_write_cmds(oled, set_addr_cmd, (uint16_t)sizeof(set_addr_cmd)) != 0) {
        return -1;
    }

    return icssd1306_write_data(oled, oled->buffer, bytes);
}

/********************************************************************************
 * @brief   全屏填充指定颜色并立即刷新.
 * @param   oled  : 驱动句柄.
 * @param   color : 颜色（BLACK/WHITE/INVERSE）.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icssd1306_fill_screen(icssd1306_t *oled, icssd1306_color_t color)
{
    if (icssd1306_clear_buffer(oled, color) != 0) {
        return -1;
    }

    return icssd1306_update_screen(oled);
}

/********************************************************************************
 * @brief   将 BGR888 摄像头图像转灰度后二值化显示到 SSD1306.
 * @param   oled               : 驱动句柄.
 * @param   frame_bgr888       : 输入 BGR888 图像缓冲区.
 * @param   frame_width        : 源图像宽度.
 * @param   frame_height       : 源图像高度.
 * @param   frame_stride_bytes : 源图像行步长（字节）.
 * @param   threshold          : 二值化阈值（0~255）.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icssd1306_show_camera_gray_bgr888(icssd1306_t *oled,
                                      const uint8_t *frame_bgr888,
                                      uint16_t frame_width,
                                      uint16_t frame_height,
                                      uint32_t frame_stride_bytes,
                                      uint8_t threshold)
{
    uint16_t x;
    uint16_t y;

    if (oled == NULL || oled->initialized == 0 || frame_bgr888 == NULL ||
        frame_width == 0U || frame_height == 0U) {
        return -1;
    }

    if (frame_stride_bytes == 0U) {
        frame_stride_bytes = (uint32_t)frame_width * 3U;
    }

    if (frame_stride_bytes < ((uint32_t)frame_width * 3U)) {
        return -1;
    }

    if (icssd1306_clear_buffer(oled, ICSSD1306_COLOR_BLACK) != 0) {
        return -1;
    }

    for (y = 0U; y < oled->height; ++y) {
        uint32_t src_y = ((uint32_t)y * (uint32_t)frame_height) / (uint32_t)oled->height;
        const uint8_t *src_row = frame_bgr888 + (src_y * frame_stride_bytes);

        for (x = 0U; x < oled->width; ++x) {
            uint32_t src_x = ((uint32_t)x * (uint32_t)frame_width) / (uint32_t)oled->width;
            const uint8_t *pix = src_row + (src_x * 3U);
            uint8_t b = pix[0];
            uint8_t g = pix[1];
            uint8_t r = pix[2];
            uint16_t gray = (uint16_t)((77U * (uint16_t)r + 150U * (uint16_t)g + 29U * (uint16_t)b) >> 8U);

            (void)icssd1306_draw_pixel(oled,
                                       x,
                                       y,
                                       (gray >= (uint16_t)threshold) ? ICSSD1306_COLOR_WHITE : ICSSD1306_COLOR_BLACK);
        }
    }

    return icssd1306_update_screen(oled);
}
