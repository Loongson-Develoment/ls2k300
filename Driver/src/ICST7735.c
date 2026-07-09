#define _POSIX_C_SOURCE 200809L

#include "ICST7735.h"

#include <errno.h>
#include <string.h>
#include <time.h>

#define ICST7735_CMD_SWRESET  (0x01U)
#define ICST7735_CMD_SLPOUT   (0x11U)
#define ICST7735_CMD_DISPOFF  (0x28U)
#define ICST7735_CMD_DISPON   (0x29U)
#define ICST7735_CMD_CASET    (0x2AU)
#define ICST7735_CMD_RASET    (0x2BU)
#define ICST7735_CMD_RAMWR    (0x2CU)
#define ICST7735_CMD_MADCTL   (0x36U)
#define ICST7735_CMD_COLMOD   (0x3AU)
#define ICST7735_CMD_NORON    (0x13U)
#define ICST7735_CMD_INVON    (0x21U)
#define ICST7735_CMD_INVOFF   (0x20U)
#define ICST7735_CMD_FRMCTR1  (0xB1U)
#define ICST7735_CMD_FRMCTR2  (0xB2U)
#define ICST7735_CMD_FRMCTR3  (0xB3U)
#define ICST7735_CMD_INVCTR   (0xB4U)
#define ICST7735_CMD_PWCTR1   (0xC0U)
#define ICST7735_CMD_PWCTR2   (0xC1U)
#define ICST7735_CMD_PWCTR3   (0xC2U)
#define ICST7735_CMD_PWCTR4   (0xC3U)
#define ICST7735_CMD_PWCTR5   (0xC4U)
#define ICST7735_CMD_VMCTR1   (0xC5U)
#define ICST7735_CMD_GMCTRP1  (0xE0U)
#define ICST7735_CMD_GMCTRN1  (0xE1U)

#define ICST7735_MADCTL_MY    (0x80U)
#define ICST7735_MADCTL_MX    (0x40U)
#define ICST7735_MADCTL_MV    (0x20U)
#define ICST7735_MADCTL_BGR   (0x08U)

#define ICST7735_TX_CHUNK_PIXELS  (42480U) /* SPI 传输分块像素数，需保证单块数据不超过 64KB */

/********************************************************************************
 * @brief   5x7 英文/数字字模数据表.
 * @note    每个字符由 5 列组成，每列低 7 bit 表示自上而下像素点.
 ********************************************************************************/
static const uint8_t g_icst7735_font5x7_digits[10][ICST7735_FONT5X7_WIDTH] = {
    {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU}, /* 0 */
    {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U}, /* 1 */
    {0x42U, 0x61U, 0x51U, 0x49U, 0x46U}, /* 2 */
    {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U}, /* 3 */
    {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U}, /* 4 */
    {0x27U, 0x45U, 0x45U, 0x45U, 0x39U}, /* 5 */
    {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U}, /* 6 */
    {0x01U, 0x71U, 0x09U, 0x05U, 0x03U}, /* 7 */
    {0x36U, 0x49U, 0x49U, 0x49U, 0x36U}, /* 8 */
    {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}, /* 9 */
};

static const uint8_t g_icst7735_font5x7_upper[26][ICST7735_FONT5X7_WIDTH] = {
    {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU}, /* A */
    {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U}, /* B */
    {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U}, /* C */
    {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU}, /* D */
    {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U}, /* E */
    {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U}, /* F */
    {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU}, /* G */
    {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU}, /* H */
    {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U}, /* I */
    {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U}, /* J */
    {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U}, /* K */
    {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U}, /* L */
    {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU}, /* M */
    {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU}, /* N */
    {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU}, /* O */
    {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U}, /* P */
    {0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU}, /* Q */
    {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U}, /* R */
    {0x46U, 0x49U, 0x49U, 0x49U, 0x31U}, /* S */
    {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U}, /* T */
    {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU}, /* U */
    {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU}, /* V */
    {0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU}, /* W */
    {0x63U, 0x14U, 0x08U, 0x14U, 0x63U}, /* X */
    {0x07U, 0x08U, 0x70U, 0x08U, 0x07U}, /* Y */
    {0x61U, 0x51U, 0x49U, 0x45U, 0x43U}, /* Z */
};

static const uint8_t g_icst7735_font5x7_lower[26][ICST7735_FONT5X7_WIDTH] = {
    {0x20U, 0x54U, 0x54U, 0x54U, 0x78U}, /* a */
    {0x7FU, 0x48U, 0x44U, 0x44U, 0x38U}, /* b */
    {0x38U, 0x44U, 0x44U, 0x44U, 0x20U}, /* c */
    {0x38U, 0x44U, 0x44U, 0x48U, 0x7FU}, /* d */
    {0x38U, 0x54U, 0x54U, 0x54U, 0x18U}, /* e */
    {0x08U, 0x7EU, 0x09U, 0x01U, 0x02U}, /* f */
    {0x0CU, 0x52U, 0x52U, 0x52U, 0x3EU}, /* g */
    {0x7FU, 0x08U, 0x04U, 0x04U, 0x78U}, /* h */
    {0x00U, 0x44U, 0x7DU, 0x40U, 0x00U}, /* i */
    {0x20U, 0x40U, 0x44U, 0x3DU, 0x00U}, /* j */
    {0x7FU, 0x10U, 0x28U, 0x44U, 0x00U}, /* k */
    {0x00U, 0x41U, 0x7FU, 0x40U, 0x00U}, /* l */
    {0x7CU, 0x04U, 0x18U, 0x04U, 0x78U}, /* m */
    {0x7CU, 0x08U, 0x04U, 0x04U, 0x78U}, /* n */
    {0x38U, 0x44U, 0x44U, 0x44U, 0x38U}, /* o */
    {0x7CU, 0x14U, 0x14U, 0x14U, 0x08U}, /* p */
    {0x08U, 0x14U, 0x14U, 0x18U, 0x7CU}, /* q */
    {0x7CU, 0x08U, 0x04U, 0x04U, 0x08U}, /* r */
    {0x48U, 0x54U, 0x54U, 0x54U, 0x20U}, /* s */
    {0x04U, 0x3FU, 0x44U, 0x40U, 0x20U}, /* t */
    {0x3CU, 0x40U, 0x40U, 0x20U, 0x7CU}, /* u */
    {0x1CU, 0x20U, 0x40U, 0x20U, 0x1CU}, /* v */
    {0x3CU, 0x40U, 0x30U, 0x40U, 0x3CU}, /* w */
    {0x44U, 0x28U, 0x10U, 0x28U, 0x44U}, /* x */
    {0x0CU, 0x50U, 0x50U, 0x50U, 0x3CU}, /* y */
    {0x44U, 0x64U, 0x54U, 0x4CU, 0x44U}, /* z */
};

/********************************************************************************
 * @brief   查询单个字符的 5x7 字模.
 * @param   ch      : 输入字符.
 * @param   glyph5  : 输出字模（长度 5 字节）.
 * @return  none.
 ********************************************************************************/
static void icst7735_get_glyph5x7(char ch, uint8_t glyph5[ICST7735_FONT5X7_WIDTH])
{
    static const uint8_t blank[ICST7735_FONT5X7_WIDTH] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    static const uint8_t question[ICST7735_FONT5X7_WIDTH] = {0x02U, 0x01U, 0x51U, 0x09U, 0x06U};
    static const uint8_t exclam[ICST7735_FONT5X7_WIDTH] = {0x00U, 0x00U, 0x5FU, 0x00U, 0x00U};
    static const uint8_t dash[ICST7735_FONT5X7_WIDTH] = {0x08U, 0x08U, 0x08U, 0x08U, 0x08U};
    static const uint8_t underline[ICST7735_FONT5X7_WIDTH] = {0x40U, 0x40U, 0x40U, 0x40U, 0x40U};
    static const uint8_t dot[ICST7735_FONT5X7_WIDTH] = {0x00U, 0x60U, 0x60U, 0x00U, 0x00U};
    static const uint8_t colon[ICST7735_FONT5X7_WIDTH] = {0x00U, 0x36U, 0x36U, 0x00U, 0x00U};
    static const uint8_t slash[ICST7735_FONT5X7_WIDTH] = {0x20U, 0x10U, 0x08U, 0x04U, 0x02U};
    static const uint8_t plus[ICST7735_FONT5X7_WIDTH] = {0x08U, 0x08U, 0x3EU, 0x08U, 0x08U};
    static const uint8_t equal[ICST7735_FONT5X7_WIDTH] = {0x14U, 0x14U, 0x14U, 0x14U, 0x14U};

    if (glyph5 == NULL) {
        return;
    }

    if (ch >= '0' && ch <= '9') {
        memcpy(glyph5, g_icst7735_font5x7_digits[(uint8_t)(ch - '0')], ICST7735_FONT5X7_WIDTH);
        return;
    }

    if (ch >= 'A' && ch <= 'Z') {
        memcpy(glyph5, g_icst7735_font5x7_upper[(uint8_t)(ch - 'A')], ICST7735_FONT5X7_WIDTH);
        return;
    }

    if (ch >= 'a' && ch <= 'z') {
        memcpy(glyph5, g_icst7735_font5x7_lower[(uint8_t)(ch - 'a')], ICST7735_FONT5X7_WIDTH);
        return;
    }

    switch (ch) {
    case ' ': memcpy(glyph5, blank, ICST7735_FONT5X7_WIDTH); break;
    case '?': memcpy(glyph5, question, ICST7735_FONT5X7_WIDTH); break;
    case '!': memcpy(glyph5, exclam, ICST7735_FONT5X7_WIDTH); break;
    case '-': memcpy(glyph5, dash, ICST7735_FONT5X7_WIDTH); break;
    case '_': memcpy(glyph5, underline, ICST7735_FONT5X7_WIDTH); break;
    case '.': memcpy(glyph5, dot, ICST7735_FONT5X7_WIDTH); break;
    case ':': memcpy(glyph5, colon, ICST7735_FONT5X7_WIDTH); break;
    case '/': memcpy(glyph5, slash, ICST7735_FONT5X7_WIDTH); break;
    case '+': memcpy(glyph5, plus, ICST7735_FONT5X7_WIDTH); break;
    case '=': memcpy(glyph5, equal, ICST7735_FONT5X7_WIDTH); break;
    default:  memcpy(glyph5, question, ICST7735_FONT5X7_WIDTH); break;
    }
}

/********************************************************************************
 * @brief   毫秒级延时辅助函数.
 * @param   ms : 延时毫秒数.
 * @return  none.
 ********************************************************************************/
static void icst7735_delay_ms(uint32_t ms)
{
    struct timespec req;
    struct timespec rem;

    if (ms == 0U) {
        return;
    }

    req.tv_sec = (time_t)(ms / 1000U);
    req.tv_nsec = (long)((ms % 1000U) * 1000000UL);

    while (nanosleep(&req, &rem) != 0) {
        if (errno != EINTR) {
            break;
        }
        req = rem;
    }
}

/********************************************************************************
 * @brief   向 ST7735 发送 1 字节命令.
 * @param   lcd : 驱动句柄.
 * @param   cmd : 命令字节.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
static int icst7735_write_command(icst7735_t *lcd, uint8_t cmd)
{
    if (lcd == NULL) {
        return -1;
    }

    ls2k0300_gpio_level_set(&lcd->dc_gpio, GPIO_LOW);
    return (ls2k0300_spi_write(&lcd->spi, &cmd, 1U) == 0) ? 0 : -1;
}

/********************************************************************************
 * @brief   向 ST7735 发送数据负载.
 * @param   lcd  : 驱动句柄.
 * @param   data : 输入数据缓冲区.
 * @param   len  : 数据长度（字节）.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
static int icst7735_write_data(icst7735_t *lcd, const uint8_t *data, size_t len)
{
    if (lcd == NULL) {
        return -1;
    }

    if (len == 0U) {
        return 0;
    }

    if (data == NULL) {
        return -1;
    }

    ls2k0300_gpio_level_set(&lcd->dc_gpio, GPIO_HIGH);
    return (ls2k0300_spi_write(&lcd->spi, data, len) == 0) ? 0 : -1;
}

/********************************************************************************
 * @brief   发送命令与可选数据，并按需延时.
 * @param   lcd      : 驱动句柄.
 * @param   cmd      : 命令字节.
 * @param   data     : 可选数据指针.
 * @param   len      : 数据长度.
 * @param   delay_ms : 命令序列后的延时.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
static int icst7735_write_cmd_data(icst7735_t *lcd, uint8_t cmd, const uint8_t *data, size_t len, uint32_t delay_ms)
{
    if (icst7735_write_command(lcd, cmd) != 0) {
        return -1;
    }

    if (len > 0U && icst7735_write_data(lcd, data, len) != 0) {
        return -1;
    }

    icst7735_delay_ms(delay_ms);
    return 0;
}

/********************************************************************************
 * @brief   将旋转配置应用到控制器 MADCTL.
 * @param   lcd      : 驱动句柄.
 * @param   rotation : 旋转模式.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
static int icst7735_apply_rotation(icst7735_t *lcd, icst7735_rotation_t rotation)
{
    uint8_t madctl;
    uint16_t rot_x_offset;
    uint16_t rot_y_offset;

    if (lcd == NULL || rotation >= ICST7735_ROTATION_INVALID) {
        return -1;
    }

    switch (rotation) {
    case ICST7735_ROTATION_0:
        madctl = (uint8_t)(ICST7735_MADCTL_MX | ICST7735_MADCTL_MY | ICST7735_MADCTL_BGR);
        break;
    case ICST7735_ROTATION_90:
        madctl = (uint8_t)(ICST7735_MADCTL_MY | ICST7735_MADCTL_MV | ICST7735_MADCTL_BGR);
        break;
    case ICST7735_ROTATION_180:
        madctl = ICST7735_MADCTL_BGR;
        break;
    case ICST7735_ROTATION_270:
        madctl = (uint8_t)(ICST7735_MADCTL_MX | ICST7735_MADCTL_MV | ICST7735_MADCTL_BGR);
        break;
    default:
        return -1;
    }

#if (ICST7735_CFG_ROTATION_AUTO_ADAPT != 0U)
    switch (rotation) {
    case ICST7735_ROTATION_0:
        lcd->width = lcd->native_width;
        lcd->height = lcd->native_height;
        rot_x_offset = lcd->base_x_offset;
        rot_y_offset = lcd->base_y_offset;
        break;
    case ICST7735_ROTATION_90:
        lcd->width = lcd->native_height;
        lcd->height = lcd->native_width;
        rot_x_offset = lcd->base_y_offset;
        rot_y_offset = lcd->base_x_offset;
        break;
    case ICST7735_ROTATION_180:
        lcd->width = lcd->native_width;
        lcd->height = lcd->native_height;
        rot_x_offset = lcd->base_x_offset;
        rot_y_offset = lcd->base_y_offset;
        break;
    case ICST7735_ROTATION_270:
        lcd->width = lcd->native_height;
        lcd->height = lcd->native_width;
        rot_x_offset = lcd->base_y_offset;
        rot_y_offset = lcd->base_x_offset;
        break;
    default:
        return -1;
    }
#else
    lcd->width = lcd->native_width;
    lcd->height = lcd->native_height;
    rot_x_offset = lcd->base_x_offset;
    rot_y_offset = lcd->base_y_offset;
#endif

    if (icst7735_write_cmd_data(lcd, ICST7735_CMD_MADCTL, &madctl, 1U, 0U) != 0) {
        return -1;
    }

    lcd->x_offset = rot_x_offset;
    lcd->y_offset = rot_y_offset;
    lcd->rotation = rotation;
    return 0;
}

/********************************************************************************
 * @brief   执行 ST7735 上电初始化序列.
 * @param   lcd : 驱动句柄.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
static int icst7735_init_sequence(icst7735_t *lcd)
{
    static const uint8_t colmod[] = {0x05U}; /* 16-bit/pixel RGB565 */
    static const uint8_t frmctr1[] = {0x01U, 0x2CU, 0x2DU};
    static const uint8_t frmctr2[] = {0x01U, 0x2CU, 0x2DU};
    static const uint8_t frmctr3[] = {0x01U, 0x2CU, 0x2DU, 0x01U, 0x2CU, 0x2DU};
    static const uint8_t invctr[] = {0x07U};
    static const uint8_t pwctr1[] = {0xA2U, 0x02U, 0x84U};
    static const uint8_t pwctr2[] = {0xC5U};
    static const uint8_t pwctr3[] = {0x0AU, 0x00U};
    static const uint8_t pwctr4[] = {0x8AU, 0x2AU};
    static const uint8_t pwctr5[] = {0x8AU, 0xEEU};
    static const uint8_t vmctr1[] = {0x0EU};
    static const uint8_t gmctrp1[] = {
        0x0FU, 0x1AU, 0x0FU, 0x18U, 0x2FU, 0x28U, 0x20U, 0x22U,
        0x1FU, 0x1BU, 0x23U, 0x37U, 0x00U, 0x07U, 0x02U, 0x10U
    };
    static const uint8_t gmctrn1[] = {
        0x0FU, 0x1BU, 0x0FU, 0x17U, 0x33U, 0x2CU, 0x29U, 0x2EU,
        0x30U, 0x30U, 0x39U, 0x3FU, 0x00U, 0x07U, 0x03U, 0x10U
    };

    if (icst7735_write_cmd_data(lcd, ICST7735_CMD_SWRESET, NULL, 0U, 150U) != 0) {
        return -1;
    }
    if (icst7735_write_cmd_data(lcd, ICST7735_CMD_SLPOUT, NULL, 0U, 120U) != 0) {
        return -1;
    }
    if (icst7735_write_cmd_data(lcd, ICST7735_CMD_FRMCTR1, frmctr1, sizeof(frmctr1), 0U) != 0) {
        return -1;
    }
    if (icst7735_write_cmd_data(lcd, ICST7735_CMD_FRMCTR2, frmctr2, sizeof(frmctr2), 0U) != 0) {
        return -1;
    }
    if (icst7735_write_cmd_data(lcd, ICST7735_CMD_FRMCTR3, frmctr3, sizeof(frmctr3), 0U) != 0) {
        return -1;
    }
    if (icst7735_write_cmd_data(lcd, ICST7735_CMD_INVCTR, invctr, sizeof(invctr), 0U) != 0) {
        return -1;
    }
    if (icst7735_write_cmd_data(lcd, ICST7735_CMD_PWCTR1, pwctr1, sizeof(pwctr1), 0U) != 0) {
        return -1;
    }
    if (icst7735_write_cmd_data(lcd, ICST7735_CMD_PWCTR2, pwctr2, sizeof(pwctr2), 0U) != 0) {
        return -1;
    }
    if (icst7735_write_cmd_data(lcd, ICST7735_CMD_PWCTR3, pwctr3, sizeof(pwctr3), 0U) != 0) {
        return -1;
    }
    if (icst7735_write_cmd_data(lcd, ICST7735_CMD_PWCTR4, pwctr4, sizeof(pwctr4), 0U) != 0) {
        return -1;
    }
    if (icst7735_write_cmd_data(lcd, ICST7735_CMD_PWCTR5, pwctr5, sizeof(pwctr5), 0U) != 0) {
        return -1;
    }
    if (icst7735_write_cmd_data(lcd, ICST7735_CMD_VMCTR1, vmctr1, sizeof(vmctr1), 0U) != 0) {
        return -1;
    }
    if (icst7735_write_cmd_data(lcd, ICST7735_CMD_COLMOD, colmod, sizeof(colmod), 10U) != 0) {
        return -1;
    }
    if (icst7735_write_cmd_data(lcd, ICST7735_CMD_GMCTRP1, gmctrp1, sizeof(gmctrp1), 0U) != 0) {
        return -1;
    }
    if (icst7735_write_cmd_data(lcd, ICST7735_CMD_GMCTRN1, gmctrn1, sizeof(gmctrn1), 0U) != 0) {
        return -1;
    }
    if (icst7735_write_cmd_data(lcd, ICST7735_CMD_NORON, NULL, 0U, 10U) != 0) {
        return -1;
    }
    if (icst7735_write_cmd_data(lcd, ICST7735_CMD_DISPON, NULL, 0U, 100U) != 0) {
        return -1;
    }

    return 0;
}

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
                  uint16_t y_offset)
{
    int dc_inited = 0;
    int rst_inited = 0;
    int bl_inited = 0;
    int spi_inited = 0;

    if (lcd == NULL || dc_pin == PIN_INVALID || width == 0U || height == 0U ||
        spi_speed_hz == 0U || spi_port >= LS_SPI_INVALID) {
        return -1;
    }

    memset(lcd, 0, sizeof(*lcd));
    lcd->native_width = width;
    lcd->native_height = height;
    lcd->width = width;
    lcd->height = height;
    lcd->base_x_offset = x_offset;
    lcd->base_y_offset = y_offset;
    lcd->x_offset = x_offset;
    lcd->y_offset = y_offset;
    lcd->rotation = ICST7735_ROTATION_0;
    lcd->has_reset_pin = (rst_pin != PIN_INVALID) ? 1 : 0;
    lcd->has_backlight_pin = (bl_pin != PIN_INVALID) ? 1 : 0;

    if (ls2k0300_gpio_init(&lcd->dc_gpio, dc_pin, GPIO_MODE_OUT, GPIO_MUX_GPIO) != 0) {
        goto err_out;
    }
    dc_inited = 1;
    ls2k0300_gpio_level_set(&lcd->dc_gpio, GPIO_HIGH);

    if (lcd->has_reset_pin) {
        if (ls2k0300_gpio_init(&lcd->rst_gpio, rst_pin, GPIO_MODE_OUT, GPIO_MUX_GPIO) != 0) {
            goto err_out;
        }
        rst_inited = 1;
        ls2k0300_gpio_level_set(&lcd->rst_gpio, GPIO_HIGH);
    }

    if (lcd->has_backlight_pin) {
        if (ls2k0300_gpio_init(&lcd->bl_gpio, bl_pin, GPIO_MODE_OUT, GPIO_MUX_GPIO) != 0) {
            goto err_out;
        }
        bl_inited = 1;
        ls2k0300_gpio_level_set(&lcd->bl_gpio, GPIO_LOW);
    }

    if (ls2k0300_spi_init(&lcd->spi, spi_port, spi_speed_hz, LS_SPI_MODE_0) != 0) {
        goto err_out;
    }
    spi_inited = 1;
    lcd->initialized = 1;

    if (icst7735_hw_reset(lcd) != 0) {
        goto err_out;
    }
    if (icst7735_init_sequence(lcd) != 0) {
        goto err_out;
    }
    if (icst7735_set_rotation(lcd, ICST7735_CFG_ROTATION) != 0) {
        goto err_out;
    }
    if (icst7735_set_backlight(lcd, 1) != 0) {
        goto err_out;
    }

    return 0;

err_out:
    lcd->initialized = 0;
    if (spi_inited) {
        ls2k0300_spi_deinit(&lcd->spi);
    }
    if (bl_inited) {
        ls2k0300_gpio_deinit(&lcd->bl_gpio);
    }
    if (rst_inited) {
        ls2k0300_gpio_deinit(&lcd->rst_gpio);
    }
    if (dc_inited) {
        ls2k0300_gpio_deinit(&lcd->dc_gpio);
    }
    memset(lcd, 0, sizeof(*lcd));
    return -1;
}

/********************************************************************************
 * @brief   释放 ST7735 驱动资源.
 * @param   lcd : 驱动句柄.
 * @return  none.
 ********************************************************************************/
void icst7735_deinit(icst7735_t *lcd)
{
    if (lcd == NULL) {
        return;
    }

    if (lcd->has_backlight_pin) {
        ls2k0300_gpio_level_set(&lcd->bl_gpio, GPIO_LOW);
    }

    ls2k0300_spi_deinit(&lcd->spi);

    if (lcd->has_backlight_pin) {
        ls2k0300_gpio_deinit(&lcd->bl_gpio);
    }
    if (lcd->has_reset_pin) {
        ls2k0300_gpio_deinit(&lcd->rst_gpio);
    }
    ls2k0300_gpio_deinit(&lcd->dc_gpio);

    memset(lcd, 0, sizeof(*lcd));
    printf("[INFO] deinit lcd.\n");
}

/********************************************************************************
 * @brief   执行面板硬件复位序列.
 * @param   lcd : 驱动句柄.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icst7735_hw_reset(icst7735_t *lcd)
{
    if (lcd == NULL || lcd->initialized == 0) {
        return -1;
    }

    if (lcd->has_reset_pin) {
        ls2k0300_gpio_level_set(&lcd->rst_gpio, GPIO_HIGH);
        icst7735_delay_ms(5U);
        ls2k0300_gpio_level_set(&lcd->rst_gpio, GPIO_LOW);
        icst7735_delay_ms(20U);
        ls2k0300_gpio_level_set(&lcd->rst_gpio, GPIO_HIGH);
        icst7735_delay_ms(120U);
        return 0;
    }

    return icst7735_write_cmd_data(lcd, ICST7735_CMD_SWRESET, NULL, 0U, 150U);
}

/********************************************************************************
 * @brief   控制面板背光 GPIO.
 * @param   lcd : 驱动句柄.
 * @param   on  : 0 表示关闭，非 0 表示开启.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icst7735_set_backlight(icst7735_t *lcd, int on)
{
    if (lcd == NULL || lcd->initialized == 0) {
        return -1;
    }

    if (!lcd->has_backlight_pin) {
        return 0;
    }

    ls2k0300_gpio_level_set(&lcd->bl_gpio, (on != 0) ? GPIO_HIGH : GPIO_LOW);
    return 0;
}

/********************************************************************************
 * @brief   设置屏幕旋转方向.
 * @param   lcd      : 驱动句柄.
 * @param   rotation : 旋转模式.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icst7735_set_rotation(icst7735_t *lcd, icst7735_rotation_t rotation)
{
    if (lcd == NULL || lcd->initialized == 0) {
        return -1;
    }
    return icst7735_apply_rotation(lcd, rotation);
}

/********************************************************************************
 * @brief   设置当前绘制窗口.
 * @param   lcd : 驱动句柄.
 * @param   x0  : 窗口起始 X.
 * @param   y0  : 窗口起始 Y.
 * @param   x1  : 窗口结束 X.
 * @param   y1  : 窗口结束 Y.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icst7735_set_window(icst7735_t *lcd, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint16_t xs;
    uint16_t xe;
    uint16_t ys;
    uint16_t ye;
    uint8_t data[4];

    if (lcd == NULL || lcd->initialized == 0) {
        return -1;
    }
    if (x0 > x1 || y0 > y1) {
        return -1;
    }
    if (x1 >= lcd->width || y1 >= lcd->height) {
        return -1;
    }

    xs = (uint16_t)(x0 + lcd->x_offset);
    xe = (uint16_t)(x1 + lcd->x_offset);
    ys = (uint16_t)(y0 + lcd->y_offset);
    ye = (uint16_t)(y1 + lcd->y_offset);

    data[0] = (uint8_t)(xs >> 8);
    data[1] = (uint8_t)(xs & 0xFFU);
    data[2] = (uint8_t)(xe >> 8);
    data[3] = (uint8_t)(xe & 0xFFU);
    if (icst7735_write_cmd_data(lcd, ICST7735_CMD_CASET, data, sizeof(data), 0U) != 0) {
        return -1;
    }

    data[0] = (uint8_t)(ys >> 8);
    data[1] = (uint8_t)(ys & 0xFFU);
    data[2] = (uint8_t)(ye >> 8);
    data[3] = (uint8_t)(ye & 0xFFU);
    if (icst7735_write_cmd_data(lcd, ICST7735_CMD_RASET, data, sizeof(data), 0U) != 0) {
        return -1;
    }

    return icst7735_write_cmd_data(lcd, ICST7735_CMD_RAMWR, NULL, 0U, 0U);
}

/********************************************************************************
 * @brief   绘制一个 RGB565 像素点.
 * @param   lcd   : 驱动句柄.
 * @param   x     : X 坐标.
 * @param   y     : Y 坐标.
 * @param   color : RGB565 颜色值.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icst7735_draw_pixel(icst7735_t *lcd, uint16_t x, uint16_t y, uint16_t color)
{
    uint8_t pixel[2];

    if (lcd == NULL || lcd->initialized == 0) {
        return -1;
    }
    if (x >= lcd->width || y >= lcd->height) {
        return -1;
    }

    if (icst7735_set_window(lcd, x, y, x, y) != 0) {
        return -1;
    }

    pixel[0] = (uint8_t)(color >> 8);
    pixel[1] = (uint8_t)(color & 0xFFU);
    return icst7735_write_data(lcd, pixel, sizeof(pixel));
}

/********************************************************************************
 * @brief   绘制单个英文/数字字符（5x7 字模）.
 * @param   lcd      : 驱动句柄.
 * @param   x        : 字符左上角 X 坐标.
 * @param   y        : 字符左上角 Y 坐标.
 * @param   ch       : 字符.
 * @param   color    : 前景色.
 * @param   bg_color : 背景色.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icst7735_draw_char(icst7735_t *lcd,
                       uint16_t x,
                       uint16_t y,
                       char ch,
                       uint16_t color,
                       uint16_t bg_color)
{
    uint16_t char_pixels[ICST7735_CHAR_WIDTH * ICST7735_CHAR_HEIGHT];
    uint8_t glyph[ICST7735_FONT5X7_WIDTH];
    uint16_t row;
    uint16_t col;

    if (lcd == NULL || lcd->initialized == 0) {
        return -1;
    }
    if (x >= lcd->width || y >= lcd->height) {
        return -1;
    }
    if ((uint32_t)x + ICST7735_CHAR_WIDTH > (uint32_t)lcd->width ||
        (uint32_t)y + ICST7735_CHAR_HEIGHT > (uint32_t)lcd->height) {
        return -1;
    }

    icst7735_get_glyph5x7(ch, glyph);

    for (row = 0U; row < ICST7735_CHAR_HEIGHT; ++row) {
        for (col = 0U; col < ICST7735_CHAR_WIDTH; ++col) {
            uint16_t pixel = bg_color;

            if (row < ICST7735_FONT5X7_HEIGHT && col < ICST7735_FONT5X7_WIDTH) {
                if ((glyph[col] & (uint8_t)(1U << row)) != 0U) {
                    pixel = color;
                }
            }

            char_pixels[(size_t)row * ICST7735_CHAR_WIDTH + col] = pixel;
        }
    }

    return icst7735_draw_rgb565(lcd, x, y, ICST7735_CHAR_WIDTH, ICST7735_CHAR_HEIGHT, char_pixels);
}

/********************************************************************************
 * @brief   绘制英文/数字字符串（自动换行）.
 * @param   lcd      : 驱动句柄.
 * @param   x        : 起始 X 坐标.
 * @param   y        : 起始 Y 坐标.
 * @param   str      : 字符串.
 * @param   color    : 前景色.
 * @param   bg_color : 背景色.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icst7735_draw_string(icst7735_t *lcd,
                         uint16_t x,
                         uint16_t y,
                         const char *str,
                         uint16_t color,
                         uint16_t bg_color)
{
    uint16_t cursor_x;
    uint16_t cursor_y;
    const char *p;

    if (lcd == NULL || lcd->initialized == 0 || str == NULL) {
        return -1;
    }
    if (x >= lcd->width || y >= lcd->height) {
        return -1;
    }

    cursor_x = x;
    cursor_y = y;
    p = str;

    while (*p != '\0') {
        char ch = *p;

        if (ch == '\r') {
            p++;
            continue;
        }

        if (ch == '\n') {
            cursor_x = x;
            cursor_y = (uint16_t)(cursor_y + ICST7735_CHAR_HEIGHT);
            if ((uint32_t)cursor_y + ICST7735_CHAR_HEIGHT > (uint32_t)lcd->height) {
                return -1;
            }
            p++;
            continue;
        }

        if ((uint32_t)cursor_x + ICST7735_CHAR_WIDTH > (uint32_t)lcd->width) {
            cursor_x = x;
            cursor_y = (uint16_t)(cursor_y + ICST7735_CHAR_HEIGHT);
        }

        if ((uint32_t)cursor_y + ICST7735_CHAR_HEIGHT > (uint32_t)lcd->height) {
            return -1;
        }

        if (icst7735_draw_char(lcd, cursor_x, cursor_y, ch, color, bg_color) != 0) {
            return -1;
        }

        cursor_x = (uint16_t)(cursor_x + ICST7735_CHAR_WIDTH);
        p++;
    }

    return 0;
}

/********************************************************************************
 * @brief   清屏（使用黑色填充全屏）.
 * @param   lcd : 驱动句柄.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icst7735_clear_screen(icst7735_t *lcd)
{
    return icst7735_fill_color(lcd, ICST7735_COLOR565(0U, 0U, 0U));
}

/********************************************************************************
 * @brief   使用单一 RGB565 颜色值填充全屏.
 * @param   lcd   : 驱动句柄.
 * @param   color : RGB565 颜色值.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icst7735_fill_color(icst7735_t *lcd, uint16_t color)
{
    size_t total_pixels;
    size_t chunk_pixels;
    uint8_t tx_buf[ICST7735_TX_CHUNK_PIXELS * 2U];
    size_t i;

    if (lcd == NULL || lcd->initialized == 0) {
        return -1;
    }

    if (icst7735_set_window(lcd, 0U, 0U, (uint16_t)(lcd->width - 1U), (uint16_t)(lcd->height - 1U)) != 0) {
        return -1;
    }

    for (i = 0U; i < ICST7735_TX_CHUNK_PIXELS; i++) {
        tx_buf[2U * i] = (uint8_t)(color >> 8);
        tx_buf[2U * i + 1U] = (uint8_t)(color & 0xFFU);
    }

    total_pixels = (size_t)lcd->width * (size_t)lcd->height;
    while (total_pixels > 0U) {
        chunk_pixels = (total_pixels > ICST7735_TX_CHUNK_PIXELS) ? ICST7735_TX_CHUNK_PIXELS : total_pixels;
        if (icst7735_write_data(lcd, tx_buf, chunk_pixels * 2U) != 0) {
            return -1;
        }
        total_pixels -= chunk_pixels;
    }

    return 0;
}

/********************************************************************************
 * @brief   在目标区域绘制 RGB565 图像块.
 * @param   lcd    : 驱动句柄.
 * @param   x      : 目标起始 X.
 * @param   y      : 目标起始 Y.
 * @param   w      : 图像块宽度.
 * @param   h      : 图像块高度.
 * @param   pixels : RGB565 像素缓冲区，按行连续存储.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icst7735_draw_rgb565(icst7735_t *lcd, uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *pixels)
{
    size_t remain_pixels;
    size_t chunk_pixels;
    uint8_t tx_buf[ICST7735_TX_CHUNK_PIXELS * 2U];
    const uint16_t *cur;
    size_t i;

    if (lcd == NULL || lcd->initialized == 0 || pixels == NULL || w == 0U || h == 0U) {
        return -1;
    }
    if ((uint32_t)x + (uint32_t)w > (uint32_t)lcd->width || (uint32_t)y + (uint32_t)h > (uint32_t)lcd->height) {
        return -1;
    }

    if (icst7735_set_window(lcd, x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U)) != 0) {
        return -1;
    }

    cur = pixels;
    remain_pixels = (size_t)w * (size_t)h;
    while (remain_pixels > 0U) {
        chunk_pixels = (remain_pixels > ICST7735_TX_CHUNK_PIXELS) ? ICST7735_TX_CHUNK_PIXELS : remain_pixels;
        for (i = 0U; i < chunk_pixels; i++) {
            uint16_t c = cur[i];
            tx_buf[2U * i] = (uint8_t)(c >> 8);
            tx_buf[2U * i + 1U] = (uint8_t)(c & 0xFFU);
        }
        if (icst7735_write_data(lcd, tx_buf, chunk_pixels * 2U) != 0) {
            return -1;
        }
        cur += chunk_pixels;
        remain_pixels -= chunk_pixels;
    }

    return 0;
}

/********************************************************************************
 * @brief   在目标区域绘制已按 LCD 字节序排列的 RGB565 图像块.
 * @param   lcd       : 驱动句柄.
 * @param   x         : 目标起始 X.
 * @param   y         : 目标起始 Y.
 * @param   w         : 图像块宽度.
 * @param   h         : 图像块高度.
 * @param   pixels_be : RGB565 字节流，高字节在前，按行连续存储.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icst7735_draw_rgb565_bytes(icst7735_t *lcd,
                               uint16_t x,
                               uint16_t y,
                               uint16_t w,
                               uint16_t h,
                               const uint8_t *pixels_be)
{
    size_t remain_bytes;
    size_t chunk_bytes;
    const uint8_t *cur;

    if (lcd == NULL || lcd->initialized == 0 || pixels_be == NULL || w == 0U || h == 0U) {
        return -1;
    }
    if ((uint32_t)x + (uint32_t)w > (uint32_t)lcd->width || (uint32_t)y + (uint32_t)h > (uint32_t)lcd->height) {
        return -1;
    }

    if (icst7735_set_window(lcd, x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U)) != 0) {
        return -1;
    }

    cur = pixels_be;
    remain_bytes = (size_t)w * (size_t)h * 2U;
    while (remain_bytes > 0U) {
        chunk_bytes = remain_bytes;
        if (chunk_bytes > (ICST7735_TX_CHUNK_PIXELS * 2U)) {
            chunk_bytes = ICST7735_TX_CHUNK_PIXELS * 2U;
        }

        if (icst7735_write_data(lcd, cur, chunk_bytes) != 0) {
            return -1;
        }

        cur += chunk_bytes;
        remain_bytes -= chunk_bytes;
    }

    return 0;
}

/********************************************************************************
 * @brief   将 BGR565 摄像头图像以彩色方式全屏显示.
 * @param   lcd                 : 驱动句柄.
 * @param   frame_bgr565        : 输入 BGR565 图像缓冲区.
 * @param   frame_width         : 源图像宽度.
 * @param   frame_height        : 源图像高度.
 * @param   frame_stride_pixels : 源图像行步长（像素），传 0 表示 width.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icst7735_show_camera_bgr565(icst7735_t *lcd,
                                const uint16_t *frame_bgr565,
                                uint16_t frame_width,
                                uint16_t frame_height,
                                uint32_t frame_stride_pixels)
{
    uint8_t tx_buf[ICST7735_TX_CHUNK_PIXELS * 2U];
    uint16_t dst_w;
    uint16_t dst_h;
    size_t total_pixels;
    size_t done_pixels;
    size_t chunk_pixels;
    size_t i;

    if (lcd == NULL || lcd->initialized == 0 || frame_bgr565 == NULL ||
        frame_width == 0U || frame_height == 0U) {
        return -1;
    }

    if (frame_stride_pixels == 0U) {
        frame_stride_pixels = frame_width;
    }
    if (frame_stride_pixels < frame_width) {
        return -1;
    }

    dst_w = lcd->width;
    dst_h = lcd->height;

    if (icst7735_set_window(lcd, 0U, 0U, (uint16_t)(dst_w - 1U), (uint16_t)(dst_h - 1U)) != 0) {
        return -1;
    }

    total_pixels = (size_t)dst_w * (size_t)dst_h;
    done_pixels = 0U;
    while (done_pixels < total_pixels) {
        chunk_pixels = total_pixels - done_pixels;
        if (chunk_pixels > ICST7735_TX_CHUNK_PIXELS) {
            chunk_pixels = ICST7735_TX_CHUNK_PIXELS;
        }

        for (i = 0U; i < chunk_pixels; i++) {
            size_t dst_index = done_pixels + i;
            uint32_t dst_x = (uint32_t)(dst_index % (size_t)dst_w);
            uint32_t dst_y = (uint32_t)(dst_index / (size_t)dst_w);
            uint32_t src_x = (dst_x * (uint32_t)frame_width) / (uint32_t)dst_w;
            uint32_t src_y = (dst_y * (uint32_t)frame_height) / (uint32_t)dst_h;
            const uint16_t *src_row = frame_bgr565 + src_y * frame_stride_pixels;
            uint16_t color565 = src_row[src_x];

            tx_buf[2U * i] = (uint8_t)(color565 >> 8);
            tx_buf[2U * i + 1U] = (uint8_t)(color565 & 0xFFU);
        }

        if (icst7735_write_data(lcd, tx_buf, chunk_pixels * 2U) != 0) {
            return -1;
        }

        done_pixels += chunk_pixels;
    }

    return 0;
}

/********************************************************************************
 * @brief   将 BGR888 摄像头图像以彩色方式全屏显示.
 * @param   lcd                : 驱动句柄.
 * @param   frame_bgr888       : 输入 BGR888 图像缓冲区.
 * @param   frame_width        : 源图像宽度.
 * @param   frame_height       : 源图像高度.
 * @param   frame_stride_bytes : 源图像行步长（字节），传 0 表示 width*3.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icst7735_show_camera_bgr888(icst7735_t *lcd,
                                const uint8_t *frame_bgr888,
                                uint16_t frame_width,
                                uint16_t frame_height,
                                uint32_t frame_stride_bytes)
{
    uint8_t tx_buf[ICST7735_TX_CHUNK_PIXELS * 2U];
    uint16_t dst_w;
    uint16_t dst_h;
    uint16_t y;
    uint16_t x;
    size_t chunk_pixels;
    size_t i;

    if (lcd == NULL || lcd->initialized == 0 || frame_bgr888 == NULL ||
        frame_width == 0U || frame_height == 0U) {
        return -1;
    }

    if (frame_stride_bytes == 0U) {
        frame_stride_bytes = (uint32_t)frame_width * 3U;
    }
    if (frame_stride_bytes < (uint32_t)frame_width * 3U) {
        return -1;
    }

    dst_w = lcd->width;
    dst_h = lcd->height;

    if (icst7735_set_window(lcd, 0U, 0U, (uint16_t)(dst_w - 1U), (uint16_t)(dst_h - 1U)) != 0) {
        return -1;
    }

    for (y = 0U; y < dst_h; y++) {
        uint32_t src_y = ((uint32_t)y * (uint32_t)frame_height) / (uint32_t)dst_h;
        const uint8_t *src_row = frame_bgr888 + src_y * frame_stride_bytes;

        x = 0U;
        while (x < dst_w) {
            chunk_pixels = (size_t)(dst_w - x);
            if (chunk_pixels > ICST7735_TX_CHUNK_PIXELS) {
                chunk_pixels = ICST7735_TX_CHUNK_PIXELS;
            }

            for (i = 0U; i < chunk_pixels; i++) {
                uint32_t src_x = ((uint32_t)(x + (uint16_t)i) * (uint32_t)frame_width) / (uint32_t)dst_w;
                const uint8_t *p = src_row + src_x * 3U;
                uint16_t color565 = ICST7735_COLOR565(p[0], p[1], p[2]);

                tx_buf[2U * i] = (uint8_t)(color565 >> 8);
                tx_buf[2U * i + 1U] = (uint8_t)(color565 & 0xFFU);
            }

            if (icst7735_write_data(lcd, tx_buf, chunk_pixels * 2U) != 0) {
                return -1;
            }

            x = (uint16_t)(x + (uint16_t)chunk_pixels);
        }
    }

    return 0;
}

static uint8_t icst7735_clip_u8(int32_t value)
{
    if (value < 0) {
        return 0U;
    }
    if (value > 255) {
        return 255U;
    }
    return (uint8_t)value;
}

static uint16_t icst7735_yuyv_to_lcd565(uint8_t y, uint8_t u, uint8_t v)
{
    int32_t c = (int32_t)y - 16;
    int32_t d = (int32_t)u - 128;
    int32_t e = (int32_t)v - 128;
    uint8_t r;
    uint8_t g;
    uint8_t b;

    if (c < 0) {
        c = 0;
    }

    r = icst7735_clip_u8((298 * c + 409 * e + 128) >> 8);
    g = icst7735_clip_u8((298 * c - 100 * d - 208 * e + 128) >> 8);
    b = icst7735_clip_u8((298 * c + 516 * d + 128) >> 8);

    return ICST7735_COLOR565(b, g, r);
}

/********************************************************************************
 * @brief   将 YUYV 摄像头图像以彩色方式全屏显示.
 * @param   lcd                : 驱动句柄.
 * @param   frame_yuyv        : 输入 YUYV/YUY2 图像缓冲区，字节顺序为 Y0 U Y1 V.
 * @param   frame_width        : 源图像宽度，需为偶数.
 * @param   frame_height       : 源图像高度.
 * @param   frame_stride_bytes : 源图像行步长（字节），传 0 表示 width*2.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icst7735_show_camera_yuyv(icst7735_t *lcd,
                              const uint8_t *frame_yuyv,
                              uint16_t frame_width,
                              uint16_t frame_height,
                              uint32_t frame_stride_bytes)
{
    uint8_t tx_buf[ICST7735_TX_CHUNK_PIXELS * 2U];
    uint16_t dst_w;
    uint16_t dst_h;
    uint16_t y;
    uint16_t x;
    size_t chunk_pixels;
    size_t i;

    if (lcd == NULL || lcd->initialized == 0 || frame_yuyv == NULL ||
        frame_width < 2U || frame_height == 0U || ((frame_width & 1U) != 0U)) {
        return -1;
    }

    if (frame_stride_bytes == 0U) {
        frame_stride_bytes = (uint32_t)frame_width * 2U;
    }
    if (frame_stride_bytes < (uint32_t)frame_width * 2U) {
        return -1;
    }

    dst_w = lcd->width;
    dst_h = lcd->height;

    if (icst7735_set_window(lcd, 0U, 0U, (uint16_t)(dst_w - 1U), (uint16_t)(dst_h - 1U)) != 0) {
        return -1;
    }

    for (y = 0U; y < dst_h; y++) {
        uint32_t src_y = ((uint32_t)y * (uint32_t)frame_height) / (uint32_t)dst_h;
        const uint8_t *src_row = frame_yuyv + src_y * frame_stride_bytes;

        x = 0U;
        while (x < dst_w) {
            chunk_pixels = (size_t)(dst_w - x);
            if (chunk_pixels > ICST7735_TX_CHUNK_PIXELS) {
                chunk_pixels = ICST7735_TX_CHUNK_PIXELS;
            }

            for (i = 0U; i < chunk_pixels; i++) {
                uint32_t src_x = ((uint32_t)(x + (uint16_t)i) * (uint32_t)frame_width) / (uint32_t)dst_w;
                const uint8_t *p = src_row + ((src_x & ~1U) * 2U);
                uint8_t y_value = ((src_x & 1U) == 0U) ? p[0] : p[2];
                uint16_t color565 = icst7735_yuyv_to_lcd565(y_value, p[1], p[3]);

                tx_buf[2U * i] = (uint8_t)(color565 >> 8);
                tx_buf[2U * i + 1U] = (uint8_t)(color565 & 0xFFU);
            }

            if (icst7735_write_data(lcd, tx_buf, chunk_pixels * 2U) != 0) {
                return -1;
            }

            x = (uint16_t)(x + (uint16_t)chunk_pixels);
        }
    }

    return 0;
}

/********************************************************************************
 * @brief   将 BGR888 摄像头图像以灰度方式全屏显示.
 * @param   lcd                : 驱动句柄.
 * @param   frame_bgr888       : 输入 BGR888 图像缓冲区.
 * @param   frame_width        : 源图像宽度.
 * @param   frame_height       : 源图像高度.
 * @param   frame_stride_bytes : 源图像行步长（字节），传 0 表示 width*3.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icst7735_show_camera_gray_bgr888(icst7735_t *lcd,
                                     const uint8_t *frame_bgr888,
                                     uint16_t frame_width,
                                     uint16_t frame_height,
                                     uint32_t frame_stride_bytes)
{
    uint8_t tx_buf[ICST7735_TX_CHUNK_PIXELS * 2U];
    uint16_t dst_w;
    uint16_t dst_h;
    uint16_t y;
    uint16_t x;
    size_t chunk_pixels;
    size_t i;

    if (lcd == NULL || lcd->initialized == 0 || frame_bgr888 == NULL ||
        frame_width == 0U || frame_height == 0U) {
        return -1;
    }

    if (frame_stride_bytes == 0U) {
        frame_stride_bytes = (uint32_t)frame_width * 3U;
    }
    if (frame_stride_bytes < (uint32_t)frame_width * 3U) {
        return -1;
    }

    dst_w = lcd->width;
    dst_h = lcd->height;

    if (icst7735_set_window(lcd, 0U, 0U, (uint16_t)(dst_w - 1U), (uint16_t)(dst_h - 1U)) != 0) {
        return -1;
    }

    for (y = 0U; y < dst_h; y++) {
        uint32_t src_y = ((uint32_t)y * (uint32_t)frame_height) / (uint32_t)dst_h;
        const uint8_t *src_row = frame_bgr888 + src_y * frame_stride_bytes;

        x = 0U;
        while (x < dst_w) {
            chunk_pixels = (size_t)(dst_w - x);
            if (chunk_pixels > ICST7735_TX_CHUNK_PIXELS) {
                chunk_pixels = ICST7735_TX_CHUNK_PIXELS;
            }

            for (i = 0U; i < chunk_pixels; i++) {
                uint32_t src_x = ((uint32_t)(x + (uint16_t)i) * (uint32_t)frame_width) / (uint32_t)dst_w;
                const uint8_t *p = src_row + src_x * 3U;
                uint8_t gray = (uint8_t)(((uint16_t)(p[2] * 77U) + (uint16_t)(p[1] * 150U) + (uint16_t)(p[0] * 29U)) >> 8);
                uint16_t color565 = ICST7735_COLOR565(gray, gray, gray);

                tx_buf[2U * i] = (uint8_t)(color565 >> 8);
                tx_buf[2U * i + 1U] = (uint8_t)(color565 & 0xFFU);
            }

            if (icst7735_write_data(lcd, tx_buf, chunk_pixels * 2U) != 0) {
                return -1;
            }

            x = (uint16_t)(x + (uint16_t)chunk_pixels);
        }
    }

    return 0;
}

/********************************************************************************
 * @brief   将 YUYV 摄像头图像以灰度方式全屏显示.
 * @param   lcd                : 驱动句柄.
 * @param   frame_yuyv        : 输入 YUYV/YUY2 图像缓冲区，字节顺序为 Y0 U Y1 V.
 * @param   frame_width        : 源图像宽度，需为偶数.
 * @param   frame_height       : 源图像高度.
 * @param   frame_stride_bytes : 源图像行步长（字节），传 0 表示 width*2.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icst7735_show_camera_gray_yuyv(icst7735_t *lcd,
                                   const uint8_t *frame_yuyv,
                                   uint16_t frame_width,
                                   uint16_t frame_height,
                                   uint32_t frame_stride_bytes)
{
    uint8_t tx_buf[ICST7735_TX_CHUNK_PIXELS * 2U];
    uint16_t dst_w;
    uint16_t dst_h;
    uint16_t y;
    uint16_t x;
    size_t chunk_pixels;
    size_t i;

    if (lcd == NULL || lcd->initialized == 0 || frame_yuyv == NULL ||
        frame_width < 2U || frame_height == 0U || ((frame_width & 1U) != 0U)) {
        return -1;
    }

    if (frame_stride_bytes == 0U) {
        frame_stride_bytes = (uint32_t)frame_width * 2U;
    }
    if (frame_stride_bytes < (uint32_t)frame_width * 2U) {
        return -1;
    }

    dst_w = lcd->width;
    dst_h = lcd->height;

    if (icst7735_set_window(lcd, 0U, 0U, (uint16_t)(dst_w - 1U), (uint16_t)(dst_h - 1U)) != 0) {
        return -1;
    }

    for (y = 0U; y < dst_h; y++) {
        uint32_t src_y = ((uint32_t)y * (uint32_t)frame_height) / (uint32_t)dst_h;
        const uint8_t *src_row = frame_yuyv + src_y * frame_stride_bytes;

        x = 0U;
        while (x < dst_w) {
            chunk_pixels = (size_t)(dst_w - x);
            if (chunk_pixels > ICST7735_TX_CHUNK_PIXELS) {
                chunk_pixels = ICST7735_TX_CHUNK_PIXELS;
            }

            for (i = 0U; i < chunk_pixels; i++) {
                uint32_t src_x = ((uint32_t)(x + (uint16_t)i) * (uint32_t)frame_width) / (uint32_t)dst_w;
                const uint8_t *p = src_row + ((src_x & ~1U) * 2U);
                uint8_t gray = ((src_x & 1U) == 0U) ? p[0] : p[2];
                uint16_t color565 = ICST7735_COLOR565(gray, gray, gray);

                tx_buf[2U * i] = (uint8_t)(color565 >> 8);
                tx_buf[2U * i + 1U] = (uint8_t)(color565 & 0xFFU);
            }

            if (icst7735_write_data(lcd, tx_buf, chunk_pixels * 2U) != 0) {
                return -1;
            }

            x = (uint16_t)(x + (uint16_t)chunk_pixels);
        }
    }

    return 0;
}

/********************************************************************************
 * @brief   打开或关闭面板显示.
 * @param   lcd : 驱动句柄.
 * @param   on  : 0 表示关闭，非 0 表示开启.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icst7735_display_on(icst7735_t *lcd, int on)
{
    if (lcd == NULL || lcd->initialized == 0) {
        return -1;
    }
    return icst7735_write_cmd_data(lcd, (on != 0) ? ICST7735_CMD_DISPON : ICST7735_CMD_DISPOFF, NULL, 0U, 50U);
}

/********************************************************************************
 * @brief   使能或关闭显示反色.
 * @param   lcd    : 驱动句柄.
 * @param   enable : 0 为正常显示，非 0 为反色显示.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int icst7735_invert_display(icst7735_t *lcd, int enable)
{
    if (lcd == NULL || lcd->initialized == 0) {
        return -1;
    }
    return icst7735_write_cmd_data(lcd, (enable != 0) ? ICST7735_CMD_INVON : ICST7735_CMD_INVOFF, NULL, 0U, 0U);
}
