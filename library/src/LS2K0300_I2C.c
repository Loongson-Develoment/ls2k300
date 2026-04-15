#include "LS2K0300_I2C.h"

#include <unistd.h>

/********************************************************************************
 * @brief   I2C 微秒级延时.
 * @param   us : 延时微秒.
 * @return  none.
 ********************************************************************************/
static void soft_i2c_delay_us(uint16_t us)
{
    usleep(us);
}

/********************************************************************************
 * @brief   产生 I2C 起始信号.
 * @param   i2c : I2C 句柄.
 * @return  none.
 ********************************************************************************/
static void soft_i2c_start(ls2k0300_i2c_t *i2c)
{
    /* 空闲状态下 SDA/SCL 为高，起始条件为 SCL 高电平时 SDA 下降沿 */
    ls2k0300_gpio_direction_set(&i2c->sda, GPIO_MODE_OUT);
    ls2k0300_gpio_level_set(&i2c->sda, GPIO_HIGH);
    ls2k0300_gpio_level_set(&i2c->scl, GPIO_HIGH);
    soft_i2c_delay_us(2);
    soft_i2c_delay_us(2);
    ls2k0300_gpio_level_set(&i2c->sda, GPIO_LOW);
    soft_i2c_delay_us(2);
    ls2k0300_gpio_level_set(&i2c->scl, GPIO_LOW);
}

/********************************************************************************
 * @brief   产生 I2C 停止信号.
 * @param   i2c : I2C 句柄.
 * @return  none.
 ********************************************************************************/
static void soft_i2c_stop(ls2k0300_i2c_t *i2c)
{
    /* 停止条件为 SCL 高电平时 SDA 上升沿 */
    ls2k0300_gpio_direction_set(&i2c->sda, GPIO_MODE_OUT);
    soft_i2c_delay_us(2);
    ls2k0300_gpio_level_set(&i2c->scl, GPIO_LOW);
    ls2k0300_gpio_level_set(&i2c->sda, GPIO_LOW);
    soft_i2c_delay_us(2);
    ls2k0300_gpio_level_set(&i2c->scl, GPIO_HIGH);
    soft_i2c_delay_us(2);
    ls2k0300_gpio_level_set(&i2c->sda, GPIO_HIGH);
    soft_i2c_delay_us(2);
}

/********************************************************************************
 * @brief   等待从机 ACK.
 * @param   i2c : I2C 句柄.
 * @return  0 表示收到 ACK，1 表示超时失败.
 ********************************************************************************/
static uint8_t soft_i2c_wait_ack(ls2k0300_i2c_t *i2c)
{
    uint8_t err_time = 0;

    ls2k0300_gpio_direction_set(&i2c->sda, GPIO_MODE_IN);
    ls2k0300_gpio_level_set(&i2c->sda, GPIO_HIGH);
    soft_i2c_delay_us(2);
    ls2k0300_gpio_level_set(&i2c->scl, GPIO_HIGH);
    soft_i2c_delay_us(2);

    while (ls2k0300_gpio_level_get(&i2c->sda) == GPIO_HIGH) {
        err_time++;
        if (err_time > 100) {
            soft_i2c_stop(i2c);
            return 1;
        }
    }

    ls2k0300_gpio_level_set(&i2c->scl, GPIO_LOW);
    return 0;
}

/********************************************************************************
 * @brief   主机发送 ACK.
 * @param   i2c : I2C 句柄.
 * @return  none.
 ********************************************************************************/
static void soft_i2c_ack(ls2k0300_i2c_t *i2c)
{
    ls2k0300_gpio_level_set(&i2c->scl, GPIO_LOW);
    ls2k0300_gpio_direction_set(&i2c->sda, GPIO_MODE_OUT);
    ls2k0300_gpio_level_set(&i2c->sda, GPIO_LOW);
    soft_i2c_delay_us(2);
    ls2k0300_gpio_level_set(&i2c->scl, GPIO_HIGH);
    soft_i2c_delay_us(2);
    ls2k0300_gpio_level_set(&i2c->scl, GPIO_LOW);
}

/********************************************************************************
 * @brief   主机发送 NACK.
 * @param   i2c : I2C 句柄.
 * @return  none.
 ********************************************************************************/
static void soft_i2c_nack(ls2k0300_i2c_t *i2c)
{
    ls2k0300_gpio_level_set(&i2c->scl, GPIO_LOW);
    ls2k0300_gpio_direction_set(&i2c->sda, GPIO_MODE_OUT);
    ls2k0300_gpio_level_set(&i2c->sda, GPIO_HIGH);
    soft_i2c_delay_us(2);
    ls2k0300_gpio_level_set(&i2c->scl, GPIO_HIGH);
    soft_i2c_delay_us(2);
    ls2k0300_gpio_level_set(&i2c->scl, GPIO_LOW);
}

/********************************************************************************
 * @brief   发送一个字节到 I2C 总线.
 * @param   i2c  : I2C 句柄.
 * @param   byte : 待发送字节.
 * @return  none.
 ********************************************************************************/
static void soft_i2c_send_byte(ls2k0300_i2c_t *i2c, uint8_t byte)
{
    uint8_t t;

    ls2k0300_gpio_direction_set(&i2c->sda, GPIO_MODE_OUT);
    ls2k0300_gpio_level_set(&i2c->scl, GPIO_LOW);

    for (t = 0; t < 8; t++) {
        /* 先输出数据位，再拉高时钟进行采样 */
        ls2k0300_gpio_level_set(&i2c->sda, (byte & 0x80U) ? GPIO_HIGH : GPIO_LOW);
        ls2k0300_gpio_level_set(&i2c->scl, GPIO_HIGH);
        soft_i2c_delay_us(2);
        byte <<= 1;
        soft_i2c_delay_us(2);
        ls2k0300_gpio_level_set(&i2c->scl, GPIO_LOW);
        soft_i2c_delay_us(2);
    }

    soft_i2c_delay_us(2);
}

/********************************************************************************
 * @brief   从 I2C 总线读取一个字节.
 * @param   i2c : I2C 句柄.
 * @param   ack : 1 发送 ACK，0 发送 NACK.
 * @return  读取到的字节值.
 ********************************************************************************/
static uint8_t soft_i2c_read_byte_internal(ls2k0300_i2c_t *i2c, uint8_t ack)
{
    uint8_t i;
    uint8_t receive = 0;

    ls2k0300_gpio_direction_set(&i2c->sda, GPIO_MODE_IN);

    for (i = 0; i < 8; i++) {
        ls2k0300_gpio_level_set(&i2c->scl, GPIO_LOW);
        soft_i2c_delay_us(2);
        ls2k0300_gpio_level_set(&i2c->scl, GPIO_HIGH);

        receive <<= 1;
        if (ls2k0300_gpio_level_get(&i2c->sda) == GPIO_HIGH) {
            receive++;
        }

        soft_i2c_delay_us(2);
    }

    if (ack != 0U) {
        soft_i2c_ack(i2c);
    } else {
        soft_i2c_nack(i2c);
    }

    return receive;
}

/********************************************************************************
 * @brief   初始化软件 I2C.
 * @param   i2c  : I2C 句柄.
 * @param   scl  : 时钟引脚.
 * @param   sda  : 数据引脚.
 * @param   addr : 7bit 设备地址.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_i2c_t i2c;
 *          ls2k0300_i2c_init(&i2c, PIN_85, PIN_86, 0x50);
 ********************************************************************************/
int ls2k0300_i2c_init(ls2k0300_i2c_t *i2c, gpio_pin_t scl, gpio_pin_t sda, uint8_t addr)
{
    if (i2c == NULL) {
        return -1;
    }

    if (ls2k0300_gpio_init(&i2c->scl, scl, GPIO_MODE_OUT, GPIO_MUX_GPIO) != 0) {
        return -1;
    }
    if (ls2k0300_gpio_init(&i2c->sda, sda, GPIO_MODE_OUT, GPIO_MUX_GPIO) != 0) {
        ls2k0300_gpio_deinit(&i2c->scl);
        return -1;
    }

    i2c->addr = addr;
    i2c->initialized = 1;

    /* 总线空闲态保持高电平 */
    ls2k0300_gpio_level_set(&i2c->scl, GPIO_HIGH);
    ls2k0300_gpio_level_set(&i2c->sda, GPIO_HIGH);

    return 0;
}

/********************************************************************************
 * @brief   释放软件 I2C.
 * @param   i2c : I2C 句柄.
 * @return  none.
 * @example ls2k0300_i2c_deinit(&i2c);
 ********************************************************************************/
void ls2k0300_i2c_deinit(ls2k0300_i2c_t *i2c)
{
    if (i2c == NULL || i2c->initialized == 0) {
        return;
    }

    ls2k0300_gpio_deinit(&i2c->scl);
    ls2k0300_gpio_deinit(&i2c->sda);
    i2c->addr = 0xFFU;
    i2c->initialized = 0;
}

/********************************************************************************
 * @brief   读取指定寄存器单字节.
 * @param   i2c : I2C 句柄.
 * @param   reg : 寄存器地址.
 * @param   val : 输出值指针.
 * @return  成功返回 0，失败返回 1.
 * @example uint8_t id = 0;
 *          ls2k0300_i2c_read_byte(&i2c, 0x00, &id);
 ********************************************************************************/
uint8_t ls2k0300_i2c_read_byte(ls2k0300_i2c_t *i2c, uint8_t reg, uint8_t *val)
{
    if (i2c == NULL || val == NULL || i2c->initialized == 0) {
        return 1;
    }

    /* 写寄存器地址阶段 */
    soft_i2c_start(i2c);
    soft_i2c_send_byte(i2c, (uint8_t)(i2c->addr << 1));
    (void)soft_i2c_wait_ack(i2c);

    soft_i2c_send_byte(i2c, reg);
    (void)soft_i2c_wait_ack(i2c);

    /* 重启后进入读阶段 */
    soft_i2c_start(i2c);
    soft_i2c_send_byte(i2c, (uint8_t)((i2c->addr << 1) + 1U));
    (void)soft_i2c_wait_ack(i2c);

    *val = soft_i2c_read_byte_internal(i2c, 0);
    soft_i2c_stop(i2c);

    return 0;
}

/********************************************************************************
 * @brief   写入指定寄存器单字节.
 * @param   i2c : I2C 句柄.
 * @param   reg : 寄存器地址.
 * @param   val : 写入值.
 * @return  成功返回 0，失败返回 1.
 * @example ls2k0300_i2c_write_byte(&i2c, 0x10, 0x5A);
 ********************************************************************************/
uint8_t ls2k0300_i2c_write_byte(ls2k0300_i2c_t *i2c, uint8_t reg, uint8_t val)
{
    if (i2c == NULL || i2c->initialized == 0) {
        return 1;
    }

    soft_i2c_start(i2c);
    soft_i2c_send_byte(i2c, (uint8_t)(i2c->addr << 1));
    (void)soft_i2c_wait_ack(i2c);

    soft_i2c_send_byte(i2c, reg);
    (void)soft_i2c_wait_ack(i2c);

    soft_i2c_send_byte(i2c, val);
    (void)soft_i2c_wait_ack(i2c);

    soft_i2c_stop(i2c);
    return 0;
}

/********************************************************************************
 * @brief   连续读取多个寄存器字节.
 * @param   i2c : I2C 句柄.
 * @param   reg : 起始寄存器.
 * @param   buf : 输出缓冲区.
 * @param   len : 读取长度.
 * @return  成功返回 0，失败返回 1.
 * @example uint8_t buf[6];
 *          ls2k0300_i2c_read_n_byte(&i2c, 0x20, buf, 6);
 ********************************************************************************/
uint8_t ls2k0300_i2c_read_n_byte(ls2k0300_i2c_t *i2c, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;

    if (i2c == NULL || buf == NULL || len == 0U || i2c->initialized == 0) {
        return 1;
    }

    soft_i2c_start(i2c);
    soft_i2c_send_byte(i2c, (uint8_t)(i2c->addr << 1));
    (void)soft_i2c_wait_ack(i2c);

    soft_i2c_send_byte(i2c, reg);
    (void)soft_i2c_wait_ack(i2c);

    soft_i2c_start(i2c);
    soft_i2c_send_byte(i2c, (uint8_t)((i2c->addr << 1) + 1U));
    (void)soft_i2c_wait_ack(i2c);

    /* 最后一个字节需要回 NACK，通知从机停止发送 */
    for (i = 0; i < len; i++) {
        buf[i] = soft_i2c_read_byte_internal(i2c, (i == (uint8_t)(len - 1U)) ? 0U : 1U);
    }

    soft_i2c_stop(i2c);
    return 0;
}

/********************************************************************************
 * @brief   连续写入多个寄存器字节.
 * @param   i2c : I2C 句柄.
 * @param   reg : 起始寄存器.
 * @param   buf : 输入缓冲区.
 * @param   len : 写入长度.
 * @return  成功返回 0，失败返回 1.
 * @example uint8_t cfg[2] = {0x01, 0x02};
 *          ls2k0300_i2c_write_n_byte(&i2c, 0x30, cfg, 2);
 ********************************************************************************/
uint8_t ls2k0300_i2c_write_n_byte(ls2k0300_i2c_t *i2c, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;

    if (i2c == NULL || buf == NULL || len == 0U || i2c->initialized == 0) {
        return 1;
    }

    soft_i2c_start(i2c);
    soft_i2c_send_byte(i2c, (uint8_t)(i2c->addr << 1));
    (void)soft_i2c_wait_ack(i2c);

    soft_i2c_send_byte(i2c, reg);
    (void)soft_i2c_wait_ack(i2c);

    for (i = 0; i < len; i++) {
        soft_i2c_send_byte(i2c, buf[i]);
        (void)soft_i2c_wait_ack(i2c);
    }

    soft_i2c_stop(i2c);
    return 0;
}
