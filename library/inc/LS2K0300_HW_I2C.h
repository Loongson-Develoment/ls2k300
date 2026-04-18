#ifndef __LS2K0300_HW_I2C_H
#define __LS2K0300_HW_I2C_H

#include <stdint.h>
#include <stddef.h>
#include <pthread.h>

#include "LS2K0300_MAP.h"

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************************
 * @brief   硬件 I2C 端口枚举.
 ********************************************************************************/
typedef enum ls_hw_i2c_port {
    LS_HW_I2C0 = 0x00,
    LS_HW_I2C1,
    LS_HW_I2C2,
    LS_HW_I2C3,
    LS_HW_I2C_INVALID,
} ls_hw_i2c_port_t;

/********************************************************************************
 * @brief   硬件 I2C 复用组枚举.
 * @note    每个 I2C 端口支持 2 组复用引脚（m0/m1）.
 ********************************************************************************/
typedef enum ls_hw_i2c_mux {
    LS_HW_I2C_MUX_M0 = 0x00,
    LS_HW_I2C_MUX_M1,
    LS_HW_I2C_MUX_INVALID,
} ls_hw_i2c_mux_t;

/********************************************************************************
 * @brief   硬件 I2C 句柄结构体.
 ********************************************************************************/
typedef struct {
    ls_hw_i2c_port_t port;
    ls_hw_i2c_mux_t  mux;
    uint8_t          addr;
    uint32_t         speed_hz;
    uint32_t         input_clk_hz;
    uint32_t         timeout_us;

    ls_reg32_addr_t  i2c_base;
    ls_reg32_addr_t  i2c_cr1;
    ls_reg32_addr_t  i2c_cr2;
    ls_reg32_addr_t  i2c_oar;
    ls_reg32_addr_t  i2c_dr;
    ls_reg32_addr_t  i2c_sr1;
    ls_reg32_addr_t  i2c_sr2;
    ls_reg32_addr_t  i2c_ccr;
    ls_reg32_addr_t  i2c_trise;

    pthread_mutex_t  mtx;
    int              initialized;
} ls2k0300_hw_i2c_t;

/********************************************************************************
 * @brief   默认输入时钟频率（用于时序计算）.
 * @note    取值可按板级 APB/I2C 输入时钟调整.
 ********************************************************************************/
#define LS_HW_I2C_DEFAULT_INPUT_CLK_HZ  (160000000U)

/********************************************************************************
 * @brief   默认步骤超时时间（微秒）.
 ********************************************************************************/
#define LS_HW_I2C_DEFAULT_TIMEOUT_US     (20000U)

/********************************************************************************
 * @brief   初始化硬件 I2C 控制器（主模式，寄存器直控）.
 * @note    默认使用每路端口的 m0 复用组；需要指定 m1 时请使用 init_ex.
 * @param   i2c      : I2C 句柄.
 * @param   port     : I2C 端口.
 * @param   addr     : 7bit 从设备地址.
 * @param   speed_hz : 目标总线速率（推荐 100000 或 400000）.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_hw_i2c_t hi2c;
 *          ls2k0300_hw_i2c_init(&hi2c, LS_HW_I2C1, 0x50, 100000);
 ********************************************************************************/
int ls2k0300_hw_i2c_init(ls2k0300_hw_i2c_t *i2c, ls_hw_i2c_port_t port, uint8_t addr, uint32_t speed_hz);

/********************************************************************************
 * @brief   初始化硬件 I2C 控制器（可显式选择复用组）.
 * @param   i2c      : I2C 句柄.
 * @param   port     : I2C 端口.
 * @param   mux      : I2C 复用组（m0/m1）.
 * @param   addr     : 7bit 从设备地址.
 * @param   speed_hz : 目标总线速率（推荐 100000 或 400000）.
 * @return  成功返回 0，失败返回 -1.
 * @note    复用引脚映射关系（来自 pinctrl）：
 *          I2C0-m0: PIN48/49, m1: PIN60/61
 *          I2C1-m0: PIN50/51, m1: PIN62/63
 *          I2C2-m0: PIN52/53, m1: PIN82/83
 *          I2C3-m0: PIN54/55, m1: PIN84/85
 * @example ls2k0300_hw_i2c_t hi2c;
 *          ls2k0300_hw_i2c_init_ex(&hi2c, LS_HW_I2C2, LS_HW_I2C_MUX_M1, 0x50, 400000);
 ********************************************************************************/
int ls2k0300_hw_i2c_init_ex(ls2k0300_hw_i2c_t *i2c, ls_hw_i2c_port_t port,
                            ls_hw_i2c_mux_t mux, uint8_t addr, uint32_t speed_hz);

/********************************************************************************
 * @brief   释放硬件 I2C 资源.
 * @param   i2c : I2C 句柄.
 * @return  none.
 * @example ls2k0300_hw_i2c_deinit(&hi2c);
 ********************************************************************************/
void ls2k0300_hw_i2c_deinit(ls2k0300_hw_i2c_t *i2c);

/********************************************************************************
 * @brief   更新从设备地址.
 * @param   i2c  : I2C 句柄.
 * @param   addr : 7bit 从设备地址.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int ls2k0300_hw_i2c_set_addr(ls2k0300_hw_i2c_t *i2c, uint8_t addr);

/********************************************************************************
 * @brief   动态更新硬件 I2C 速率.
 * @param   i2c      : I2C 句柄.
 * @param   speed_hz : 目标速率（Hz）.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int ls2k0300_hw_i2c_set_speed(ls2k0300_hw_i2c_t *i2c, uint32_t speed_hz);

/********************************************************************************
 * @brief   执行一次总线恢复命令（9 个 SCL + STOP）.
 * @param   i2c : I2C 句柄.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int ls2k0300_hw_i2c_recover(ls2k0300_hw_i2c_t *i2c);

/********************************************************************************
 * @brief   读取单字节寄存器.
 * @param   i2c : I2C 句柄.
 * @param   reg : 寄存器地址.
 * @param   val : 输出数据指针.
 * @return  成功返回 0，失败返回 1.
 * @example uint8_t id = 0;
 *          ls2k0300_hw_i2c_read_byte(&hi2c, 0x00, &id);
 ********************************************************************************/
uint8_t ls2k0300_hw_i2c_read_byte(ls2k0300_hw_i2c_t *i2c, uint8_t reg, uint8_t *val);

/********************************************************************************
 * @brief   写入单字节寄存器.
 * @param   i2c : I2C 句柄.
 * @param   reg : 寄存器地址.
 * @param   val : 写入值.
 * @return  成功返回 0，失败返回 1.
 * @example ls2k0300_hw_i2c_write_byte(&hi2c, 0x10, 0x5A);
 ********************************************************************************/
uint8_t ls2k0300_hw_i2c_write_byte(ls2k0300_hw_i2c_t *i2c, uint8_t reg, uint8_t val);

/********************************************************************************
 * @brief   连续读取多个寄存器字节.
 * @param   i2c : I2C 句柄.
 * @param   reg : 起始寄存器.
 * @param   buf : 输出缓冲区.
 * @param   len : 读取长度.
 * @return  成功返回 0，失败返回 1.
 * @note    采用“单字节随机读 + 地址递增”策略，稳定性优先.
 * @example uint8_t buf[6];
 *          ls2k0300_hw_i2c_read_n_byte(&hi2c, 0x20, buf, 6);
 ********************************************************************************/
uint8_t ls2k0300_hw_i2c_read_n_byte(ls2k0300_hw_i2c_t *i2c, uint8_t reg, uint8_t *buf, uint16_t len);

/********************************************************************************
 * @brief   连续写入多个寄存器字节.
 * @param   i2c : I2C 句柄.
 * @param   reg : 起始寄存器.
 * @param   buf : 输入缓冲区.
 * @param   len : 写入长度.
 * @return  成功返回 0，失败返回 1.
 * @example uint8_t cfg[2] = {0x01, 0x02};
 *          ls2k0300_hw_i2c_write_n_byte(&hi2c, 0x30, cfg, 2);
 ********************************************************************************/
uint8_t ls2k0300_hw_i2c_write_n_byte(ls2k0300_hw_i2c_t *i2c, uint8_t reg, const uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
