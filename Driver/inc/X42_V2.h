#ifndef _X42_V2_H_
#define _X42_V2_H_

#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "LS2K0300_UART.h"
#include "LS2K0300_CANFD.h"

#define		ABS(x)		((x) > 0 ? (x) : -(x)) 

typedef enum {
  S_VER   = 0,      /* 读取固件版本和对应的硬件版本 */
  S_RL    = 1,      /* 读取读取相电阻和相电感 */
  S_PID   = 2,      /* 读取PID参数 */
  S_ORG   = 3,      /* 读取回零参数 */
  S_VBUS  = 4,      /* 读取总线电压 */
  S_CBUS  = 5,      /* 读取总线电流 */
  S_CPHA  = 6,      /* 读取相电流 */
  S_ENC   = 7,      /* 读取编码器原始值 */
  S_CPUL  = 8,      /* 读取实时脉冲数（根据实时位置计算得到的脉冲数） */
  S_ENCL  = 9,      /* 读取经过线性化校准后的编码器值 */
  S_TPUL  = 10,     /* 读取输入脉冲数 */
  S_TPOS  = 11,     /* 读取电机目标位置 */
  S_OPOS  = 12,     /* 读取电机实时设定的目标位置（开环模式的实时位置） */
  S_VEL   = 13,     /* 读取电机实时转速 */
  S_CPOS  = 14,     /* 读取电机实时位置（基于角度编码器累加的电机实时位置） */
  S_PERR  = 15,     /* 读取电机位置误差 */
  S_TEMP  = 16,     /* 读取电机实时温度 */
  S_SFLAG = 17,     /* 读取状态标志位 */
  S_OFLAG = 18,     /* 读取回零状态标志位 */
  S_Conf  = 19,     /* 读取驱动参数 */
  S_State = 20,     /* 读取系统状态参数 */
}SysParams_t;

void ZDT_X42_V2_Reset_CurPos_To_Zero(ls2k0300_uart_t *uart, uint8_t addr); // 将当前位置清零
void ZDT_X42_V2_Reset_Clog_Pro(ls2k0300_uart_t *uart, uint8_t addr); // 解除堵转保护
void ZDT_X42_V2_Read_Sys_Params(ls2k0300_uart_t *uart, uint8_t addr, SysParams_t s); // 读取参数
void ZDT_X42_V2_Modify_Ctrl_Mode(ls2k0300_uart_t *uart, uint8_t addr, bool svF, uint8_t ctrl_mode); // 发送命令切换开环/闭环控制模式
void ZDT_X42_V2_En_Control(ls2k0300_uart_t *uart, uint8_t addr, bool state, uint8_t snF); // 电机使能控制
void ZDT_X42_V2_Torque_Control(ls2k0300_uart_t *uart, uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, uint8_t snF); // 力矩模式控制
void ZDT_X42_V2_Velocity_Control(ls2k0300_uart_t *uart, uint8_t addr, uint8_t dir, uint16_t v_ramp, float velocity, uint8_t snF); // 速度模式控制
void ZDT_X42_V2_Bypass_Position_LV_Control(ls2k0300_uart_t *uart, uint8_t addr, uint8_t dir, float velocity, float position, uint8_t raf, uint8_t snF); // 直通限速位置模式控制
void ZDT_X42_V2_Traj_Position_Control(ls2k0300_uart_t *uart, uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float velocity, float position, uint8_t raf, uint8_t snF); // 梯形曲线加减速位置模式控制
void ZDT_X42_V2_Stop_Now(ls2k0300_uart_t *uart, uint8_t addr, uint8_t snF); // 让电机立即停止运动
void ZDT_X42_V2_Synchronous_motion(ls2k0300_uart_t *uart, uint8_t addr); // 触发多机同步开始运动
void ZDT_X42_V2_Origin_Set_O(ls2k0300_uart_t *uart, uint8_t addr, bool svF); // 设置单圈回零的零点位置
void ZDT_X42_V2_Origin_Modify_Params(ls2k0300_uart_t *uart, uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF); // 修改回零参数
void ZDT_X42_V2_Origin_Trigger_Return(ls2k0300_uart_t *uart, uint8_t addr, uint8_t o_mode, bool snF); // 发送命令触发回零
void ZDT_X42_V2_Origin_Interrupt(ls2k0300_uart_t *uart, uint8_t addr); // 强制中断并退出回零
// void ZDT_X42_V2_Receive_Data(ls2k0300_uart_t *uart, uint8_t *rxCmd, uint8_t *rxCount); // 返回数据接收函数
static int can_send_cmd(ls2k0300_canfd_t *canfd, const uint8_t *cmd, uint8_t len);




#ifdef __cplusplus
}
#endif  



#endif