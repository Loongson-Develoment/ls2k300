#ifndef __LS2K0300_DRV_INC_H
#define __LS2K0300_DRV_INC_H

/********************************************************************************
 * @brief   LS2K0300 驱动总入口头文件.
 * @note    应用层包含该头文件即可一次性获得常用驱动声明.
 * @example #include "LS2K0300_DRV_INC.h"
 ********************************************************************************/

#include "LS2K0300_CLOCK.h"
#include "LS2K0300_MAP.h"

#include "LS2K0300_GPIO.h"
#include "LS2K0300_ADC.h"
#include "LS2K0300_PWM.h"
#include "LS2K0300_SPI.h"
#include "LS2K0300_UART.h"
#include "LS2K0300_I2C.h"
#include "LS2K0300_HW_I2C.h"
#include "LS2K0300_CANFD.h"

#include "LS2K0300_SOFT_SPI.h"
#include "LS2K0300_ATIM_PWM.h"
#include "LS2K0300_GTIM_PWM.h"
#include "LS2K0300_PWM_ENCODER.h"
#include "LS2K0300_TIMER.h"
// #include "LS2K0300_SIGNAL_HANDLE.h"
#include "LS2K0300_THREADPOOL.h"

#include "ICST7735.h"
#include "ICSSD1306.h"
#include "X42_V2.h"

#endif
