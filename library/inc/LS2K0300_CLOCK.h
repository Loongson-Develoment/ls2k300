#ifndef __LS2K0300_CLOCK_H
#define __LS2K0300_CLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************************
 * @brief   PMON 时钟频率统一配置.
 * @note    当前工程统一按 160MHz 使用，不做运行时切换.
 * @example uint32_t tick = (uint32_t)(LS_PMON_CLOCK_FREQ / 1000L);
 ********************************************************************************/
#define LS_PMON_CLOCK_FREQ  (160000000L)

#ifdef __cplusplus
}
#endif

#endif
