#ifndef __LS2K0300_SIGNAL_HANDLE_H
#define __LS2K0300_SIGNAL_HANDLE_H

#include <signal.h>

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************************
 * @brief   退出回调函数类型.
 ********************************************************************************/
typedef void (*ls2k0300_signal_exit_cb_t)(void);

/********************************************************************************
 * @brief   设置 SIGINT 退出回调.
 * @param   cb : 回调函数.
 * @return  none.
 * @example ls2k0300_signal_set_exit_cb(app_cleanup);
 ********************************************************************************/
void ls2k0300_signal_set_exit_cb(ls2k0300_signal_exit_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif
