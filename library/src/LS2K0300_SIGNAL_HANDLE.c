#include "LS2K0300_SIGNAL_HANDLE.h"

#include <unistd.h>
#include <stdlib.h>

static ls2k0300_signal_exit_cb_t g_exit_cb = NULL;

/********************************************************************************
 * @brief   SIGINT 信号处理函数.
 * @param   sig : 信号编号.
 * @return  none.
 * @note    先执行用户清理回调，再统一打印退出提示并结束进程.
 ********************************************************************************/
static void ls2k0300_sigint_handler(int sig)
{
    const char msg[] = "\n[LS2K0300] [EXIT] Ctrl+C safe exit\n";

    (void)sig;

    if (g_exit_cb != NULL) {
        /* 回调中可释放 GPIO/UART/CANFD 等资源 */
        g_exit_cb();
    }

    (void)write(STDOUT_FILENO, msg, sizeof(msg) - 1U);
    exit(0);
}

/********************************************************************************
 * @brief   自动注册 SIGINT 处理函数.
 * @return  none.
 * @note    通过 constructor 在 main 之前自动完成注册.
 ********************************************************************************/
__attribute__((constructor))
static void ls2k0300_auto_setup_signal(void)
{
    struct sigaction sa;

    sa.sa_handler = ls2k0300_sigint_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);
}

/********************************************************************************
 * @brief   设置 SIGINT 退出回调.
 * @param   cb : 退出回调函数.
 * @return  none.
 * @example ls2k0300_signal_set_exit_cb(app_cleanup);
 ********************************************************************************/
void ls2k0300_signal_set_exit_cb(ls2k0300_signal_exit_cb_t cb)
{
    g_exit_cb = cb;
}
