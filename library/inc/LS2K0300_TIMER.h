#ifndef __LS2K0300_TIMER_H
#define __LS2K0300_TIMER_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************************
 * @brief   定时器回调类型.
 ********************************************************************************/
typedef void (*ls2k0300_timer_callback_t)(void *user_data);

/********************************************************************************
 * @brief   定时器句柄结构体.
 ********************************************************************************/
typedef struct {
    pthread_mutex_t              mutex;
    pthread_cond_t               cond;
    pthread_t                    thread;
    bool                         running;
    bool                         timer_active;
    uint64_t                     interval_ns;
    ls2k0300_timer_callback_t    callback;
    void                        *user_data;
    bool                         initialized;
} ls2k0300_timer_t;

/********************************************************************************
 * @brief   初始化软件定时器线程.
 * @param   timer : 定时器句柄指针.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_timer_init(&timer);
 ********************************************************************************/
int ls2k0300_timer_init(ls2k0300_timer_t *timer);

/********************************************************************************
 * @brief   释放软件定时器资源.
 * @param   timer : 定时器句柄指针.
 * @return  none.
 * @example ls2k0300_timer_deinit(&timer);
 ********************************************************************************/
void ls2k0300_timer_deinit(ls2k0300_timer_t *timer);

/********************************************************************************
 * @brief   按秒设置周期回调.
 * @param   timer     : 定时器句柄指针.
 * @param   s         : 周期秒数.
 * @param   cb        : 周期回调函数.
 * @param   user_data : 回调上下文.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_timer_set_seconds_s(&timer, 1, timer_cb, NULL);
 ********************************************************************************/
int ls2k0300_timer_set_seconds_s(ls2k0300_timer_t *timer, uint64_t s, ls2k0300_timer_callback_t cb, void *user_data);

/********************************************************************************
 * @brief   按毫秒设置周期回调.
 * @param   timer     : 定时器句柄指针.
 * @param   ms        : 周期毫秒数.
 * @param   cb        : 周期回调函数.
 * @param   user_data : 回调上下文.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_timer_set_seconds_ms(&timer, 10, timer_cb, NULL);
 ********************************************************************************/
int ls2k0300_timer_set_seconds_ms(ls2k0300_timer_t *timer, uint64_t ms, ls2k0300_timer_callback_t cb, void *user_data);

/********************************************************************************
 * @brief   停止定时器调度.
 * @param   timer : 定时器句柄指针.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_timer_stop(&timer);
 ********************************************************************************/
int ls2k0300_timer_stop(ls2k0300_timer_t *timer);

/********************************************************************************
 * @brief   查询定时器是否处于运行状态.
 * @param   timer : 定时器句柄指针.
 * @return  true 表示运行，false 表示未运行或无效句柄.
 * @example bool running = ls2k0300_timer_is_running(&timer);
 ********************************************************************************/
bool ls2k0300_timer_is_running(ls2k0300_timer_t *timer);

#ifdef __cplusplus
}
#endif

#endif
