#include "LS2K0300_TIMER.h"

#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sched.h>

/********************************************************************************
 * @brief   纳秒级休眠函数.
 * @param   ns : 休眠时长（纳秒）.
 * @return  none.
 * @note    长延时走 nanosleep，短延时走忙等提升精度.
 ********************************************************************************/
static void timer_sleep_ns(uint64_t ns)
{
    const uint64_t threshold = 100ULL * 1000ULL;

    if (ns == 0ULL) {
        return;
    }

    if (ns > threshold) {
        struct timespec req;
        req.tv_sec = (time_t)(ns / 1000000000ULL);
        req.tv_nsec = (long)(ns % 1000000000ULL);
        /* 大于阈值直接交给内核休眠 */
        (void)clock_nanosleep(CLOCK_MONOTONIC, 0, &req, NULL);
    } else {
        struct timespec now;
        struct timespec end;

        clock_gettime(CLOCK_MONOTONIC, &now);
        end.tv_sec = now.tv_sec + (time_t)(ns / 1000000000ULL);
        end.tv_nsec = now.tv_nsec + (long)(ns % 1000000000ULL);
        if (end.tv_nsec >= 1000000000L) {
            end.tv_sec += 1;
            end.tv_nsec -= 1000000000L;
        }

        /* 小于阈值采用忙等，尽量降低调度抖动 */
        while (1) {
            clock_gettime(CLOCK_MONOTONIC, &now);
            if ((now.tv_sec > end.tv_sec) ||
                (now.tv_sec == end.tv_sec && now.tv_nsec >= end.tv_nsec)) {
                break;
            }
            sched_yield();
        }
    }
}

/********************************************************************************
 * @brief   定时器线程主循环.
 * @param   arg : ls2k0300_timer_t 指针.
 * @return  线程退出时返回 NULL.
 ********************************************************************************/
static void *timer_thread_entry(void *arg)
{
    ls2k0300_timer_t *timer = (ls2k0300_timer_t *)arg;

    while (1) {
        uint64_t interval;
        ls2k0300_timer_callback_t cb;
        void *user_data;
        struct timespec t_start;
        struct timespec t_end;
        uint64_t used_ns;

        pthread_mutex_lock(&timer->mutex);
        while (timer->running && !timer->timer_active) {
            pthread_cond_wait(&timer->cond, &timer->mutex);
        }

        if (!timer->running) {
            pthread_mutex_unlock(&timer->mutex);
            break;
        }

        /* 拷贝当前回调参数，减少持锁时间 */
        interval = timer->interval_ns;
        cb = timer->callback;
        user_data = timer->user_data;
        pthread_mutex_unlock(&timer->mutex);

        clock_gettime(CLOCK_MONOTONIC, &t_start);
        if (cb != NULL) {
            cb(user_data);
        }
        clock_gettime(CLOCK_MONOTONIC, &t_end);

        used_ns = (uint64_t)(t_end.tv_sec - t_start.tv_sec) * 1000000000ULL
                + (uint64_t)(t_end.tv_nsec - t_start.tv_nsec);

        if (used_ns < interval) {
            /* 回调耗时小于周期时补足剩余间隔 */
            timer_sleep_ns(interval - used_ns);
        }
    }

    return NULL;
}

/********************************************************************************
 * @brief   初始化定时器.
 * @param   timer : 定时器句柄指针.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_timer_init(&timer);
 ********************************************************************************/
int ls2k0300_timer_init(ls2k0300_timer_t *timer)
{
    if (timer == NULL) {
        return -1;
    }

    memset(timer, 0, sizeof(*timer));

    pthread_mutex_init(&timer->mutex, NULL);
    pthread_cond_init(&timer->cond, NULL);

    timer->running = true;
    timer->timer_active = false;
    timer->interval_ns = 0ULL;
    timer->callback = NULL;
    timer->user_data = NULL;

    if (pthread_create(&timer->thread, NULL, timer_thread_entry, timer) != 0) {
        pthread_mutex_destroy(&timer->mutex);
        pthread_cond_destroy(&timer->cond);
        return -1;
    }

    timer->initialized = true;
    return 0;
}

/********************************************************************************
 * @brief   释放定时器.
 * @param   timer : 定时器句柄指针.
 * @return  none.
 * @example ls2k0300_timer_deinit(&timer);
 ********************************************************************************/
void ls2k0300_timer_deinit(ls2k0300_timer_t *timer)
{
    if (timer == NULL || !timer->initialized) {
        return;
    }

    pthread_mutex_lock(&timer->mutex);
    /* 通知线程退出并唤醒阻塞等待 */
    timer->running = false;
    timer->timer_active = false;
    timer->callback = NULL;
    timer->user_data = NULL;
    pthread_cond_broadcast(&timer->cond);
    pthread_mutex_unlock(&timer->mutex);

    pthread_join(timer->thread, NULL);

    pthread_mutex_destroy(&timer->mutex);
    pthread_cond_destroy(&timer->cond);

    memset(timer, 0, sizeof(*timer));
}

/********************************************************************************
 * @brief   设置秒级定时周期.
 * @param   timer     : 定时器句柄指针.
 * @param   s         : 周期秒数.
 * @param   cb        : 回调函数.
 * @param   user_data : 回调上下文.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_timer_set_seconds_s(&timer, 1, timer_cb, NULL);
 ********************************************************************************/
int ls2k0300_timer_set_seconds_s(ls2k0300_timer_t *timer, uint64_t s, ls2k0300_timer_callback_t cb, void *user_data)
{
    if (timer == NULL || !timer->initialized || s == 0ULL) {
        return -1;
    }

    /* 激活周期任务并广播唤醒工作线程 */
    pthread_mutex_lock(&timer->mutex);
    timer->interval_ns = s * 1000000000ULL;
    timer->callback = cb;
    timer->user_data = user_data;
    timer->timer_active = true;
    pthread_cond_broadcast(&timer->cond);
    pthread_mutex_unlock(&timer->mutex);

    return 0;
}

/********************************************************************************
 * @brief   设置毫秒级定时周期.
 * @param   timer     : 定时器句柄指针.
 * @param   ms        : 周期毫秒数.
 * @param   cb        : 回调函数.
 * @param   user_data : 回调上下文.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_timer_set_seconds_ms(&timer, 10, timer_cb, NULL);
 ********************************************************************************/
int ls2k0300_timer_set_seconds_ms(ls2k0300_timer_t *timer, uint64_t ms, ls2k0300_timer_callback_t cb, void *user_data)
{
    if (timer == NULL || !timer->initialized || ms == 0ULL) {
        return -1;
    }

    pthread_mutex_lock(&timer->mutex);
    timer->interval_ns = ms * 1000000ULL;
    timer->callback = cb;
    timer->user_data = user_data;
    timer->timer_active = true;
    pthread_cond_broadcast(&timer->cond);
    pthread_mutex_unlock(&timer->mutex);

    return 0;
}

/********************************************************************************
 * @brief   停止定时器线程.
 * @param   timer : 定时器句柄指针.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_timer_stop(&timer);
 ********************************************************************************/
int ls2k0300_timer_stop(ls2k0300_timer_t *timer)
{
    if (timer == NULL || !timer->initialized) {
        return -1;
    }

    /* 只关闭调度标志，不在此处 join 线程 */
    pthread_mutex_lock(&timer->mutex);
    timer->running = false;
    timer->timer_active = false;
    timer->callback = NULL;
    timer->user_data = NULL;
    pthread_cond_broadcast(&timer->cond);
    pthread_mutex_unlock(&timer->mutex);

    return 0;
}

/********************************************************************************
 * @brief   查询定时器是否运行.
 * @param   timer : 定时器句柄指针.
 * @return  true 表示运行，false 表示未运行.
 * @example bool running = ls2k0300_timer_is_running(&timer);
 ********************************************************************************/
bool ls2k0300_timer_is_running(ls2k0300_timer_t *timer)
{
    bool running;

    if (timer == NULL || !timer->initialized) {
        return false;
    }

    pthread_mutex_lock(&timer->mutex);
    running = timer->running;
    pthread_mutex_unlock(&timer->mutex);

    return running;
}
