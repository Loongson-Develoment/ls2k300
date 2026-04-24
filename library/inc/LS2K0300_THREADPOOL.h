#ifndef _LS2K0300_THREADPOOL_H
#define _LS2K0300_THREADPOOL_H

#include <stddef.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************************
 * @brief   线程池任务回调类型.
 * @param   arg : 用户参数.
 * @return  none.
 ********************************************************************************/
typedef void (*ls2k0300_threadpool_task_fn_t)(void *arg);

/********************************************************************************
 * @brief   线程池任务节点.
 ********************************************************************************/
typedef struct ls2k0300_threadpool_task {
    ls2k0300_threadpool_task_fn_t   task_fn;
    void                           *task_arg;
    struct ls2k0300_threadpool_task *next;
} ls2k0300_threadpool_task_t;

/********************************************************************************
 * @brief   线程池句柄.
 ********************************************************************************/
typedef struct {
    pthread_t                   *workers;
    size_t                       worker_num;
    ls2k0300_threadpool_task_t *task_head;
    ls2k0300_threadpool_task_t *task_tail;
    size_t                       working_cnt;
    int                          stop;
    int                          initialized;
    pthread_mutex_t              mtx;
    pthread_cond_t               cond_task;
    pthread_cond_t               cond_idle;
} ls2k0300_threadpool_t;

/********************************************************************************
 * @brief   初始化线程池.
 * @param   pool       : 线程池句柄.
 * @param   worker_num : 工作线程数量（>0）.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int ls2k0300_threadpool_init(ls2k0300_threadpool_t *pool, size_t worker_num);

/********************************************************************************
 * @brief   向线程池提交任务.
 * @param   pool     : 线程池句柄.
 * @param   task_fn  : 任务函数.
 * @param   task_arg : 任务参数.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int ls2k0300_threadpool_add_task(ls2k0300_threadpool_t *pool,
                                 ls2k0300_threadpool_task_fn_t task_fn, void *task_arg);

/********************************************************************************
 * @brief   等待当前已提交任务执行完毕.
 * @param   pool : 线程池句柄.
 * @return  none.
 ********************************************************************************/
void ls2k0300_threadpool_wait(ls2k0300_threadpool_t *pool);

/********************************************************************************
 * @brief   释放线程池资源.
 * @param   pool : 线程池句柄.
 * @param   force_destroy_flag : 0 表示处理完队列后再退出，非 0 表示丢弃队列中未执行任务.
 * @return  none.
 ********************************************************************************/
void ls2k0300_threadpool_destroy(ls2k0300_threadpool_t *pool, int force_destroy_flag);

/********************************************************************************
 * @brief   兼容接口：submit.
 ********************************************************************************/
int ls2k0300_threadpool_submit(ls2k0300_threadpool_t *pool,
                               ls2k0300_threadpool_task_fn_t task_fn, void *task_arg);

/********************************************************************************
 * @brief   兼容接口：deinit（等价 destroy(force_destroy_flag=0)）.
 ********************************************************************************/
void ls2k0300_threadpool_deinit(ls2k0300_threadpool_t *pool);

#ifdef __cplusplus
}
#endif

#endif /* _LS2K0300_THREADPOOL_H */
