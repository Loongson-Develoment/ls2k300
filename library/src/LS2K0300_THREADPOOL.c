#include "LS2K0300_THREADPOOL.h"

#include <stdlib.h>
#include <string.h>

/********************************************************************************
 * @brief   释放任务队列中尚未执行的任务（需在加锁状态下调用）.
 * @param   pool : 线程池句柄.
 * @return  none.
 ********************************************************************************/
static void ls2k0300_threadpool_free_pending_tasks(ls2k0300_threadpool_t *pool)
{
    while (pool->task_head != NULL) {
        ls2k0300_threadpool_task_t *task = pool->task_head;
        pool->task_head = task->next;
        free(task);
    }
    pool->task_tail = NULL;
}

/********************************************************************************
 * @brief   线程池工作线程入口.
 * @param   arg : 线程池句柄.
 * @return  none.
 ********************************************************************************/
static void *ls2k0300_threadpool_worker(void *arg)
{
    ls2k0300_threadpool_t *pool = (ls2k0300_threadpool_t *)arg;

    for (;;) {
        ls2k0300_threadpool_task_t *task;

        pthread_mutex_lock(&pool->mtx);

        while ((pool->task_head == NULL) && (pool->stop == 0)) {
            pthread_cond_wait(&pool->cond_task, &pool->mtx);
        }

        if ((pool->stop != 0) && (pool->task_head == NULL)) {
            pthread_mutex_unlock(&pool->mtx);
            break;
        }

        task = pool->task_head;
        pool->task_head = task->next;
        if (pool->task_head == NULL) {
            pool->task_tail = NULL;
        }
        pool->working_cnt++;

        pthread_mutex_unlock(&pool->mtx);

        task->task_fn(task->task_arg);
        free(task);

        pthread_mutex_lock(&pool->mtx);
        pool->working_cnt--;
        if ((pool->task_head == NULL) && (pool->working_cnt == 0U)) {
            pthread_cond_broadcast(&pool->cond_idle);
        }
        pthread_mutex_unlock(&pool->mtx);
    }

    return NULL;
}

/********************************************************************************
 * @brief   初始化线程池.
 * @param   pool       : 线程池句柄.
 * @param   worker_num : 工作线程数量（>0）.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int ls2k0300_threadpool_init(ls2k0300_threadpool_t *pool, size_t worker_num)
{
    size_t i;

    if ((pool == NULL) || (worker_num == 0U)) {
        return -1;
    }

    memset(pool, 0, sizeof(*pool));

    if (pthread_mutex_init(&pool->mtx, NULL) != 0) {
        return -1;
    }
    if (pthread_cond_init(&pool->cond_task, NULL) != 0) {
        pthread_mutex_destroy(&pool->mtx);
        return -1;
    }
    if (pthread_cond_init(&pool->cond_idle, NULL) != 0) {
        pthread_cond_destroy(&pool->cond_task);
        pthread_mutex_destroy(&pool->mtx);
        return -1;
    }

    pool->workers = (pthread_t *)calloc(worker_num, sizeof(pthread_t));
    if (pool->workers == NULL) {
        pthread_cond_destroy(&pool->cond_idle);
        pthread_cond_destroy(&pool->cond_task);
        pthread_mutex_destroy(&pool->mtx);
        return -1;
    }

    pool->worker_num = worker_num;
    pool->stop = 0;

    for (i = 0U; i < worker_num; ++i) {
        if (pthread_create(&pool->workers[i], NULL, ls2k0300_threadpool_worker, pool) != 0) {
            size_t j;

            pthread_mutex_lock(&pool->mtx);
            pool->stop = 1;
            pthread_mutex_unlock(&pool->mtx);
            pthread_cond_broadcast(&pool->cond_task);

            for (j = 0U; j < i; ++j) {
                pthread_join(pool->workers[j], NULL);
            }

            free(pool->workers);
            pool->workers = NULL;
            pthread_cond_destroy(&pool->cond_idle);
            pthread_cond_destroy(&pool->cond_task);
            pthread_mutex_destroy(&pool->mtx);
            memset(pool, 0, sizeof(*pool));
            return -1;
        }
    }

    pool->initialized = 1;
    return 0;
}

/********************************************************************************
 * @brief   向线程池提交任务.
 * @param   pool     : 线程池句柄.
 * @param   task_fn  : 任务函数.
 * @param   task_arg : 任务参数.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int ls2k0300_threadpool_add_task(ls2k0300_threadpool_t *pool,
                                 ls2k0300_threadpool_task_fn_t task_fn, void *task_arg)
{
    ls2k0300_threadpool_task_t *task;

    if ((pool == NULL) || (task_fn == NULL)) {
        return -1;
    }

    task = (ls2k0300_threadpool_task_t *)malloc(sizeof(*task));
    if (task == NULL) {
        return -1;
    }

    task->task_fn = task_fn;
    task->task_arg = task_arg;
    task->next = NULL;

    pthread_mutex_lock(&pool->mtx);

    if ((pool->initialized == 0) || (pool->stop != 0)) {
        pthread_mutex_unlock(&pool->mtx);
        free(task);
        return -1;
    }

    if (pool->task_tail == NULL) {
        pool->task_head = task;
        pool->task_tail = task;
    } else {
        pool->task_tail->next = task;
        pool->task_tail = task;
    }

    pthread_cond_signal(&pool->cond_task);
    pthread_mutex_unlock(&pool->mtx);

    return 0;
}

/********************************************************************************
 * @brief   等待当前已提交任务执行完毕.
 * @param   pool : 线程池句柄.
 * @return  none.
 ********************************************************************************/
void ls2k0300_threadpool_wait(ls2k0300_threadpool_t *pool)
{
    if ((pool == NULL) || (pool->initialized == 0)) {
        return;
    }

    pthread_mutex_lock(&pool->mtx);
    while ((pool->task_head != NULL) || (pool->working_cnt != 0U)) {
        pthread_cond_wait(&pool->cond_idle, &pool->mtx);
    }
    pthread_mutex_unlock(&pool->mtx);
}

/********************************************************************************
 * @brief   释放线程池资源.
 * @param   pool : 线程池句柄.
 * @param   force_destroy_flag : 0 表示处理完队列后再退出，非 0 表示丢弃队列中未执行任务.
 * @return  none.
 ********************************************************************************/
void ls2k0300_threadpool_destroy(ls2k0300_threadpool_t *pool, int force_destroy_flag)
{
    size_t i;

    if ((pool == NULL) || (pool->initialized == 0)) {
        return;
    }

    pthread_mutex_lock(&pool->mtx);
    if (force_destroy_flag != 0) {
        ls2k0300_threadpool_free_pending_tasks(pool);
        if (pool->working_cnt == 0U) {
            pthread_cond_broadcast(&pool->cond_idle);
        }
    }
    pool->stop = 1;
    pthread_mutex_unlock(&pool->mtx);

    pthread_cond_broadcast(&pool->cond_task);

    for (i = 0U; i < pool->worker_num; ++i) {
        pthread_join(pool->workers[i], NULL);
    }

    pthread_mutex_lock(&pool->mtx);
    ls2k0300_threadpool_free_pending_tasks(pool);
    pool->working_cnt = 0U;
    pool->initialized = 0;
    pthread_mutex_unlock(&pool->mtx);

    free(pool->workers);
    pool->workers = NULL;
    pool->worker_num = 0U;

    pthread_cond_destroy(&pool->cond_idle);
    pthread_cond_destroy(&pool->cond_task);
    pthread_mutex_destroy(&pool->mtx);
    memset(pool, 0, sizeof(*pool));
}

/********************************************************************************
 * @brief   兼容接口：submit.
 * @param   pool     : 线程池句柄.
 * @param   task_fn  : 任务函数.
 * @param   task_arg : 任务参数.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int ls2k0300_threadpool_submit(ls2k0300_threadpool_t *pool,
                               ls2k0300_threadpool_task_fn_t task_fn, void *task_arg)
{
    return ls2k0300_threadpool_add_task(pool, task_fn, task_arg);
}

/********************************************************************************
 * @brief   兼容接口：deinit（等价 destroy(force_destroy_flag=0)）.
 * @param   pool : 线程池句柄.
 * @return  none.
 ********************************************************************************/
void ls2k0300_threadpool_deinit(ls2k0300_threadpool_t *pool)
{
    ls2k0300_threadpool_destroy(pool, 0);
}
