#ifndef __LS2K0300_CANFD_H
#define __LS2K0300_CANFD_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CANFD_MAX_DATA_LEN  64
#define CAN_EXT_MAX_DATA_LEN  8
#define CAN0                "can0"
#define CAN1                "can1"

/********************************************************************************
 * @brief   CANFD 接收模式枚举.
 * @note    推荐优先使用线程模式，避免 SIGIO 对主线程行为造成影响.
 ********************************************************************************/
typedef enum ls2k0300_canfd_rx_mode {
    CANFD_MODE_BLOCKING = 0, /* 阻塞轮询模式 */
    CANFD_MODE_ASYNC    = 1, /* SIGIO 异步信号模式 */
    CANFD_MODE_THREAD   = 2, /* 独立线程接收模式 */
} ls2k0300_canfd_rx_mode_t;

/********************************************************************************
 * @brief   CANFD 数据帧结构体.
 ********************************************************************************/
typedef struct {
    uint32_t can_id;
    uint8_t  len;
    uint8_t  data[CANFD_MAX_DATA_LEN];
} ls2k0300_canfd_frame_t;

/********************************************************************************
 * @brief   经典 CAN 扩展帧结构体.
 * @note    can_id 只填写 29bit 扩展帧 ID，库内部会设置扩展帧标志位.
 * @note    该结构对应硬件报文格式中的 XTD=1、FDF=0、RTR=0 数据帧.
 ********************************************************************************/
typedef struct {
    uint32_t can_id;
    uint8_t  len;
    uint8_t  data[CAN_EXT_MAX_DATA_LEN];
} ls2k0300_can_ext_frame_t;

/********************************************************************************
 * @brief   CANFD 接收回调类型.
 * @param   frame     : 接收到的帧数据.
 * @param   user_data : 用户上下文指针.
 * @return  none.
 ********************************************************************************/
typedef void (*ls2k0300_canfd_rx_callback_t)(const ls2k0300_canfd_frame_t *frame, void *user_data);

/********************************************************************************
 * @brief   CANFD 句柄结构体.
 * @note    句柄必须先调用 ls2k0300_canfd_init 初始化，使用完成后调用 deinit 释放.
 ********************************************************************************/
typedef struct {
    int                           socket;
    char                          ifname[16];
    ls2k0300_canfd_rx_mode_t      rx_mode;
    ls2k0300_canfd_rx_callback_t  rx_cb;
    void                         *user_data;
    int                           initialized;

    pthread_t                     rx_thread;
    volatile int                  running;
    pthread_mutex_t               mtx;
} ls2k0300_canfd_t;

/********************************************************************************
 * @brief   初始化 CANFD 设备与句柄.
 * @param   canfd      : CANFD 句柄指针.
 * @param   ifname     : 接口名，例如 "can0" 或 "can1".
 * @param   rx_mode    : 接收模式，参考 ls2k0300_canfd_rx_mode_t.
 * @param   cb         : 接收回调，可传 NULL.
 * @param   user_data  : 回调上下文，可传 NULL.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_canfd_t can;
 *          ls2k0300_canfd_init(&can, CAN0, CANFD_MODE_THREAD, my_cb, NULL);
 ********************************************************************************/
int ls2k0300_canfd_init(ls2k0300_canfd_t *canfd, const char *ifname,
                        ls2k0300_canfd_rx_mode_t rx_mode,
                        ls2k0300_canfd_rx_callback_t cb, void *user_data);

/********************************************************************************
 * @brief   释放 CANFD 句柄资源.
 * @param   canfd : CANFD 句柄指针.
 * @return  none.
 * @example ls2k0300_canfd_deinit(&can);
 ********************************************************************************/
void ls2k0300_canfd_deinit(ls2k0300_canfd_t *canfd);

/********************************************************************************
 * @brief   发送 CANFD 数据.
 * @param   canfd   : CANFD 句柄.
 * @param   can_id  : 帧 ID.
 * @param   data    : 数据缓冲区.
 * @param   len     : 数据长度，超过 64 将自动截断.
 * @return  成功返回 write 实际字节数，失败返回 -1.
 * @example uint8_t tx[8] = {1,2,3,4,5,6,7,8};
 *          ls2k0300_canfd_write_data(&can, 0x123, tx, 8);
 ********************************************************************************/
int ls2k0300_canfd_write_data(ls2k0300_canfd_t *canfd, uint32_t can_id, const uint8_t *data, uint8_t len);

/********************************************************************************
 * @brief   按帧结构发送 CANFD 数据.
 * @param   canfd : CANFD 句柄.
 * @param   frame : 待发送帧.
 * @return  成功返回 write 实际字节数，失败返回 -1.
 * @example ls2k0300_canfd_frame_t frame = {0x321, 2, {0xAA, 0x55}};
 *          ls2k0300_canfd_write_frame(&can, &frame);
 ********************************************************************************/
int ls2k0300_canfd_write_frame(ls2k0300_canfd_t *canfd, const ls2k0300_canfd_frame_t *frame);

/********************************************************************************
 * @brief   发送经典 CAN 扩展帧数据.
 * @param   canfd   : CANFD 句柄.
 * @param   can_id  : 29bit 扩展帧 ID，不需要包含扩展帧标志位.
 * @param   data    : 数据缓冲区.
 * @param   len     : 数据长度，范围 0~8.
 * @return  成功返回 write 实际字节数，失败返回 -1.
 * @note    该接口发送 classic CAN 帧，而不是 CAN FD 帧；硬件报文格式为 XTD=1、FDF=0.
 * @example uint8_t tx[2] = {0x36, 0x6B};
 *          ls2k0300_canfd_write_ext_data(&can, 0x0100, tx, 2);
 ********************************************************************************/
int ls2k0300_canfd_write_ext_data(ls2k0300_canfd_t *canfd, uint32_t can_id, const uint8_t *data, uint8_t len);

/********************************************************************************
 * @brief   按结构体发送经典 CAN 扩展帧.
 * @param   canfd : CANFD 句柄.
 * @param   frame : 待发送扩展帧.
 * @return  成功返回 write 实际字节数，失败返回 -1.
 * @example ls2k0300_can_ext_frame_t frame = {0x0100, 2, {0x36, 0x6B}};
 *          ls2k0300_canfd_write_ext_frame(&can, &frame);
 ********************************************************************************/
int ls2k0300_canfd_write_ext_frame(ls2k0300_canfd_t *canfd, const ls2k0300_can_ext_frame_t *frame);

/********************************************************************************
 * @brief   读取一帧 CANFD 数据.
 * @param   canfd       : CANFD 句柄.
 * @param   frame       : 输出帧结构体.
 * @param   timeout_ms  : 超时毫秒，-1 表示无限等待.
 * @return  >0 为读取字节数，0 为超时，-1 为失败.
 * @example ls2k0300_canfd_frame_t rx;
 *          int ret = ls2k0300_canfd_read_frame(&can, &rx, 1000);
 ********************************************************************************/
int ls2k0300_canfd_read_frame(ls2k0300_canfd_t *canfd, ls2k0300_canfd_frame_t *frame, int timeout_ms);

/********************************************************************************
 * @brief   读取一帧经典 CAN 扩展帧.
 * @param   canfd       : CANFD 句柄.
 * @param   frame       : 输出扩展帧结构体.
 * @param   timeout_ms  : 超时毫秒，-1 表示无限等待.
 * @return  >0 为读取字节数，0 为超时，-1 为失败.
 * @note    该接口只返回 classic CAN 扩展数据帧；标准帧、RTR、CAN FD 帧会被忽略.
 * @note    使用 CANFD_MODE_THREAD 时，后台线程会先读取 socket；需要同步读取时推荐使用 CANFD_MODE_BLOCKING.
 * @example ls2k0300_can_ext_frame_t rx;
 *          int ret = ls2k0300_canfd_read_ext_frame(&can, &rx, 1000);
 ********************************************************************************/
int ls2k0300_canfd_read_ext_frame(ls2k0300_canfd_t *canfd, ls2k0300_can_ext_frame_t *frame, int timeout_ms);

/********************************************************************************
 * @brief   运行时更新接收回调.
 * @param   canfd      : CANFD 句柄.
 * @param   cb         : 新回调函数，可传 NULL 清空.
 * @param   user_data  : 新上下文指针.
 * @return  none.
 * @example ls2k0300_canfd_set_rx_callback(&can, my_cb, &ctx);
 ********************************************************************************/
void ls2k0300_canfd_set_rx_callback(ls2k0300_canfd_t *canfd, ls2k0300_canfd_rx_callback_t cb, void *user_data);

/********************************************************************************
 * @brief   获取当前 socket fd.
 * @param   canfd : CANFD 句柄.
 * @return  成功返回 fd，失败返回 -1.
 * @example int fd = ls2k0300_canfd_get_socket(&can);
 ********************************************************************************/
int ls2k0300_canfd_get_socket(ls2k0300_canfd_t *canfd);

#ifdef __cplusplus
}
#endif

#endif
