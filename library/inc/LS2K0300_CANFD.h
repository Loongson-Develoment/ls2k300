#ifndef __LS2K0300_CANFD_H
#define __LS2K0300_CANFD_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CANFD_MAX_DATA_LEN 64
#define CAN0 "can0"
#define CAN1 "can1"

typedef enum {
    CANFD_MODE_BLOCKING = 0,    // 阻塞模式
    CANFD_MODE_ASYNC    = 1,    // 异步信号模式
    CANFD_MODE_THREAD   = 2,    // 独立线程模式
} ls2k0300_canfd_rx_mode_t;

typedef struct {
    uint32_t can_id;
    uint8_t  len;
    uint8_t  data[CANFD_MAX_DATA_LEN];
} ls2k0300_canfd_frame_t;

typedef void (*ls2k0300_canfd_rx_callback_t)(const ls2k0300_canfd_frame_t* frame, void* user_data);

typedef struct {
    int socket;
    char ifname[16];
    ls2k0300_canfd_rx_mode_t rx_mode;
    ls2k0300_canfd_rx_callback_t rx_cb;
    void* user_data;
    bool initialized;

    pthread_t rx_thread;
    volatile bool running;
    pthread_mutex_t mtx;
} ls2k0300_canfd_t;

int ls2k0300_canfd_init(ls2k0300_canfd_t* canfd, const char* ifname, ls2k0300_canfd_rx_mode_t rx_mode, ls2k0300_canfd_rx_callback_t cb, void* user_data);
void ls2k0300_canfd_deinit(ls2k0300_canfd_t* canfd);

int ls2k0300_canfd_write_data(ls2k0300_canfd_t* canfd, uint32_t can_id, const uint8_t* data, uint8_t len);
int ls2k0300_canfd_write_frame(ls2k0300_canfd_t* canfd, const ls2k0300_canfd_frame_t* frame);
int ls2k0300_canfd_read_frame(ls2k0300_canfd_t* canfd, ls2k0300_canfd_frame_t* frame, int timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // __LS2K0300_CANFD_H
