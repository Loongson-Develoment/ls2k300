#include "LS2K0300_CANFD.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <fcntl.h>
#include <signal.h>
#include <poll.h>

static ls2k0300_canfd_t* s_canfd_instance = NULL;

static void canfd_signal_handler(int signo) {
    if (s_canfd_instance == NULL) return;
    
    struct canfd_frame frame;
    while (1) {
        int nbytes = read(s_canfd_instance->socket, &frame, sizeof(frame));
        if (nbytes <= 0) break;
        
        if (s_canfd_instance->rx_cb != NULL) {
            ls2k0300_canfd_frame_t rx_frame;
            rx_frame.can_id = frame.can_id;
            rx_frame.len = frame.len;
            memcpy(rx_frame.data, frame.data, frame.len);
            s_canfd_instance->rx_cb(&rx_frame, s_canfd_instance->user_data);
        }
    }
}

static void* canfd_rx_thread_func(void* arg) {
    ls2k0300_canfd_t* canfd = (ls2k0300_canfd_t*)arg;
    struct pollfd pfd;
    pfd.fd = canfd->socket;
    pfd.events = POLLIN;

    while (canfd->running) {
        int ret = poll(&pfd, 1, 100);
        if (!canfd->running) break;
        
        if (ret > 0 && (pfd.revents & POLLIN)) {
            struct canfd_frame frame;
            int nbytes = read(canfd->socket, &frame, sizeof(frame));
            if (nbytes > 0 && canfd->rx_cb != NULL) {
                ls2k0300_canfd_frame_t rx_frame;
                rx_frame.can_id = frame.can_id;
                rx_frame.len = frame.len;
                memcpy(rx_frame.data, frame.data, frame.len);
                canfd->rx_cb(&rx_frame, canfd->user_data);
            }
        }
    }
    return NULL;
}

int ls2k0300_canfd_init(ls2k0300_canfd_t* canfd, const char* ifname, ls2k0300_canfd_rx_mode_t rx_mode, ls2k0300_canfd_rx_callback_t cb, void* user_data) {
    if (canfd == NULL || ifname == NULL) return -1;

    pthread_mutex_init(&canfd->mtx, NULL);
    canfd->socket = -1;
    canfd->rx_mode = rx_mode;
    canfd->rx_cb = cb;
    canfd->user_data = user_data;
    canfd->running = false;
    canfd->initialized = false;
    strncpy(canfd->ifname, ifname, sizeof(canfd->ifname) - 1);

    char cmd[256];
    snprintf(cmd, sizeof(cmd), "ip link set %s down", ifname);
    system(cmd);

    snprintf(cmd, sizeof(cmd), "ip link set %s up type can bitrate 500000 dbitrate 2000000 fd on", ifname);
    if (system(cmd) != 0) {
        printf("CANFD cmd failed\n");
        return -1;
    }
    usleep(100 * 1000);

    canfd->socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (canfd->socket < 0) return -1;

    int enable_fd = 1;
    if (setsockopt(canfd->socket, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) < 0) {
        close(canfd->socket);
        canfd->socket = -1;
        return -1;
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, ifname);
    if (ioctl(canfd->socket, SIOCGIFINDEX, &ifr) < 0) {
        close(canfd->socket);
        canfd->socket = -1;
        return -1;
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(canfd->socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(canfd->socket);
        canfd->socket = -1;
        return -1;
    }

    if (rx_mode == CANFD_MODE_ASYNC) {
        signal(SIGIO, canfd_signal_handler);
        if (fcntl(canfd->socket, F_SETOWN, getpid()) < 0) return -1;
        int flags = fcntl(canfd->socket, F_GETFL);
        if (fcntl(canfd->socket, F_SETFL, flags | O_ASYNC | O_NONBLOCK) < 0) return -1;
        s_canfd_instance = canfd;
    } else if (rx_mode == CANFD_MODE_THREAD) {
        canfd->running = true;
        if (pthread_create(&canfd->rx_thread, NULL, canfd_rx_thread_func, canfd) != 0) {
            canfd->running = false;
            return -1;
        }
    } else {
        int flags = fcntl(canfd->socket, F_GETFL, 0);
        fcntl(canfd->socket, F_SETFL, flags | O_NONBLOCK);
    }

    canfd->initialized = true;
    return 0;
}

void ls2k0300_canfd_deinit(ls2k0300_canfd_t* canfd) {
    if (canfd == NULL) return;
    
    canfd->running = false;
    if (canfd->rx_mode == CANFD_MODE_THREAD) {
        pthread_join(canfd->rx_thread, NULL);
    }

    pthread_mutex_lock(&canfd->mtx);
    if (canfd->socket >= 0) {
        close(canfd->socket);
        canfd->socket = -1;
    }
    if (s_canfd_instance == canfd) {
        s_canfd_instance = NULL;
    }
    canfd->initialized = false;
    pthread_mutex_unlock(&canfd->mtx);
    pthread_mutex_destroy(&canfd->mtx);
}

int ls2k0300_canfd_write_data(ls2k0300_canfd_t* canfd, uint32_t can_id, const uint8_t* data, uint8_t len) {
    if (canfd == NULL || !canfd->initialized || canfd->socket < 0) return -1;
    
    if (len > CANFD_MAX_DATA_LEN) len = CANFD_MAX_DATA_LEN;
    
    struct canfd_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id;
    frame.len = len;
    memcpy(frame.data, data, len);
    
    pthread_mutex_lock(&canfd->mtx);
    int ret = write(canfd->socket, &frame, sizeof(frame));
    pthread_mutex_unlock(&canfd->mtx);
    return ret;
}

int ls2k0300_canfd_write_frame(ls2k0300_canfd_t* canfd, const ls2k0300_canfd_frame_t* frame) {
    if (frame == NULL) return -1;
    return ls2k0300_canfd_write_data(canfd, frame->can_id, frame->data, frame->len);
}

int ls2k0300_canfd_read_frame(ls2k0300_canfd_t* canfd, ls2k0300_canfd_frame_t* frame, int timeout_ms) {
    if (canfd == NULL || !canfd->initialized || canfd->socket < 0 || frame == NULL) return -1;

    struct pollfd pfd;
    pfd.fd = canfd->socket;
    pfd.events = POLLIN;

    int ret = poll(&pfd, 1, timeout_ms);
    if (ret <= 0) return ret; // 0 for timeout, -1 for error

    struct canfd_frame can_frame;
    int nbytes = read(canfd->socket, &can_frame, sizeof(can_frame));
    if (nbytes <= 0) return -1;

    frame->can_id = can_frame.can_id;
    frame->len = can_frame.len;
    memcpy(frame->data, can_frame.data, can_frame.len);
    return nbytes;
}
