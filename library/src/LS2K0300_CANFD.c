#include "LS2K0300_CANFD.h"

#include <errno.h>
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

#ifndef LS2K0300_CAN_DEFAULT_BITRATE
#define LS2K0300_CAN_DEFAULT_BITRATE     500000U
#endif

#ifndef LS2K0300_CANFD_DEFAULT_DBITRATE
#define LS2K0300_CANFD_DEFAULT_DBITRATE  2000000U
#endif

/*
 * 电机协议使用 classic CAN 扩展帧。默认先按 classic CAN 拉起接口，
 * 如需 CAN FD，可在编译参数中定义 LS2K0300_CANFD_ENABLE=1。
 */
#ifndef LS2K0300_CANFD_ENABLE
#define LS2K0300_CANFD_ENABLE            0
#endif

/* SIGIO 模式下用于从信号处理函数访问实例 */
static ls2k0300_canfd_t *s_canfd_instance = NULL;

/********************************************************************************
 * @brief   SIGIO 信号回调函数（异步模式内部使用）.
 * @param   signo : 信号编号.
 * @return  none.
 * @note    在信号上下文中仅做最小工作：读 socket + 转发回调.
 ********************************************************************************/
static void canfd_signal_handler(int signo)
{
    struct canfd_frame frame;
    int nbytes;

    (void)signo;

    if (s_canfd_instance == NULL) {
        return;
    }

    /* 把当前 socket 可读数据一次性读空，避免信号重复堆积 */
    while (1) {
        nbytes = (int)read(s_canfd_instance->socket, &frame, sizeof(frame));
        if (nbytes <= 0) {
            break;
        }

        if (s_canfd_instance->rx_cb != NULL) {
            ls2k0300_canfd_frame_t rx_frame;
            rx_frame.can_id = frame.can_id;
            rx_frame.len = frame.len;
            memcpy(rx_frame.data, frame.data, frame.len);
            s_canfd_instance->rx_cb(&rx_frame, s_canfd_instance->user_data);
        }
    }
}

/********************************************************************************
 * @brief   线程接收函数（线程模式内部使用）.
 * @param   arg : ls2k0300_canfd_t 指针.
 * @return  线程退出时返回 NULL.
 * @note    使用 poll 定时醒来，便于快速响应 running 关闭标记.
 ********************************************************************************/
static void *canfd_rx_thread_func(void *arg)
{
    ls2k0300_canfd_t *canfd;
    struct pollfd pfd;

    canfd = (ls2k0300_canfd_t *)arg;
    pfd.fd = canfd->socket;
    pfd.events = POLLIN;

    while (canfd->running) {
        int ret = poll(&pfd, 1, 100);
        if (!canfd->running) {
            break;
        }

        if (ret > 0 && (pfd.revents & POLLIN) != 0) {
            struct canfd_frame frame;
            int nbytes = (int)read(canfd->socket, &frame, sizeof(frame));
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

/********************************************************************************
 * @brief   初始化 CANFD 句柄.
 * @param   canfd      : CANFD 句柄.
 * @param   ifname     : 接口名称（如 can0）.
 * @param   rx_mode    : 接收模式.
 * @param   cb         : 接收回调.
 * @param   user_data  : 回调上下文.
 * @return  成功返回 0，失败返回 -1.
 * @example ls2k0300_canfd_t can;
 *          ls2k0300_canfd_init(&can, CAN0, CANFD_MODE_THREAD, my_cb, NULL);
 ********************************************************************************/
int ls2k0300_canfd_init(ls2k0300_canfd_t *canfd, const char *ifname,
                        ls2k0300_canfd_rx_mode_t rx_mode,
                        ls2k0300_canfd_rx_callback_t cb, void *user_data)
{
    struct ifreq ifr;
    struct sockaddr_can addr;
    char cmd[256];
    unsigned int bitrate;
    const char *bitrate_env;

    if (canfd == NULL || ifname == NULL) {
        return -1;
    }

    memset(canfd, 0, sizeof(*canfd));
    pthread_mutex_init(&canfd->mtx, NULL);

    canfd->socket = -1;
    canfd->rx_mode = rx_mode;
    canfd->rx_cb = cb;
    canfd->user_data = user_data;
    canfd->running = 0;
    canfd->initialized = 0;
    bitrate = (unsigned int)LS2K0300_CAN_DEFAULT_BITRATE;

    /*
     * 调试现场时可用环境变量临时覆盖波特率，例如:
     * LS2K0300_CAN_BITRATE=1000000 ./example_canfd
     */
    bitrate_env = getenv("LS2K0300_CAN_BITRATE");
    if (bitrate_env != NULL && bitrate_env[0] != '\0') {
        char *endptr = NULL;
        unsigned long value = strtoul(bitrate_env, &endptr, 10);

        if (endptr != bitrate_env && *endptr == '\0' && value > 0UL && value <= 8000000UL) {
            bitrate = (unsigned int)value;
        } else {
            printf("Ignore invalid LS2K0300_CAN_BITRATE=%s\n", bitrate_env);
        }
    }

    strncpy(canfd->ifname, ifname, sizeof(canfd->ifname) - 1U);
    canfd->ifname[sizeof(canfd->ifname) - 1U] = '\0';

    /* 先重置接口状态，再统一拉起到 CAN/CANFD 参数 */
    snprintf(cmd, sizeof(cmd), "ip link set %s down", ifname);
    (void)system(cmd);

#if LS2K0300_CANFD_ENABLE
    snprintf(cmd, sizeof(cmd),
             "ip link set %s up type can bitrate %u dbitrate %u fd on restart-ms 100",
             ifname,
             bitrate,
             (unsigned int)LS2K0300_CANFD_DEFAULT_DBITRATE);
#else
    snprintf(cmd, sizeof(cmd),
             "ip link set %s up type can bitrate %u fd off restart-ms 100",
             ifname,
             bitrate);
#endif
    printf("CAN init cmd: %s\n", cmd);
    if (system(cmd) != 0) {
        printf("CAN cmd failed: %s\n", cmd);
        pthread_mutex_destroy(&canfd->mtx);
        return -1;
    }

    usleep(100 * 1000);

    /* 建立原始 CAN socket 并开启 FD 帧支持 */
    canfd->socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (canfd->socket < 0) {
        pthread_mutex_destroy(&canfd->mtx);
        return -1;
    }

    {
        int enable_fd = 1;
        if (setsockopt(canfd->socket, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) < 0) {
            close(canfd->socket);
            canfd->socket = -1;
            pthread_mutex_destroy(&canfd->mtx);
            return -1;
        }
    }

    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1U);
    if (ioctl(canfd->socket, SIOCGIFINDEX, &ifr) < 0) {
        close(canfd->socket);
        canfd->socket = -1;
        pthread_mutex_destroy(&canfd->mtx);
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(canfd->socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(canfd->socket);
        canfd->socket = -1;
        pthread_mutex_destroy(&canfd->mtx);
        return -1;
    }

    /* 根据模式配置不同接收策略 */
    if (rx_mode == CANFD_MODE_ASYNC) {
        signal(SIGIO, canfd_signal_handler);
        if (fcntl(canfd->socket, F_SETOWN, getpid()) < 0) {
            close(canfd->socket);
            canfd->socket = -1;
            pthread_mutex_destroy(&canfd->mtx);
            return -1;
        }

        {
            int flags = fcntl(canfd->socket, F_GETFL);
            if (fcntl(canfd->socket, F_SETFL, flags | O_ASYNC | O_NONBLOCK) < 0) {
                close(canfd->socket);
                canfd->socket = -1;
                pthread_mutex_destroy(&canfd->mtx);
                return -1;
            }
        }

        s_canfd_instance = canfd;
    } else if (rx_mode == CANFD_MODE_THREAD) {
        canfd->running = 1;
        if (pthread_create(&canfd->rx_thread, NULL, canfd_rx_thread_func, canfd) != 0) {
            canfd->running = 0;
            close(canfd->socket);
            canfd->socket = -1;
            pthread_mutex_destroy(&canfd->mtx);
            return -1;
        }
    } else {
        int flags = fcntl(canfd->socket, F_GETFL, 0);
        (void)fcntl(canfd->socket, F_SETFL, flags | O_NONBLOCK);
    }

    canfd->initialized = 1;
    return 0;
}

/********************************************************************************
 * @brief   释放 CANFD 句柄.
 * @param   canfd : CANFD 句柄.
 * @return  none.
 * @example ls2k0300_canfd_deinit(&can);
 ********************************************************************************/
void ls2k0300_canfd_deinit(ls2k0300_canfd_t *canfd)
{
    if (canfd == NULL) {
        return;
    }

    /* 先拉低运行标记，再等待接收线程退出 */
    canfd->running = 0;
    if (canfd->rx_mode == CANFD_MODE_THREAD && canfd->rx_thread != 0U) {
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

    canfd->initialized = 0;

    pthread_mutex_unlock(&canfd->mtx);
    pthread_mutex_destroy(&canfd->mtx);
}

/********************************************************************************
 * @brief   发送 CANFD 数据.
 * @param   canfd   : CANFD 句柄.
 * @param   can_id  : 帧 ID.
 * @param   data    : 数据缓冲区.
 * @param   len     : 数据长度.
 * @return  成功返回写入字节数，失败返回 -1.
 * @example uint8_t tx[8] = {1,2,3,4,5,6,7,8};
 *          ls2k0300_canfd_write_data(&can, 0x123, tx, 8);
 ********************************************************************************/
int ls2k0300_canfd_write_data(ls2k0300_canfd_t *canfd, uint32_t can_id, const uint8_t *data, uint8_t len)
{
    struct canfd_frame frame;
    int ret;

    if (canfd == NULL || canfd->initialized == 0 || canfd->socket < 0 || data == NULL) {
        return -1;
    }

    if (len > CANFD_MAX_DATA_LEN) {
        len = CANFD_MAX_DATA_LEN;
    }

    /* 组帧后统一走 write 发送 */
    memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id;
    frame.len = len;
    memcpy(frame.data, data, len);

    pthread_mutex_lock(&canfd->mtx);
    ret = (int)write(canfd->socket, &frame, sizeof(frame));
    pthread_mutex_unlock(&canfd->mtx);

    return ret;
}

/********************************************************************************
 * @brief   发送 CANFD 帧结构.
 * @param   canfd : CANFD 句柄.
 * @param   frame : 待发送帧.
 * @return  成功返回写入字节数，失败返回 -1.
 * @example ls2k0300_canfd_frame_t frame = {0x321, 2, {0xAA, 0x55}};
 *          ls2k0300_canfd_write_frame(&can, &frame);
 ********************************************************************************/
int ls2k0300_canfd_write_frame(ls2k0300_canfd_t *canfd, const ls2k0300_canfd_frame_t *frame)
{
    if (frame == NULL) {
        return -1;
    }

    return ls2k0300_canfd_write_data(canfd, frame->can_id, frame->data, frame->len);
}

/********************************************************************************
 * @brief   发送经典 CAN 扩展帧数据.
 * @param   canfd   : CANFD 句柄.
 * @param   can_id  : 29bit 扩展帧 ID，不需要包含扩展帧标志位.
 * @param   data    : 数据缓冲区.
 * @param   len     : 数据长度，范围 0~8.
 * @return  成功返回 write 实际字节数，失败返回 -1.
 * @note    硬件报文格式中 XTD=1 表示 29bit 扩展 ID，FDF=0 表示 CAN2.0
 *          classic 帧，RTR=0 表示数据帧；SocketCAN 中分别由 CAN_EFF_FLAG
 *          和 struct can_frame 体现。
 * @example uint8_t tx[2] = {0x36, 0x6B};
 *          ls2k0300_canfd_write_ext_data(&can, 0x0100, tx, 2);
 ********************************************************************************/
int ls2k0300_canfd_write_ext_data(ls2k0300_canfd_t *canfd, uint32_t can_id, const uint8_t *data, uint8_t len)
{
    struct can_frame frame;
    int ret;

    if (canfd == NULL || canfd->initialized == 0 || canfd->socket < 0) {
        return -1;
    }
    if (len > CAN_EXT_MAX_DATA_LEN || (len > 0U && data == NULL)) {
        return -1;
    }

    memset(&frame, 0, sizeof(frame));
    frame.can_id = (can_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
    frame.can_dlc = len;
    if (len > 0U) {
        memcpy(frame.data, data, len);
    }

    pthread_mutex_lock(&canfd->mtx);
    ret = (int)write(canfd->socket, &frame, sizeof(frame));
    pthread_mutex_unlock(&canfd->mtx);

    return ret;
}

/********************************************************************************
 * @brief   按结构体发送经典 CAN 扩展帧.
 * @param   canfd : CANFD 句柄.
 * @param   frame : 待发送扩展帧.
 * @return  成功返回 write 实际字节数，失败返回 -1.
 * @example ls2k0300_can_ext_frame_t frame = {0x0100, 2, {0x36, 0x6B}};
 *          ls2k0300_canfd_write_ext_frame(&can, &frame);
 ********************************************************************************/
int ls2k0300_canfd_write_ext_frame(ls2k0300_canfd_t *canfd, const ls2k0300_can_ext_frame_t *frame)
{
    if (frame == NULL) {
        return -1;
    }

    return ls2k0300_canfd_write_ext_data(canfd, frame->can_id, frame->data, frame->len);
}

/********************************************************************************
 * @brief   接收一帧 CANFD 数据.
 * @param   canfd      : CANFD 句柄.
 * @param   frame      : 输出帧.
 * @param   timeout_ms : 超时毫秒.
 * @return  >0 为读取字节数，0 为超时，-1 为失败.
 * @example ls2k0300_canfd_frame_t rx;
 *          int ret = ls2k0300_canfd_read_frame(&can, &rx, 1000);
 ********************************************************************************/
int ls2k0300_canfd_read_frame(ls2k0300_canfd_t *canfd, ls2k0300_canfd_frame_t *frame, int timeout_ms)
{
    struct pollfd pfd;
    struct canfd_frame can_frame;
    int ret;
    int nbytes;

    if (canfd == NULL || frame == NULL || canfd->initialized == 0 || canfd->socket < 0) {
        return -1;
    }

    pfd.fd = canfd->socket;
    pfd.events = POLLIN;

    ret = poll(&pfd, 1, timeout_ms);
    if (ret <= 0) {
        return ret;
    }

    nbytes = (int)read(canfd->socket, &can_frame, sizeof(can_frame));
    if (nbytes <= 0) {
        return -1;
    }

    /* 转换为库内统一帧结构输出 */
    frame->can_id = can_frame.can_id;
    frame->len = can_frame.len;
    memcpy(frame->data, can_frame.data, can_frame.len);

    return nbytes;
}

/********************************************************************************
 * @brief   读取一帧经典 CAN 扩展帧.
 * @param   canfd      : CANFD 句柄.
 * @param   frame      : 输出扩展帧结构体.
 * @param   timeout_ms : 超时毫秒，-1 表示无限等待.
 * @return  >0 为读取字节数，0 为超时，-1 为失败.
 * @note    只返回 XTD=1、FDF=0、RTR=0 的 classic CAN 扩展数据帧。
 *          标准帧、远程帧和 CANFD 帧会被忽略。
 * @example ls2k0300_can_ext_frame_t rx;
 *          int ret = ls2k0300_canfd_read_ext_frame(&can, &rx, 1000);
 ********************************************************************************/
int ls2k0300_canfd_read_ext_frame(ls2k0300_canfd_t *canfd, ls2k0300_can_ext_frame_t *frame, int timeout_ms)
{
    struct pollfd pfd;

    if (canfd == NULL || frame == NULL || canfd->initialized == 0 || canfd->socket < 0) {
        return -1;
    }

    memset(frame, 0, sizeof(*frame));

    pfd.fd = canfd->socket;
    pfd.events = POLLIN;
    pfd.revents = 0;

    while (1) {
        struct canfd_frame raw_frame;
        int ret;
        int nbytes;

        ret = poll(&pfd, 1, timeout_ms);
        if (ret == 0) {
            return 0;
        }
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }

        if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            return -1;
        }
        if ((pfd.revents & POLLIN) == 0) {
            continue;
        }

        memset(&raw_frame, 0, sizeof(raw_frame));
        nbytes = (int)read(canfd->socket, &raw_frame, sizeof(raw_frame));
        if (nbytes < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            return -1;
        }

        /* classic CAN 扩展帧在 SocketCAN 中长度为 CAN_MTU，CANFD_MTU 表示 FDF=1。 */
        if (nbytes != CAN_MTU) {
            continue;
        }
        if ((raw_frame.can_id & CAN_EFF_FLAG) == 0U) {
            continue;
        }
        if ((raw_frame.can_id & CAN_RTR_FLAG) != 0U) {
            continue;
        }
        if (raw_frame.len > CAN_EXT_MAX_DATA_LEN) {
            continue;
        }

        frame->can_id = raw_frame.can_id & CAN_EFF_MASK;
        frame->len = raw_frame.len;
        if (frame->len > 0U) {
            memcpy(frame->data, raw_frame.data, frame->len);
        }

        return nbytes;
    }
}

/********************************************************************************
 * @brief   设置接收回调.
 * @param   canfd      : CANFD 句柄.
 * @param   cb         : 回调函数.
 * @param   user_data  : 回调上下文.
 * @return  none.
 * @example ls2k0300_canfd_set_rx_callback(&can, my_cb, &ctx);
 ********************************************************************************/
void ls2k0300_canfd_set_rx_callback(ls2k0300_canfd_t *canfd, ls2k0300_canfd_rx_callback_t cb, void *user_data)
{
    if (canfd == NULL) {
        return;
    }

    pthread_mutex_lock(&canfd->mtx);
    canfd->rx_cb = cb;
    canfd->user_data = user_data;
    pthread_mutex_unlock(&canfd->mtx);
}

/********************************************************************************
 * @brief   获取 socket 描述符.
 * @param   canfd : CANFD 句柄.
 * @return  成功返回 fd，失败返回 -1.
 * @example int fd = ls2k0300_canfd_get_socket(&can);
 ********************************************************************************/
int ls2k0300_canfd_get_socket(ls2k0300_canfd_t *canfd)
{
    if (canfd == NULL) {
        return -1;
    }
    return canfd->socket;
}
