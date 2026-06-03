#include <errno.h>
#include <linux/can.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "LS2K0300_CANFD.h"

#define CAN_REPLY_QUEUE_CAPACITY  (16U)

typedef struct {
    uint8_t addr;
    uint8_t packet;
    uint8_t code;
    uint8_t payload_len;
    uint8_t payload[7];
} can_motor_reply_t;

typedef struct {
    can_motor_reply_t items[CAN_REPLY_QUEUE_CAPACITY];
    size_t head;
    size_t tail;
    size_t count;
    size_t dropped;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
} can_reply_queue_t;

static const char *can_reply_status_string(uint8_t status)
{
    switch (status) {
        case 0x02U: return "OK";
        case 0xE2U: return "PARAM_ERROR";
        case 0xEEU: return "FORMAT_ERROR";
        case 0x9FU: return "ACTION_DONE";
        default:    return "UNKNOWN";
    }
}

/********************************************************************************
 * @brief   初始化电机应答队列.
 ********************************************************************************/
static int can_reply_queue_init(can_reply_queue_t *queue)
{
    if (queue == NULL) {
        return -1;
    }

    memset(queue, 0, sizeof(*queue));
    if (pthread_mutex_init(&queue->mutex, NULL) != 0) {
        return -1;
    }
    if (pthread_cond_init(&queue->cond, NULL) != 0) {
        pthread_mutex_destroy(&queue->mutex);
        return -1;
    }

    return 0;
}

/********************************************************************************
 * @brief   释放电机应答队列资源.
 ********************************************************************************/
static void can_reply_queue_deinit(can_reply_queue_t *queue)
{
    if (queue == NULL) {
        return;
    }

    pthread_cond_destroy(&queue->cond);
    pthread_mutex_destroy(&queue->mutex);
}

/********************************************************************************
 * @brief   将一帧已解析应答写入队列.
 * @note    队列满时丢弃最旧的一帧，保留最新数据.
 ********************************************************************************/
static void can_reply_queue_push(can_reply_queue_t *queue, const can_motor_reply_t *reply)
{
    if (queue == NULL || reply == NULL) {
        return;
    }

    pthread_mutex_lock(&queue->mutex);

    if (queue->count == CAN_REPLY_QUEUE_CAPACITY) {
        queue->head = (queue->head + 1U) % CAN_REPLY_QUEUE_CAPACITY;
        queue->count--;
        queue->dropped++;
    }

    queue->items[queue->tail] = *reply;
    queue->tail = (queue->tail + 1U) % CAN_REPLY_QUEUE_CAPACITY;
    queue->count++;

    pthread_cond_signal(&queue->cond);
    pthread_mutex_unlock(&queue->mutex);
}

/********************************************************************************
 * @brief   从队列中等待一帧应答.
 * @param   timeout_ms : 超时毫秒；-1 表示一直等待.
 * @return  1=成功取到，0=超时，-1=错误.
 ********************************************************************************/
static int can_reply_queue_pop(can_reply_queue_t *queue, can_motor_reply_t *reply, int timeout_ms)
{
    if (queue == NULL || reply == NULL) {
        return -1;
    }

    pthread_mutex_lock(&queue->mutex);

    while (queue->count == 0U) {
        if (timeout_ms < 0) {
            if (pthread_cond_wait(&queue->cond, &queue->mutex) != 0) {
                pthread_mutex_unlock(&queue->mutex);
                return -1;
            }
        } else {
            struct timespec ts;

            if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
                pthread_mutex_unlock(&queue->mutex);
                return -1;
            }

            ts.tv_sec += (time_t)(timeout_ms / 1000);
            ts.tv_nsec += (long)((timeout_ms % 1000) * 1000000L);
            if (ts.tv_nsec >= 1000000000L) {
                ts.tv_sec += 1;
                ts.tv_nsec -= 1000000000L;
            }

            {
                int wait_ret = pthread_cond_timedwait(&queue->cond, &queue->mutex, &ts);
                if (wait_ret == ETIMEDOUT) {
                    pthread_mutex_unlock(&queue->mutex);
                    return 0;
                }
                if (wait_ret != 0) {
                    pthread_mutex_unlock(&queue->mutex);
                    return -1;
                }
            }
        }
    }

    *reply = queue->items[queue->head];
    queue->head = (queue->head + 1U) % CAN_REPLY_QUEUE_CAPACITY;
    queue->count--;

    pthread_mutex_unlock(&queue->mutex);
    return 1;
}

/********************************************************************************
 * @brief   将原始回调帧解析为电机协议应答.
 * @return  1=解析成功，0=不是目标协议帧.
 ********************************************************************************/
static int can_parse_reply_frame(const ls2k0300_canfd_frame_t *frame, can_motor_reply_t *reply)
{
    uint32_t can_id;

    if (frame == NULL || reply == NULL) {
        return 0;
    }

    if ((frame->can_id & CAN_EFF_FLAG) == 0U) {
        return 0;
    }
    if ((frame->can_id & CAN_RTR_FLAG) != 0U) {
        return 0;
    }
    if (frame->len == 0U || frame->len > CAN_EXT_MAX_DATA_LEN) {
        return 0;
    }

    memset(reply, 0, sizeof(*reply));

    can_id = frame->can_id & CAN_EFF_MASK;
    reply->addr = (uint8_t)((can_id >> 8U) & 0xFFU);
    reply->packet = (uint8_t)(can_id & 0xFFU);
    reply->code = frame->data[0];
    reply->payload_len = (uint8_t)(frame->len - 1U);
    if (reply->payload_len > 0U) {
        memcpy(reply->payload, &frame->data[1], reply->payload_len);
    }

    return 1;
}

/********************************************************************************
 * @brief   后台接收回调：解析电机应答并压入队列.
 ********************************************************************************/
static void can_reply_rx_callback(const ls2k0300_canfd_frame_t *frame, void *user_data)
{
    can_reply_queue_t *queue;
    can_motor_reply_t reply;

    queue = (can_reply_queue_t *)user_data;
    if (queue == NULL) {
        return;
    }

    if (can_parse_reply_frame(frame, &reply) == 0) {
        return;
    }

    can_reply_queue_push(queue, &reply);
}

/********************************************************************************
 * @brief   按电机 CAN 扩展帧协议发送一条完整命令.
 * @note    命令格式: cmd[0]=Addr, cmd[1]=Code, cmd[2...]=命令数据/校验码.
 * @note    扩展帧 ID: (Addr << 8) | Packet, Packet 从 0 开始递增.
 * @note    经典 CAN 单帧最多 8 字节数据，本协议每帧第 1 字节固定为 Code,
 *          因此每包最多携带 7 字节命令数据/校验码.
 ********************************************************************************/
static int can_send_cmd(ls2k0300_canfd_t *canfd, const uint8_t *cmd, uint8_t len)
{
    uint8_t payload_len;
    uint8_t offset;
    uint8_t packet;

    if (canfd == NULL || cmd == NULL || len < 2U) {
        return -1;
    }

    /* 除去 Addr 与 Code 后，剩余部分都作为命令数据/校验码参与分包. */
    payload_len = (uint8_t)(len - 2U);
    offset = 0U;
    packet = 0U;

    do {
        uint8_t frame_data[CAN_EXT_MAX_DATA_LEN];
        uint8_t chunk_len;
        uint8_t i;
        int ret;

        /* 每包最多放 7 字节 payload，因为 data[0] 要放功能码 Code. */
        chunk_len = (uint8_t)(payload_len - offset);
        if (chunk_len > 7U) {
            chunk_len = 7U;
        }

        /* 每包第一个数据字节固定为功能码 Code. */
        frame_data[0] = cmd[1];

        /* 从 cmd[2] 开始拷贝命令数据/校验码到 data[1...]. */
        for (i = 0U; i < chunk_len; ++i) {
            frame_data[i + 1U] = cmd[offset + 2U + i];
        }

        /*
         * 新库接口要求 can_id 只传 29bit 扩展帧 ID 本体：
         * (Addr << 8) | Packet，不需要手动拼 CAN_EFF_FLAG。
         */
        ret = ls2k0300_canfd_write_ext_data(
            canfd,
            ((uint32_t)cmd[0] << 8U) | (uint32_t)packet,
            frame_data,
            (uint8_t)(chunk_len + 1U)
        );
        if (ret < 0) {
            return -1;
        }

        /* 更新下一包的 payload 偏移与 Packet 序号. */
        offset = (uint8_t)(offset + chunk_len);
        ++packet;
    } while (offset < payload_len);

    return 0;
}

/********************************************************************************
 * @brief   接收并解析一包电机返回帧.
 * @param   canfd      : CAN 句柄.
 * @param   reply      : 输出解析结果.
 * @param   timeout_ms : 等待超时，单位 ms；-1 表示一直等.
 * @return  1=收到有效扩展帧，0=超时，-1=错误.
 * @note    线程模式下实际由后台回调收包，这里只负责从队列中取解析后的结果.
 ********************************************************************************/
static int can_recv_reply(can_reply_queue_t *queue, can_motor_reply_t *reply, int timeout_ms)
{
    return can_reply_queue_pop(queue, reply, timeout_ms);
}

/********************************************************************************
 * @brief   打印电机返回帧，便于观察协议内容.
 ********************************************************************************/
static void can_print_reply(const can_motor_reply_t *reply)
{
    uint8_t i;

    if (reply == NULL) {
        return;
    }

    printf("[RX] addr=0x%02X packet=%u code=0x%02X payload:",
           reply->addr, reply->packet, reply->code);

    for (i = 0U; i < reply->payload_len; ++i) {
        printf(" %02X", reply->payload[i]);
    }

    if (reply->payload_len >= 2U) {
        uint8_t status = reply->payload[0];
        uint8_t checksum = reply->payload[reply->payload_len - 1U];

        printf(" status=%s checksum=0x%02X",
               can_reply_status_string(status), checksum);
    }

    printf("\n");
}

int main(void)
{
    ls2k0300_canfd_t canfd;
    can_reply_queue_t reply_queue;
    can_motor_reply_t reply;

    /* 示例 1: 短命令 01 36 6B -> ID 0x0100, data: 36 6B. */
    const uint8_t cmd_short[] = {0x01U, 0x36U, 0x6BU};

    /*
     * 示例 2: 长命令需要拆包:
     * 第 1 包 ID 0x0100, data: FD 01 0F A0 00 00 01 FA
     * 第 2 包 ID 0x0101, data: FD 00 00 00 6B
     */
    const uint8_t cmd_long[] = {
        0x01U, 0xFDU, 0x01U, 0x0FU, 0xA0U, 0x00U, 0x00U, 0x01U,
        0xFAU, 0x00U, 0x00U, 0x00U, 0x6BU
    };
    int ret;

    if (geteuid() != 0) {
        printf("[SKIP] canfd requires root\n");
        return 0;
    }

    if (can_reply_queue_init(&reply_queue) != 0) {
        printf("[FAIL] reply queue init failed\n");
        return 1;
    }

    /*
     * 这里使用 CANFD_MODE_THREAD，后台线程负责 read socket，
     * 回调中解析扩展帧并压入本地队列，主线程只等待队列结果。
     */
    if (ls2k0300_canfd_init(&canfd, CAN0, CANFD_MODE_THREAD,
                            can_reply_rx_callback, &reply_queue) != 0) {
        printf("[FAIL] canfd init failed\n");
        can_reply_queue_deinit(&reply_queue);
        return 1;
    }

    /* 发送短命令后等待一包应答，例如 data: 36 02 6B. */
    ret = can_send_cmd(&canfd, cmd_short, (uint8_t)sizeof(cmd_short));
    if (ret == 0) {
        int rx_ret = can_recv_reply(&reply_queue, &reply, 1000);
        if (rx_ret > 0) {
            can_print_reply(&reply);
        } else if (rx_ret == 0) {
            printf("[WARN] short cmd reply timeout\n");
        } else {
            printf("[WARN] short cmd receive failed\n");
        }
    }

    /* 发送长命令后等待一包应答；动作完成类命令后续可能还会主动返回 9F. */
    if (ret == 0) {
        ret = can_send_cmd(&canfd, cmd_long, (uint8_t)sizeof(cmd_long));
        if (ret == 0) {
            int rx_ret = can_recv_reply(&reply_queue, &reply, 1000);
            if (rx_ret > 0) {
                can_print_reply(&reply);
            } else if (rx_ret == 0) {
                printf("[WARN] long cmd reply timeout\n");
            } else {
                printf("[WARN] long cmd receive failed\n");
            }
        }
    }

    /*
     * 对于 FD/9A 等动作命令，执行完成后电机会主动回:
     * FD 9F 校验码 / 9A 9F 校验码 等。
     * 可以继续等待更长时间接收动作完成帧。
     */
    if (ret == 0) {
        int rx_ret = can_recv_reply(&reply_queue, &reply, 5000);
        if (rx_ret > 0) {
            can_print_reply(&reply);
        } else if (rx_ret == 0) {
            printf("[INFO] no action-done reply\n");
        } else {
            printf("[WARN] action-done receive failed\n");
        }
    }

    /* 释放 SocketCAN 与接收线程资源. */
    ls2k0300_canfd_deinit(&canfd);
    can_reply_queue_deinit(&reply_queue);

    if (ret < 0) {
        printf("[FAIL] can cmd write failed\n");
        return 1;
    }

    if (reply_queue.dropped > 0U) {
        printf("[WARN] dropped replies: %lu\n", (unsigned long)reply_queue.dropped);
    }

    printf("[PASS] can cmd write ok\n");
    return 0;
}
