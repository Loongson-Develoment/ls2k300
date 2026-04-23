#ifndef __LS2K0300_V4L2_H
#define __LS2K0300_V4L2_H

#include <stddef.h>
#include <stdint.h>
#include <sys/time.h>

#include <linux/videodev2.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LS2K0300_V4L2_MAX_BUFFERS  (8U)

/********************************************************************************
 * @brief   V4L2 mmap 缓冲区描述.
 ********************************************************************************/
typedef struct {
    void   *start;
    size_t  length;
} ls2k0300_v4l2_buffer_t;

/********************************************************************************
 * @brief   V4L2 摄像头句柄.
 ********************************************************************************/
typedef struct {
    int fd;
    int initialized;
    int streaming;

    uint32_t width;
    uint32_t height;
    uint32_t pixelformat;
    uint32_t fps_num;
    uint32_t fps_den;

    uint32_t buffer_count;
    ls2k0300_v4l2_buffer_t buffers[LS2K0300_V4L2_MAX_BUFFERS];
} ls2k0300_v4l2_t;

/********************************************************************************
 * @brief   V4L2 已出队帧描述.
 ********************************************************************************/
typedef struct {
    const uint8_t *data;
    size_t bytesused;
    uint32_t index;
    uint32_t sequence;
    struct timeval timestamp;
} ls2k0300_v4l2_frame_t;

/********************************************************************************
 * @brief   打开并初始化 V4L2 mmap 采集.
 * @param   cam          : 摄像头句柄.
 * @param   device       : 设备路径，例如 "/dev/video0".
 * @param   width        : 请求宽度.
 * @param   height       : 请求高度.
 * @param   pixelformat  : 请求像素格式，例如 V4L2_PIX_FMT_MJPEG.
 * @param   fps          : 请求帧率，0 表示不设置.
 * @param   buffer_count : mmap 缓冲数量，0 表示默认 4.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int ls2k0300_v4l2_init(ls2k0300_v4l2_t *cam,
                       const char *device,
                       uint32_t width,
                       uint32_t height,
                       uint32_t pixelformat,
                       uint32_t fps,
                       uint32_t buffer_count);

/********************************************************************************
 * @brief   释放 V4L2 采集资源.
 * @param   cam : 摄像头句柄.
 ********************************************************************************/
void ls2k0300_v4l2_deinit(ls2k0300_v4l2_t *cam);

/********************************************************************************
 * @brief   阻塞等待并取出一帧.
 * @param   cam        : 摄像头句柄.
 * @param   frame      : 输出帧描述.
 * @param   timeout_ms : 等待超时，负数表示无限等待.
 * @return  成功返回 0，失败返回 -1，超时返回 1.
 * @note    使用完 frame 后必须调用 ls2k0300_v4l2_enqueue().
 ********************************************************************************/
int ls2k0300_v4l2_dequeue(ls2k0300_v4l2_t *cam,
                          ls2k0300_v4l2_frame_t *frame,
                          int timeout_ms);

/********************************************************************************
 * @brief   将已处理帧重新放回 V4L2 队列.
 * @param   cam   : 摄像头句柄.
 * @param   frame : 已出队帧描述.
 * @return  成功返回 0，失败返回 -1.
 ********************************************************************************/
int ls2k0300_v4l2_enqueue(ls2k0300_v4l2_t *cam,
                          const ls2k0300_v4l2_frame_t *frame);

/********************************************************************************
 * @brief   将 FOURCC 转为 4 字节字符串.
 * @param   fourcc : V4L2 FOURCC.
 * @param   out    : 输出字符串缓冲区，长度至少 5.
 ********************************************************************************/
void ls2k0300_v4l2_fourcc_to_string(uint32_t fourcc, char out[5]);

#ifdef __cplusplus
}
#endif

#endif
