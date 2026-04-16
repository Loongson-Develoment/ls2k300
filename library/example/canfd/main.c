#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "LS2K0300_DRV_INC.h"

int main(void)
{
    ls2k0300_canfd_t canfd;
    ls2k0300_canfd_frame_t frame;
    int ret;

    if (geteuid() != 0) {
        printf("[SKIP] canfd requires root\n");
        return 0;
    }

    if (ls2k0300_canfd_init(&canfd, CAN0, CANFD_MODE_THREAD, NULL, NULL) != 0) {
        printf("[FAIL] canfd init failed\n");
        return 1;
    }

    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x123U;
    frame.len = 8U;
    frame.data[0] = 1U;
    frame.data[1] = 2U;
    frame.data[2] = 3U;
    frame.data[3] = 4U;
    frame.data[4] = 5U;
    frame.data[5] = 6U;
    frame.data[6] = 7U;
    frame.data[7] = 8U;

    ret = ls2k0300_canfd_write_frame(&canfd, &frame);
    ls2k0300_canfd_deinit(&canfd);

    if (ret < 0) {
        printf("[FAIL] canfd write failed\n");
        return 1;
    }

    printf("[PASS] canfd init/write/deinit ok\n");
    return 0;
}

