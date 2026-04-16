#include <stdio.h>

#include "LS2K0300_DRV_INC.h"

static void example_signal_exit_cb(void)
{
    printf("[INFO] SIGINT callback invoked\n");
}

int main(void)
{
    ls2k0300_signal_set_exit_cb(example_signal_exit_cb);
    printf("[PASS] signal callback registered\n");
    return 0;
}
