#include "X42_V2.h"
#include <stdio.h>
#include <string.h>

#define 	FIFO_SIZE   128

uint8_t rxCmd[FIFO_SIZE] = {0};
pthread_cond_t start_receive = PTHREAD_COND_INITIALIZER;
ssize_t rxCount = 0;
ls2k0300_uart_t uart1;
float vel = 0.0, Motor_Vel = 0.0;
pthread_mutex_t uart_mutex_lock = PTHREAD_MUTEX_INITIALIZER;
bool receive_flag = false;
uint16_t FW_ver = 0;
struct timespec ts = {0, 1 * 1000 * 1000};
struct timespec read_ts = {0, 1000 * 1000 * 1000};

void *ls2k0300_uart_thread(void* arg)
{
    while(1)
    {
        // pthread_mutex_lock(&uart_mutex_lock);
        rxCount = ls2k0300_uart_read_var(&uart1, rxCmd, FIFO_SIZE, 1);
        if(rxCount > 0)
        {
            pthread_mutex_lock(&uart_mutex_lock);
            receive_flag = true;
            pthread_cond_signal(&start_receive);
            pthread_mutex_unlock(&uart_mutex_lock);
        }
        // pthread_mutex_unlock(&uart_mutex_lock);
    }

}
void *ls2k0300_setspeed_thread(void* arg)
{
    int set_speed = 0;
    int  set_dir = 0;
    int set_acc = 0;
    while(1)
    {
        scanf("%d,%d", &set_dir, &set_speed);
        ZDT_X42_V2_Velocity_Control(&uart1, 1, set_dir, 10, set_speed, 0);
        nanosleep(&ts, NULL);

    }
}

int main()
{
    ls2k0300_uart_block_init(&uart1, UART1, B115200, LS_UART_STOP1, LS_UART_DATA8, LS_UART_PARITY_NONE,1);
    pthread_t uart1_thread, setspeed_thread;
    pthread_create(&uart1_thread, NULL, ls2k0300_uart_thread, NULL);
    pthread_create(&setspeed_thread, NULL, ls2k0300_setspeed_thread, NULL);

    // pthread_mutex_lock(&uart_mutex_lock);
    // ZDT_X42_V2_Velocity_Control(&uart1, 1, 0, 1, 0, 0);
    // nanosleep(&ts, NULL);

    // pthread_mutex_lock(&uart_mutex_lock);
    // while(!receive_flag) {
    //     pthread_cond_wait(&start_receive, &uart_mutex_lock);
    // }

    // printf("rxCount=%zd :", rxCount);
    // for (ssize_t i = 0; i < rxCount; ++i) {
    //     printf(" %02X", rxCmd[i]);
    // }
    // printf("\n");
    
    // receive_flag = false;
    // rxCount = 0;
    // memset(rxCmd, 0, FIFO_SIZE);
    // pthread_mutex_unlock(&uart_mutex_lock);
    //ls2k0300_uart_write(&uart1, "Hello, World!", 13);
    int set_speed = 0;
    int  set_dir = 0;
    int set_acc = 0;
    while(1)
    {
        ZDT_X42_V2_Read_Sys_Params(&uart1, 1, S_VEL);
        pthread_mutex_lock(&uart_mutex_lock);
        while(!receive_flag) {
            pthread_cond_wait(&start_receive, &uart_mutex_lock);
        }
        receive_flag = false;
        // printf("rxCount=%zd :", rxCount);
        // for (ssize_t i = 0; i < rxCount; ++i) {
        //     printf(" %02X", rxCmd[i]);
        // }
        // printf("\n");
        if(rxCmd[0] == 1 && rxCmd[1] == 0x35 && rxCount == 6)
        {
            // 拼接成uint16_t类型数据
            vel = (uint16_t)(
                            ((uint16_t)rxCmd[3] << 8)   |
                            ((uint16_t)rxCmd[4] << 0)
                            );

            // 实时转速
            Motor_Vel = vel;

            // 符号
            if(rxCmd[2]) { Motor_Vel = -Motor_Vel; }
            // printf("FW Version: %d\n", FW_ver);
            printf("Motor Velocity: %.2f RPM\n", Motor_Vel);
        }

        memset(rxCmd, 0, FIFO_SIZE);
        rxCount = 0;
        pthread_mutex_unlock(&uart_mutex_lock);
        // scanf("%d,%d", &set_dir, &set_speed);
        // ZDT_X42_V2_Velocity_Control(&uart1, 1, set_dir, 10, set_speed, 0);
        // nanosleep(&read_ts, NULL);
        sleep(1);

    }
    pthread_cond_destroy(&start_receive);
    pthread_mutex_destroy(&uart_mutex_lock);


    return 0;
}
