#ifndef _ENCODER_H_
#define _ENCODER_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "stdlib.h"

#define ENCODER_PROC_PATH "/proc/encoder_count"
#define ENCODER_SAVE_PATH "/home/root/encoder_count.txt"

typedef struct {
    long long current_count;
    long long last_count;
    long long diff_count;

} EncoderData;

long long Read_encoder(void);
int Reset_encoder(void);
int Save_encoder(long long count);
long long Load_encoder(void);

void Init_encoder_data(EncoderData *data);
void Get_encoder_data(EncoderData *data);

#ifdef __cplusplus
}
#endif


#endif /* _ENCODER_H_ */