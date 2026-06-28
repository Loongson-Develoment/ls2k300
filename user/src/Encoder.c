#include "Encoder.h"


long long Read_encoder(void)
{
    FILE *fp;
    long long count = 0;

    fp = fopen(ENCODER_PROC_PATH, "r");
    if (fp == NULL) {
        perror("Failed to open " ENCODER_PROC_PATH);
        return -1;
    }

    if (fscanf(fp, "%lld", &count) != 1) {
        perror("Failed to read encoder count");
        fclose(fp);
        return -1;
    }

    fclose(fp);
    return count;

}

int Reset_encoder(void)
{
    FILE *fp;
    fp = fopen(ENCODER_PROC_PATH, "w");
    if (fp == NULL) {
        perror("Failed to open " ENCODER_PROC_PATH " for writing");
        return -1;
    }

    fprintf(fp, "0\n");
    fclose(fp);
    return 0;
}


int Save_encoder(long long count)
{
    FILE *fp;
    fp = fopen(ENCODER_SAVE_PATH, "w");
    if (fp == NULL) {
        perror("Failed to open " ENCODER_SAVE_PATH " for writing");
        return -1;
    }

    fprintf(fp, "%lld\n", count);
    
    fflush(fp);
    // fsync(fileno(fp));

    fclose(fp);
    return 0;
}

long long Load_encoder(void)
{
    FILE *fp;
    long long count = 0;

    fp = fopen(ENCODER_SAVE_PATH, "r");
    if (fp == NULL) {
        perror("Failed to open " ENCODER_SAVE_PATH " for reading");
        return -1;
    }

    if (fscanf(fp, "%lld", &count) != 1) {
        perror("Failed to read encoder count from file");
        fclose(fp);
        return -1;
    }

    fclose(fp);
    return count;
}


void Init_encoder_data(EncoderData *data)
{
    data->current_count = 0;
    data->last_count = 0;
    data->diff_count = 0.0f;

}


void Get_encoder_data(EncoderData *data)
{
    
    data->last_count = data->current_count;
    data->current_count = Read_encoder();;
    data->diff_count = data->current_count - data->last_count;

}