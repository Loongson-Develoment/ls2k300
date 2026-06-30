#ifndef APP_LOG_H
#define APP_LOG_H

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

static inline FILE *app_log_stream_for_level(const char *level)
{
    return (level != NULL && level[0] == 'E') ? stderr : stdout;
}

static inline void app_log_write(const char *level, const char *fmt, ...)
{
    FILE *stream = app_log_stream_for_level(level);
    va_list args;

    fprintf(stream, "[%s] ", level);
    va_start(args, fmt);
    vfprintf(stream, fmt, args);
    va_end(args);
    fprintf(stream, "\n");
    fflush(stream);
}

static inline void app_log_bytes(const char *level,
                                 const char *prefix,
                                 const uint8_t *data,
                                 size_t size)
{
    FILE *stream = app_log_stream_for_level(level);

    fprintf(stream, "[%s] %s size=%zu, data:", level, prefix, size);
    for (size_t i = 0U; i < size; ++i) {
        fprintf(stream, " %02X", data[i]);
    }
    fprintf(stream, "\n");
    fflush(stream);
}

#define APP_LOG_INFO(...)  app_log_write("INFO", __VA_ARGS__)
#define APP_LOG_WARN(...)  app_log_write("WARN", __VA_ARGS__)
#define APP_LOG_ERROR(...) app_log_write("ERROR", __VA_ARGS__)

#endif
