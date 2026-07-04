#ifndef FANTASI_LOG_H
#define FANTASI_LOG_H

#include <stdarg.h>

typedef enum {
    LOG_TRACE,
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR
} log_level_t;

void fantasi_log_init(void);
void fantasi_log(log_level_t level, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));
void fantasi_log_stream(void);

#endif
