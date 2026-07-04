#include "log.h"
#include "cli.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

#define LOG_ENTRY_MAX  128
#define LOG_RING_SIZE  2048

static char ring[LOG_RING_SIZE];
static volatile uint16_t ring_head;
static volatile uint16_t ring_tail;
static SemaphoreHandle_t log_mutex;

static const char *level_tag[] = { "T", "D", "I", "W", "E" };

void fantasi_log_init(void)
{
    log_mutex = xSemaphoreCreateMutex();
}

void fantasi_log(log_level_t level, const char *fmt, ...)
{
    if (!log_mutex) return;

    char buf[LOG_ENTRY_MAX];
    TickType_t ticks = xTaskGetTickCount();
    uint32_t sec = ticks / configTICK_RATE_HZ;
    uint32_t ms  = (ticks % configTICK_RATE_HZ) * 1000 / configTICK_RATE_HZ;
    int pos = snprintf(buf, sizeof(buf), "[%07lu.%03lu] [%s] ",
                       (unsigned long)sec, (unsigned long)ms,
                       level_tag[level < LOG_ERROR ? level : LOG_ERROR]);

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf + pos, sizeof(buf) - pos - 2, fmt, ap);
    va_end(ap);

    if (n < 0) return;
    pos += n;
    if (pos > LOG_ENTRY_MAX - 6) {
        pos = LOG_ENTRY_MAX - 6;
        buf[pos++] = '.';
        buf[pos++] = '.';
        buf[pos++] = '.';
    }
    buf[pos++] = '\r';
    buf[pos++] = '\n';

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
        xSemaphoreTake(log_mutex, portMAX_DELAY);

    for (int i = 0; i < pos; i++) {
        ring[ring_head] = buf[i];
        ring_head = (ring_head + 1) % LOG_RING_SIZE;
        if (ring_head == ring_tail)
            ring_tail = (ring_tail + 1) % LOG_RING_SIZE;
    }

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
        xSemaphoreGive(log_mutex);
}


void fantasi_log_stream(void)
{
    cli_ctx_t *ctx = cli_current_ctx();
    if (!ctx) return;

    uint16_t cursor = ring_tail;
    char line[LOG_ENTRY_MAX + 4];

    for (;;) {
        if (ctx->transport.poll) ctx->transport.poll();
        /* Drain ALL pending input each pass (not one byte) and scan for ^C, so a
         * ^C is honoured within one iteration regardless of preceding bytes -
         * reading one byte per 100 ms tick made ^C detection hopelessly slow. */
        uint8_t in[64];
        size_t got = ctx->transport.read(in, sizeof(in), ctx->transport.ctx);
        for (size_t i = 0; i < got; i++)
            if (in[i] == 0x03) return;
        /* End the stream if the controlling link drops - otherwise a host that
         * closed the port (or a BLE peer that went away) without sending ^C
         * leaves this loop running forever, stalling the CLI task: it can never
         * get back to servicing commands and the serial looks dead. */
        if (ctx->transport.connected && !ctx->transport.connected(ctx->transport.ctx))
            return;

        xSemaphoreTake(log_mutex, portMAX_DELAY);

        int lpos = 0;
        while (cursor != ring_head && lpos < (int)sizeof(line) - 1) {
            line[lpos++] = ring[cursor];
            cursor = (cursor + 1) % LOG_RING_SIZE;
            if (line[lpos - 1] == '\n') {
                line[lpos] = '\0';
                cli_write(line);
                lpos = 0;
            }
        }

        xSemaphoreGive(log_mutex);

        if (ctx->transport.flush) ctx->transport.flush();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
