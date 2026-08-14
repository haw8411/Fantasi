#ifndef FANTASI_PROTO_PIPE_H
#define FANTASI_PROTO_PIPE_H

#include <stdint.h>
#include <stddef.h>

typedef size_t (*proto_pipe_write_fn)(const uint8_t *buf, size_t len, void *ctx);
typedef void   (*proto_pipe_poll_fn)(void);

#define PROTO_PIPE_BUF_SIZE 512

typedef struct {
    uint8_t          buf[PROTO_PIPE_BUF_SIZE];
    uint16_t         len;
    uint16_t         mtu_payload;
    proto_pipe_write_fn write_fn;
    proto_pipe_poll_fn  poll_fn;
    void             *write_ctx;
} proto_pipe_t;

void   proto_pipe_init(proto_pipe_t *p, proto_pipe_write_fn wfn,
                     proto_pipe_poll_fn pfn, void *ctx, uint16_t mtu_payload);
void   proto_pipe_set_mtu(proto_pipe_t *p, uint16_t att_mtu);
size_t proto_pipe_write(const uint8_t *buf, size_t len, void *pipe_ptr);
void   proto_pipe_flush(proto_pipe_t *p);

#endif
