/* C-library glue for the picolibc.
 *
 * 1. Route the malloc family onto the single FreeRTOS heap (heap_4 / the elastic
 *    ucHeap) so libc allocations, including float printf's conversion buffers,
 *    share the pvPortMalloc pool. Enabled per platform with
 *      -Wl,--wrap=malloc,--wrap=free,--wrap=calloc,--wrap=realloc
 *    pvPortMalloc is task-safe (it suspends the scheduler) but not ISR-safe, so
 *    never call malloc, or printf with %f, from an interrupt.
 *
 * 2. The bare-metal stubs picolibc needs from the platform: assert() resets the
 *    device, and the clock hooks report FreeRTOS ticks scaled to CLOCKS_PER_SEC. */
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <sys/times.h>
#include <sys/time.h>

#include "FreeRTOS.h"    /* FreeRTOSConfig.h declares __heap_end__ (heap top) */
#include "task.h"

/* ---- malloc family -> the one FreeRTOS heap ---- */
void *__wrap_malloc(size_t size) { return pvPortMalloc(size); }
void  __wrap_free(void *ptr)     { vPortFree(ptr); }

void *__wrap_calloc(size_t count, size_t size)
{
    /* Reject count*size overflow with NULL instead of returning an undersized
     * buffer. */
    if (size && count > (size_t)-1 / size) return NULL;
    size_t total = count * size;
    void *p = pvPortMalloc(total);
    if (p) memset(p, 0, total);
    return p;
}

void *__wrap_realloc(void *ptr, size_t size)
{
    if (size == 0) { vPortFree(ptr); return NULL; }
    void *p = pvPortMalloc(size);
    if (p && ptr) {
        /* heap_4 exposes no old-block size, so bound the copy by the heap top: a
         * grow of a block near __heap_end__ must not read past it into unmapped
         * memory. Bytes beyond the old allocation are undefined per realloc's
         * contract, so copying a few extra in-heap bytes into the new block's
         * tail is harmless. */
        size_t copy = size;
        size_t avail = (size_t)(&__heap_end__ - (unsigned char *)ptr);
        if (copy > avail) copy = avail;
        memcpy(p, ptr, copy);
        vPortFree(ptr);
    }
    return p;
}

/* ---- picolibc thread-local storage (errno, rand state, localtime buffer) ----
 * picolibc keeps errno, the rand()/random() state (_rand_next) and _localtime_buf
 * in TLS, reached through __aeabi_read_tp(). With -nostartfiles there is no crt0
 * to point the thread register at a RAM TLS block, so provide one fixed block and
 * hand it out (our strong __aeabi_read_tp wins over the library's). Without a
 * valid thread pointer, TLS accesses target low memory and corrupt the vector
 * table. One block serves the whole firmware, since the libc runs a single
 * context (configUSE_NEWLIB_REENTRANT 0). Zero-init is fine: errno starts 0, and
 * the PRNG seeds from 0 because the .tdata image is not copied, which only shifts
 * math.rand()'s output before srand().
 *
 * Sizing: an 8-byte ARM EABI TCB precedes the data, so a variable at TLS offset X
 * lands at block + 8 + X; _rand_next/_localtime_buf/errno reach +0x38. 128 leaves
 * headroom, and the ASSERT in each platform's linker.ld fails the build if
 * .tdata+.tbss ever outgrows it instead of corrupting .bss. */
/* used: the only reference is the naked asm below, invisible to the compiler. */
static unsigned char fantasi_tls_block[128] __attribute__((aligned(8), used));

/* __aeabi_read_tp has a special AAPCS: it may clobber only r0, ip, lr and the
 * flags, because the caller keeps live values in r1-r3 across TLS accesses.
 * Hand-write it in asm to guarantee that: load the block address into r0 and
 * return. Valid in both ARM and Thumb; the literal pool auto-emits after bx. */
__attribute__((naked)) void *__aeabi_read_tp(void)
{
    __asm volatile("ldr r0, =fantasi_tls_block\n\tbx lr");
}

/* ---- picolibc bare-metal stubs ---- */
extern void fantasi_reset(void);   /* core/main.c: reset so the device self-recovers */

/* assert() failure - only LittleFS's LFS_ASSERT reaches this. Reset rather than
 * pull in picolibc's default __assert_func -> stderr/fputs/abort/_exit chain. */
void __assert_func(const char *file, int line, const char *func, const char *expr)
{
    (void)file; (void)line; (void)func; (void)expr;
    fantasi_reset();
    for (;;);
}

void _exit(int code) { (void)code; fantasi_reset(); for (;;); }

/* Berry's time.clock()/time.time() reach these. No RTC, so wall-clock reads back
 * as the epoch and process time tracks FreeRTOS ticks. picolibc's clock() sums
 * these tms fields and callers divide by CLOCKS_PER_SEC (1e6 here, not the 1 kHz
 * tick rate), so scale ticks into CLOCKS_PER_SEC units. Multiply before dividing,
 * in 64 bits, so the result holds for any CLOCKS_PER_SEC and the scale factor
 * never rounds to a zero divisor. */
clock_t times(struct tms *buf)
{
    clock_t t = (clock_t)((uint64_t)xTaskGetTickCount() * CLOCKS_PER_SEC / configTICK_RATE_HZ);
    if (buf) { buf->tms_utime = t; buf->tms_stime = 0; buf->tms_cutime = 0; buf->tms_cstime = 0; }
    return t;
}

int gettimeofday(struct timeval *tv, void *tz)
{
    (void)tz;
    if (tv) { tv->tv_sec = 0; tv->tv_usec = 0; }
    return 0;
}
