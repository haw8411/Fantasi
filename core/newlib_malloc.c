/* Route newlib's malloc family onto the single FreeRTOS heap (heap_4 / the
 * elastic ucHeap), so libc allocations - notably float printf's dtoa/_Balloc -
 * come from the same pool as pvPortMalloc instead of a separate newlib _sbrk
 * arena. Together with the elastic ucHeap (heap_4.c) and the runtime
 * configTOTAL_HEAP_SIZE (FreeRTOSConfig.h), this lets the app heap span all
 * free RAM with nothing reserved for a second allocator.
 *
 * Enabled per platform with:
 *   -Wl,--wrap=_malloc_r,--wrap=_free_r,--wrap=_calloc_r,--wrap=_realloc_r
 * newlib funnels malloc/calloc/realloc/free through these _r entry points, so
 * wrapping them catches every libc allocation.
 *
 * pvPortMalloc is task-safe (it suspends the scheduler) but NOT ISR-safe, so
 * never call malloc - or printf with %f - from an interrupt. */
#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"

struct _reent;

void *__wrap__malloc_r(struct _reent *r, size_t size)
{
    (void)r;
    return pvPortMalloc(size);
}

void __wrap__free_r(struct _reent *r, void *ptr)
{
    (void)r;
    vPortFree(ptr);
}

void *__wrap__calloc_r(struct _reent *r, size_t count, size_t size)
{
    (void)r;
    size_t total = count * size;
    void *p = pvPortMalloc(total);
    if (p) {
        memset(p, 0, total);
    }
    return p;
}

void *__wrap__realloc_r(struct _reent *r, void *ptr, size_t size)
{
    (void)r;
    if (size == 0) {
        vPortFree(ptr);
        return NULL;
    }
    void *p = pvPortMalloc(size);
    if (p && ptr) {
        memcpy(p, ptr, size);
        vPortFree(ptr);
    }
    return p;
}
