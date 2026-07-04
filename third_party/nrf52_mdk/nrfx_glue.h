/* Fantasi - minimal nrfx_glue.h replacement.
 *
 * Stock nrfx_glue.h drags in half the nRF SDK (legacy config, app_util,
 * soc IRQ tables, the logger). We don't build any nrfx drivers - the
 * only consumer of these macros is TinyUSB's dcd_nrf5x.c, and it only
 * uses the ASSERT + IRQ priority helpers. This stub provides those
 * and nothing else, breaking the SDK include chain entirely.
 */
#ifndef NRFX_GLUE_H__
#define NRFX_GLUE_H__

#include "nrf.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NRFX_ASSERT(expr)                   ((void)(expr))
#define NRFX_STATIC_ASSERT(expr)            _Static_assert((expr), #expr)

#define NRFX_IRQ_PRIORITY_SET(irq, prio)    NVIC_SetPriority((irq), (prio))
#define NRFX_IRQ_ENABLE(irq)                NVIC_EnableIRQ((irq))
#define NRFX_IRQ_DISABLE(irq)               NVIC_DisableIRQ((irq))
#define NRFX_IRQ_PENDING_CLEAR(irq)         NVIC_ClearPendingIRQ((irq))
#define NRFX_IRQ_IS_ENABLED(irq)            (NVIC->ISER[(irq) >> 5] & (1UL << ((irq) & 0x1F)))

#define NRFX_CRITICAL_SECTION_ENTER()       { uint32_t _ps = __get_PRIMASK(); __disable_irq();
#define NRFX_CRITICAL_SECTION_EXIT()          if (!_ps) __enable_irq(); }

#define NRFX_DELAY_US(us_time)              do { volatile uint32_t _n = (us_time) * 16; while (_n--) __NOP(); } while (0)
#define NRFX_DELAY_DWT_BASED                0

#define NRFX_ATOMIC_FETCH_STORE(p, v)       __atomic_exchange_n((p), (v), __ATOMIC_SEQ_CST)
#define NRFX_ATOMIC_FETCH_OR(p, v)          __atomic_fetch_or((p), (v), __ATOMIC_SEQ_CST)
#define NRFX_ATOMIC_FETCH_AND(p, v)         __atomic_fetch_and((p), (v), __ATOMIC_SEQ_CST)
#define NRFX_ATOMIC_FETCH_XOR(p, v)         __atomic_fetch_xor((p), (v), __ATOMIC_SEQ_CST)
#define NRFX_ATOMIC_FETCH_ADD(p, v)         __atomic_fetch_add((p), (v), __ATOMIC_SEQ_CST)
#define NRFX_ATOMIC_FETCH_SUB(p, v)         __atomic_fetch_sub((p), (v), __ATOMIC_SEQ_CST)

#define NRFX_CUSTOM_ERROR_CODES             0

#ifdef __cplusplus
}
#endif

#endif
