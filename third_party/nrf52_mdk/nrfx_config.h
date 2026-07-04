/* Fantasi - minimal nrfx_config.h replacement.
 * The stock nrfx_config.h redirects to sdk_config.h, which is the
 * giant nRF SDK feature-flag file. We don't build any nrfx drivers
 * (TinyUSB talks directly to the USBD/POWER/CLOCK peripherals via
 * the HAL headers), so an empty config is enough. */
#ifndef NRFX_CONFIG_H__
#define NRFX_CONFIG_H__

/* Define NRFX_ASSERT to a no-op so HAL inline helpers compile without
 * dragging in the SDK's logger. */
#define NRFX_ASSERT(expr)  ((void)(expr))
#define NRFX_STATIC_ASSERT(expr)  _Static_assert((expr), #expr)

#endif
