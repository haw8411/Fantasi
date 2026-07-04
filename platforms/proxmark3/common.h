/* Fantasi / Proxmark3 - shim for common.h referenced by ported PM3
 * sources. Upstream common.h is full of host-side path macros we don't
 * need; this pulls in just the stdint/bool types the ARM code assumes. */
#ifndef FANTASI_PM3_COMMON_H
#define FANTASI_PM3_COMMON_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#endif
