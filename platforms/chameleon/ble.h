#ifndef FANTASI_CHAMELEON_BLE_H
#define FANTASI_CHAMELEON_BLE_H

#include "../../hal/hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- SVC call macro (matches Nordic nrf_svc.h for GCC/ARM) ---- */

#define SVCALL(num, ret, sig)                                          \
    _Pragma("GCC diagnostic push")                                     \
    _Pragma("GCC diagnostic ignored \"-Wreturn-type\"")                \
    __attribute__((naked, unused)) static ret sig {                    \
        __asm("svc %0\nbx lr" : : "I" ((uint16_t)(num)) : "r0");     \
    }                                                                  \
    _Pragma("GCC diagnostic pop")

/* ---- SoftDevice constants ---- */

#define NRF_SUCCESS            0x00
#define NRF_ERROR_NOT_FOUND    0x05
#define NRF_ERROR_RESOURCES    0x13

#define BLE_GAP_ADDR_LEN       6
#define BLE_GAP_EVT_BASE       0x10
#define BLE_GATTS_EVT_BASE     0x50

#define APP_RAM_BASE           0x20005000

/* ---- Common structs (binary-compatible with S140 v7.2.0) ---- */

typedef struct {
    uint8_t source;
    uint8_t rc_ctiv;
    uint8_t rc_temp_ctiv;
    uint8_t accuracy;
} sd_lfclk_cfg_t;

typedef void (*sd_fault_handler_t)(uint32_t id, uint32_t pc, uint32_t info);

typedef struct {
    uint8_t *p_data;
    uint16_t len;
} sd_data_t;

typedef struct {
    uint8_t addr_id_peer : 1;
    uint8_t addr_type    : 7;
    uint8_t addr[BLE_GAP_ADDR_LEN];
} sd_addr_t;

typedef struct {
    uint8_t  extended               : 1;
    uint8_t  report_incomplete_evts : 1;
    uint8_t  active                 : 1;
    uint8_t  filter_policy          : 2;
    uint8_t  scan_phys;
    uint16_t interval;
    uint16_t window;
    uint16_t timeout;
    uint8_t  channel_mask[5];
} sd_scan_params_t;

/* ---- Public API ---- */

bool cu_ble_sd_init(void);
bool cu_ble_sd_is_active(void);

/* Drain pending SoftDevice SoC events (flash completions + USB VBUS events).
 * Defined in flash_storage.c (co-located with the sd_evt_get SVC); the single
 * consumer, called from ble_serial_poll. */
void cu_soc_drain(void);

int  cu_ble_scan(hal_ble_scan_cb_t cb, uint32_t duration_ms);
void cu_ble_radio_info(hal_radio_info_t *info);

#endif
