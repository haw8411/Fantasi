#ifndef FANTASI_HAL_H
#define FANTASI_HAL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Minimal HAL contract every platform must implement.
 * Platform code brings up its serial transport (USB-CDC) and heap; the
 * core CLI/task code uses only these entry points. */

/* Called once from main(), before the scheduler starts. Must leave the
 * transport ready to enqueue bytes; actual host attach can happen later. */
void hal_init(void);

/* Called once from cli_task after the scheduler is running. Platforms
 * use this for init that needs FreeRTOS primitives (e.g. BLE stack). */
void hal_post_init(void);

/* Non-blocking: returns number of bytes written (0..len). Safe to call
 * from a FreeRTOS task context. */
size_t hal_serial_write(const uint8_t *buf, size_t len);

/* Non-blocking: returns number of bytes read into buf (0..len). */
size_t hal_serial_read(uint8_t *buf, size_t len);

/* True when a USB-CDC host is connected and DTR is asserted. Platforms
 * without DTR semantics may always return true once enumerated. */
bool hal_serial_connected(void);

/* Bytes currently free on the FreeRTOS heap. Thin wrapper so the "free"
 * command doesn't need to know which heap_N.c the platform chose. */
size_t hal_free_heap_bytes(void);

/* Smallest free-heap figure observed since boot. Useful alongside the
 * instantaneous value. 0 if the platform's heap can't report it. */
size_t hal_min_ever_free_heap_bytes(void);

/* Short, stable identifier for the current build target. One of:
 * "FZ" (Flipper Zero), "PM3" (Proxmark3), "CU" (Chameleon Ultra). */
const char *hal_device_id(void);

/* Unique pseudoword name derived from hardware UID. Deterministic,
 * no storage needed. Writes at most len bytes including '\0'. */
const char *hal_device_name(void);

/* Bytes of on-chip flash not occupied by firmware. Returns -1 when the
 * platform cannot determine this (e.g. no linker/register support). */
int32_t hal_flash_free_bytes(void);

/* Battery level as a percentage (0-100). Returns -1 when the platform
 * has no battery sense circuitry or the reading is unavailable. */
int hal_battery_percent(void);

/* Clean warm reboot of the MCU. Does not return. */
void hal_reboot(void) __attribute__((noreturn));

/* Reboot into the platform's bootloader / DFU entry path so the device
 * can be reflashed by the host. Does not return. */
void hal_reboot_dfu(void) __attribute__((noreturn));

/* Arm the DFU magic so the next reset (however caused) lands in DFU. */
void hal_set_dfu_magic(void);

/* Power the device off. Does not return on success (the device stays powered
 * down until a wake source resets it). On failure it returns one of the
 * HAL_SHUTDOWN_* codes:
 *   - HAL_SHUTDOWN_UNSUPPORTED: platform has no off state (e.g. PM3).
 *   - HAL_SHUTDOWN_USB_POWERED:  externally powered right now, so the battery
 *     rail can't be cut (FZ: the BQ25896 keeps the system alive from VBUS).
 *     Unplug and retry. */
#define HAL_SHUTDOWN_UNSUPPORTED (-1)
#define HAL_SHUTDOWN_USB_POWERED (-2)
int hal_shutdown(void);

/* True while the platform's designated power button is physically held
 * (CU: button B; FZ: back button). Polled by the core power-button monitor
 * to detect a long hold. Implemented only where FANTASI_ENABLE_PWR_BUTTON
 * is set (CU, FZ); PM3 has no power button. */
bool hal_shutdown_button_held(void);

/* Per-SRAM-region memory info. Platforms with multiple physical SRAM
 * banks (e.g. STM32WB55) report each one; single-bank parts may return
 * 0 (no regions to report). */
typedef struct {
    const char *name;       /* e.g. "SRAM1", "SRAM2a" */
    uint32_t    total;      /* region size in bytes */
    uint32_t    free;       /* free/unallocated bytes; 0 if N/A */
    const char *note;       /* NULL or short annotation */
} hal_mem_region_t;

#define HAL_MEM_REGIONS_MAX 4

int hal_mem_regions(hal_mem_region_t *out, int max);

/* Regions of SRAM safe to write for diagnostics. Each entry is a
 * contiguous span not used by any linker section, heap, stack, or
 * CPU2 secure area. Platforms return 0 if not implemented. */
typedef struct {
    const char *name;
    uint32_t    addr;
    uint32_t    size;       /* bytes, word-aligned */
} hal_test_region_t;

#define HAL_TEST_REGIONS_MAX 4

int hal_test_regions(hal_test_region_t *out, int max);

/* Radio coprocessor diagnostic info (STM32WB CM0+, nRF SoftDevice, …).
 * Platforms without a radio coprocessor set available = false.
 * TODO: Has a few stub entries. */
typedef struct {
    bool     available;
    uint32_t secure_flash_start;
    uint32_t secure_flash_kb;
    uint32_t fus_major, fus_minor, fus_sub;
    uint32_t ws_major, ws_minor, ws_sub;
    uint32_t ws_type;
} hal_radio_info_t;

void hal_radio_info(hal_radio_info_t *info);

/* Activate FUS on CPU2, replacing the wireless stack.
 * Sends SHCI_C2_FUS_GetState twice (ST AN5185 protocol).
 * No-op on platforms without a radio coprocessor. */
void hal_ble_activate_fus(void);

/* Request USB mode switch to MSC (storage) mode. Returns 0 if the
 * switch will happen, -1 if MSC already runs alongside CDC (no switch
 * needed), or -2 if storage is unavailable. On platforms that need
 * mode switching (PM3), the USB task handles the actual disconnect /
 * reconnect cycle. */
int hal_enter_msc_mode(void);

/* Request USB mode switch to WebUSB (vendor protobuf) mode / back to CDC serial.
 * Returns 0 if a switch will happen, -1 if the vendor interface already runs
 * alongside CDC (composite targets - no switch needed). Switch-mode targets
 * (PM3) re-enumerate; composite targets (FZ/CU) return -1. */
int hal_enter_webusb_mode(void);
int hal_enter_cdc_mode(void);

/* ---- USB HID keyboard emulation (app-driven; weak-default to unsupported
 * in core/app_run.c, real implementation per platform) ---- */
/* Arm the keyboard for use (on=1) or release held keys (on=0). On the composite
 * targets the keyboard is a persistent interface, so this is just a readiness
 * wait; the endpoint-scarce PM3 re-enumerates as a HID-only device here (and
 * back to CDC on on=0). Returns 0, or -1 if HID isn't supported / no host. */
int      hal_hid_enable(int on);
/* Send one keyboard report: `modifiers` held with up to `n` (<=6) key usages.
 * n=0 releases all keys. Waits for the endpoint to drain. Returns 0, or -1. */
int      hal_hid_send(uint8_t modifiers, const uint8_t *keys, uint8_t n);
/* Host hint bits (keyboard-LED output report + mount state); 0 when unknown. */
uint32_t hal_hid_host(void);

/* Select the composite HID mode (true = persistent, false = switch). Applied to
 * the descriptor before enumeration (read from the `hid` setting at boot) and on
 * a live change from the settings menu. No-op where HID isn't supported. */
void     hal_hid_set_persistent(bool persistent);

/* Enable/disable the USB mass-storage interface (true = present, the default).
 * Applied to the descriptor before enumeration (read from the `msc` setting at
 * boot) and on a live change. No-op where MSC isn't supported. */
void     hal_msc_set_enabled(bool enabled);

/* Request the USB device task to re-enumerate (tud_disconnect + reconnect) so a
 * descriptor change - switch-mode arming, or a mode change - takes effect.
 * Implemented in the shared TinyUSB serial task (FZ/CU). */
void     hal_usb_reenumerate(void);

/* BLE scan callback - called for each discovered device.
 * addr is 6 bytes, name may be empty (""). */
typedef void (*hal_ble_scan_cb_t)(const uint8_t *addr, uint8_t addr_type,
                                   int8_t rssi, const char *name);

/* Scan for BLE devices for duration_ms. Returns device count, or -1
 * if the platform has no BLE radio or initialisation failed. */
int hal_ble_scan(hal_ble_scan_cb_t cb, uint32_t duration_ms);

/* BLE pairing event types. */
typedef enum {
    HAL_BLE_EVT_CONNECTED,
    HAL_BLE_EVT_DISCONNECTED,
    HAL_BLE_EVT_PASSKEY_REQUEST,
    HAL_BLE_EVT_PASSKEY_DISPLAY,
    HAL_BLE_EVT_PAIR_COMPLETE,
} hal_ble_evt_type_t;

typedef struct {
    hal_ble_evt_type_t type;
    uint16_t conn_handle;
    uint8_t  status;
    uint8_t  reason;
    uint32_t passkey;
} hal_ble_evt_t;

#define HAL_BLE_IO_DISPLAY_ONLY     0x00
#define HAL_BLE_IO_DISPLAY_YES_NO   0x01
#define HAL_BLE_IO_KEYBOARD_ONLY    0x02
#define HAL_BLE_IO_NO_IO            0x03
#define HAL_BLE_IO_KEYBOARD_DISPLAY 0x04

/* Configure IO capability and authentication requirements for pairing.
 * Must be called before connect or advertise. Returns 0 on success. */
int hal_ble_pair_setup(uint8_t io_cap);

/* Route pairing events to pair_queue for the CLI pair command.
 * Call hal_ble_pair_begin before waiting on hal_ble_pair_wait.
 * Call hal_ble_pair_end when the pair command exits (any path). */
void hal_ble_pair_begin(void);
void hal_ble_pair_end(void);

/* Connect to a peer device by MAC address (central role).
 * Returns 0 if connection attempt started, -1 on error. */
int hal_ble_pair_connect(const uint8_t *addr, uint8_t addr_type);

/* Stop BLE: disconnect all peers, stop advertising. */
void hal_ble_shutdown(void);

/* Read a key from settings.cfg into buf (max len). Returns strlen or -1. */
int hal_settings_get(const char *key, char *buf, int len);

/* Write a key=value pair to settings.cfg. Returns 0 on success. */
int hal_settings_set(const char *key, const char *value);

/* Remove a key from settings.cfg (no-op if absent). Returns 0 on success. */
int hal_settings_unset(const char *key);

/* Iterate settings line by line (streamed - no whole-file buffer), calling
 * cb(line, ctx) for each non-empty "key=value" line. Returns 0, or -1 if
 * storage is unavailable. */
int hal_settings_foreach(void (*cb)(const char *line, void *ctx), void *ctx);

/* True if the BLE stack is initialised and active. */
bool hal_ble_is_active(void);

/* Initiate pairing on an established connection. */
int hal_ble_pair_initiate(uint16_t conn_handle);

/* Respond to a passkey request with the given 6-digit passkey. */
int hal_ble_pair_passkey(uint16_t conn_handle, uint32_t passkey);

/* Confirm numeric comparison (yes=1, no=0). */
int hal_ble_pair_confirm(uint16_t conn_handle, bool accept);

/* Wait for the next BLE pairing event, with timeout in ms.
 * Returns 0 if an event was received, -1 on timeout. */
int hal_ble_pair_wait(hal_ble_evt_t *evt, uint32_t timeout_ms);

/* Disconnect from a peer. */
int hal_ble_disconnect(uint16_t conn_handle);

/* Generate a cryptographically random 6-digit passkey (0-999999)
 * using hardware RNG. */
uint32_t hal_ble_generate_passkey(void);

/* Active BLE connection info. */
typedef struct {
    uint16_t handle;
    uint8_t  addr[6];
    uint8_t  addr_type;
} hal_ble_conn_info_t;

/* List active BLE connections. Returns count (0 if none). */
int hal_ble_connections(hal_ble_conn_info_t *out, int max);

/* Bonded (paired) device info. */
typedef struct {
    uint8_t addr[6];
    uint8_t addr_type;
} hal_ble_bonded_t;

/* List bonded devices stored in the BLE stack's secure flash. */
int hal_ble_get_bonded(hal_ble_bonded_t *out, int max);

/* Remove bond for a specific device. Returns 0 on success. */
int hal_ble_remove_bond(const uint8_t *addr, uint8_t addr_type);

/* Clear all bonds. */
int hal_ble_clear_bonds(void);

#endif
