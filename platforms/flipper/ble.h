#ifndef FANTASI_FLIPPER_BLE_H
#define FANTASI_FLIPPER_BLE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t addr[6];
    uint8_t addr_type;
    int8_t  rssi;
    char    name[29];
} ble_scan_result_t;

typedef void (*ble_scan_cb_t)(const ble_scan_result_t *result);

typedef enum {
    BLE_EVT_CONNECTED,
    BLE_EVT_DISCONNECTED,
    BLE_EVT_PASSKEY_REQUEST,
    BLE_EVT_PASSKEY_DISPLAY,
    BLE_EVT_PAIR_COMPLETE,
} ble_evt_type_t;

typedef struct {
    ble_evt_type_t type;
    uint16_t conn_handle;
    uint8_t  status;
    uint8_t  reason;
    uint32_t passkey;
} ble_event_t;

#define BLE_IO_CAP_DISPLAY_ONLY     0x00
#define BLE_IO_CAP_DISPLAY_YES_NO   0x01
#define BLE_IO_CAP_KEYBOARD_ONLY    0x02
#define BLE_IO_CAP_NO_IO            0x03
#define BLE_IO_CAP_KEYBOARD_DISPLAY 0x04

bool     ble_init(void);
void     ble_shutdown(void);
bool     ble_is_active(void);
bool     ble_cpu2_running(void);
int      ble_flash_erase_activity(int on);
void     ble_activate_fus(void);
int      ble_scan(ble_scan_cb_t cb, uint32_t duration_ms);

int      ble_pair_setup_security(uint8_t io_cap);
void     ble_pair_set_manual(bool on);
int      ble_pair_connect(const uint8_t *addr, uint8_t addr_type);
int      ble_pair_initiate(uint16_t conn_handle);
int      ble_pair_send_passkey(uint16_t conn_handle, uint32_t passkey);
int      ble_pair_numeric_confirm(uint16_t conn_handle, bool accept);
int      ble_pair_wait_event(ble_event_t *evt, uint32_t timeout_ms);
int      ble_pair_disconnect(uint16_t conn_handle);
uint32_t ble_generate_passkey(void);

#define BLE_MAX_CONN 2

typedef struct {
    uint16_t handle;
    uint8_t  addr[6];
    uint8_t  addr_type;
    bool     active;
} ble_conn_info_t;

int ble_get_connections(ble_conn_info_t *out, int max);

typedef struct {
    uint8_t addr[6];
    uint8_t addr_type;
} ble_bonded_dev_t;

int  ble_get_bonded_devices(ble_bonded_dev_t *out, int max);
int  ble_remove_bond(const uint8_t *addr, uint8_t addr_type);
int  ble_clear_all_bonds(void);

#endif
