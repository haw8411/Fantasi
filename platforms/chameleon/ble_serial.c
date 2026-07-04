/* Fantasi / Chameleon Ultra - BLE GATT serial service via SoftDevice S140.
 *
 * Registers a GATT service with RX (write) and TX (notify) characteristics
 * using the same UUIDs as the Flipper for host CLI compatibility. Provides
 * the cli_transport_t interface for the nanopb proto task. */

#include "ble_serial.h"
#include "ble.h"
#include "nrf.h"
#include "../../core/cli.h"
#include "../../core/log.h"
#include "../../hal/hal.h"
#include "hal_storage.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>

/* ---- SVC numbers ---- */

#define SVC_BLE_UUID_VS_ADD           0x62
#define SVC_BLE_GATTS_SERVICE_ADD     0xA8
#define SVC_BLE_GATTS_CHAR_ADD        0xAA
#define SVC_BLE_GATTS_HVX             0xAE
#define SVC_BLE_GAP_ADV_SET_CONFIGURE 0x72
#define SVC_BLE_GAP_ADV_START         0x73
#define SVC_BLE_GAP_DISCONNECT        0x76
#define SVC_BLE_GAP_CONN_PARAM_UPDATE 0x75
#define SVC_BLE_GAP_DATA_LENGTH_UPDATE 0x90
#define SVC_BLE_GATTS_SYS_ATTR_SET    0xB1
#define SVC_BLE_GAP_SEC_PARAMS_REPLY  0x7F
#define SVC_BLE_GATTS_EXCHANGE_MTU_REPLY 0xB5
#define SVC_BLE_EVT_GET               0x61

/* ---- BLE constants ---- */

#define BLE_UUID_TYPE_VENDOR_BEGIN  2
#define BLE_GATTS_VLOC_STACK       1
#define BLE_GATT_HVX_NOTIFICATION  1
#define BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED 0x01
#define BLE_CONN_HANDLE_INVALID    0xFFFF

#define BLE_GAP_EVT_CONNECTED          0x10
#define BLE_GAP_EVT_DISCONNECTED       0x11
#define BLE_GAP_EVT_SEC_PARAMS_REQUEST 0x13
#define BLE_GAP_EVT_AUTH_STATUS        0x19
#define BLE_GAP_EVT_ADV_SET_TERMINATED 0x26
#define BLE_GATTS_EVT_WRITE            0x50
#define BLE_GATTS_EVT_HVN_TX_COMPLETE  0x57

/* ---- S140 structs ---- */

typedef struct { uint8_t uuid128[16]; } ble_uuid128_t;
typedef struct { uint16_t uuid; uint8_t type; } ble_uuid_t;

typedef struct {
    uint8_t sm : 4;
    uint8_t lv : 4;
} ble_gap_conn_sec_mode_t;

typedef struct {
    ble_gap_conn_sec_mode_t read_perm;
    ble_gap_conn_sec_mode_t write_perm;
    uint8_t vlen    : 1;
    uint8_t vloc    : 2;
    uint8_t rd_auth : 1;
    uint8_t wr_auth : 1;
} ble_gatts_attr_md_t;

typedef struct {
    uint8_t broadcast       : 1;
    uint8_t read            : 1;
    uint8_t write_wo_resp   : 1;
    uint8_t write           : 1;
    uint8_t notify          : 1;
    uint8_t indicate        : 1;
    uint8_t auth_signed_wr  : 1;
} ble_gatt_char_props_t;

typedef struct {
    uint8_t reliable_wr : 1;
    uint8_t wr_aux      : 1;
} ble_gatt_char_ext_props_t;

typedef struct {
    ble_gatt_char_props_t      char_props;
    ble_gatt_char_ext_props_t  char_ext_props;
    uint8_t const             *p_char_user_desc;
    uint16_t                   char_user_desc_max_size;
    uint16_t                   char_user_desc_size;
    void const                *p_char_pf;
    ble_gatts_attr_md_t const *p_user_desc_md;
    ble_gatts_attr_md_t const *p_cccd_md;
    ble_gatts_attr_md_t const *p_sccd_md;
} ble_gatts_char_md_t;

typedef struct {
    ble_uuid_t const          *p_uuid;
    ble_gatts_attr_md_t const *p_attr_md;
    uint16_t                   init_len;
    uint16_t                   init_offs;
    uint16_t                   max_len;
    uint8_t                   *p_value;
} ble_gatts_attr_t;

typedef struct {
    uint16_t value_handle;
    uint16_t user_desc_handle;
    uint16_t cccd_handle;
    uint16_t sccd_handle;
} ble_gatts_char_handles_t;

typedef struct {
    uint16_t       handle;
    uint8_t        type;
    uint16_t       offset;
    uint16_t      *p_len;
    uint8_t const *p_data;
} ble_gatts_hvx_params_t;

typedef struct {
    uint8_t type;
    uint8_t _padding[3];
} ble_gap_adv_properties_t;

typedef struct {
    uint8_t *p_data;
    uint16_t len;
} ble_data_t;

typedef struct {
    ble_data_t adv_data;
    ble_data_t scan_rsp_data;
} ble_gap_adv_data_t;

typedef struct {
    ble_gap_adv_properties_t properties;
    void const              *p_peer_addr;
    uint32_t                 interval;
    uint16_t                 duration;
    uint8_t                  max_adv_evts;
    uint8_t                  channel_mask[5];
    uint8_t                  filter_policy;
    uint8_t                  primary_phy;
    uint8_t                  secondary_phy;
    uint8_t                  set_id       : 4;
    uint8_t                  scan_req_notification : 1;
} ble_gap_adv_params_t;

typedef struct {
    uint8_t bond     : 1;
    uint8_t mitm     : 1;
    uint8_t lesc     : 1;
    uint8_t keypress : 1;
    uint8_t io_caps  : 3;
    uint8_t oob      : 1;
    uint8_t min_key_size;
    uint8_t max_key_size;
    uint8_t kdist_own;
    uint8_t kdist_peer;
} ble_gap_sec_params_t;

/* ---- Bonding key structs (S140 ble_gap.h layout) ---- */

#define BLE_SEC_KEY_LEN  16
#define BLE_SEC_RAND_LEN 8

typedef struct {
    uint8_t ltk[BLE_SEC_KEY_LEN];
    uint8_t lesc    : 1;
    uint8_t auth    : 1;
    uint8_t ltk_len : 6;
} ble_gap_enc_info_t;

typedef struct {
    uint16_t ediv;
    uint8_t  rand[BLE_SEC_RAND_LEN];
} ble_gap_master_id_t;

typedef struct { uint8_t irk[BLE_SEC_KEY_LEN]; }  ble_gap_irk_t;
typedef struct { uint8_t csrk[BLE_SEC_KEY_LEN]; } ble_gap_sign_info_t;

typedef struct {
    ble_gap_enc_info_t  enc_info;
    ble_gap_master_id_t master_id;
} ble_gap_enc_key_t;

typedef struct {
    ble_gap_irk_t irk;
    sd_addr_t     id_addr;
} ble_gap_id_key_t;

typedef struct {
    ble_gap_enc_key_t   *p_enc_key;
    ble_gap_id_key_t    *p_id_key;
    ble_gap_sign_info_t *p_sign_key;
    void                *p_pk;
} ble_gap_sec_keys_t;

typedef struct {
    ble_gap_sec_keys_t keys_own;
    ble_gap_sec_keys_t keys_peer;
} ble_gap_sec_keyset_t;

_Static_assert(sizeof(ble_gap_enc_info_t)  == 17, "enc_info layout");
_Static_assert(sizeof(ble_gap_master_id_t) == 10, "master_id layout");
_Static_assert(sizeof(ble_gap_enc_key_t)   == 28, "enc_key layout");

/* ---- SVC stubs ---- */

SVCALL(SVC_BLE_UUID_VS_ADD,           uint32_t, svc_uuid_vs_add(const ble_uuid128_t *uuid128, uint8_t *p_type))
SVCALL(SVC_BLE_GATTS_SERVICE_ADD,     uint32_t, svc_gatts_service_add(uint8_t type, const ble_uuid_t *uuid, uint16_t *handle))
SVCALL(SVC_BLE_GATTS_CHAR_ADD,        uint32_t, svc_gatts_char_add(uint16_t svc_handle, const ble_gatts_char_md_t *md, const ble_gatts_attr_t *attr, ble_gatts_char_handles_t *handles))
SVCALL(SVC_BLE_GATTS_HVX,            uint32_t, svc_gatts_hvx(uint16_t conn, ble_gatts_hvx_params_t *p))
SVCALL(SVC_BLE_GAP_ADV_SET_CONFIGURE, uint32_t, svc_gap_adv_set_configure(uint8_t *handle, const ble_gap_adv_data_t *data, const ble_gap_adv_params_t *params))
SVCALL(SVC_BLE_GAP_ADV_START,         uint32_t, svc_gap_adv_start(uint8_t handle, uint8_t conn_cfg_tag))
SVCALL(SVC_BLE_GAP_DISCONNECT,        uint32_t, svc_gap_disconnect(uint16_t conn, uint8_t reason))
SVCALL(SVC_BLE_GAP_SEC_PARAMS_REPLY,  uint32_t, svc_gap_sec_params_reply(uint16_t conn, uint8_t status, const ble_gap_sec_params_t *own, const ble_gap_sec_keyset_t *keyset))
SVCALL(0x86,                          uint32_t, svc_gap_sec_info_reply(uint16_t conn, const ble_gap_enc_info_t *enc, const ble_gap_irk_t *id, const ble_gap_sign_info_t *sign))
SVCALL(0x80,                          uint32_t, svc_gap_auth_key_reply(uint16_t conn, uint8_t key_type, const uint8_t *key))
SVCALL(0x74,                          uint32_t, svc_gap_adv_stop_pair(uint8_t adv_h))
SVCALL(0x8C,                          uint32_t, svc_gap_connect(const sd_addr_t *addr, const void *scan_params, const void *conn_params, uint8_t tag))
SVCALL(0x7E,                          uint32_t, svc_gap_authenticate(uint16_t conn, const ble_gap_sec_params_t *params))
SVCALL(SVC_BLE_GAP_CONN_PARAM_UPDATE, uint32_t, svc_gap_conn_param_update(uint16_t conn, const void *params))
SVCALL(0x8F,                          uint32_t, svc_gap_phy_update(uint16_t conn, const void *phys))
SVCALL(SVC_BLE_GAP_DATA_LENGTH_UPDATE,uint32_t, svc_gap_data_length_update(uint16_t conn, const void *params, void *limitation))
SVCALL(SVC_BLE_GATTS_EXCHANGE_MTU_REPLY, uint32_t, svc_gatts_exchange_mtu_reply(uint16_t conn, uint16_t server_rx_mtu))
SVCALL(SVC_BLE_GATTS_SYS_ATTR_SET,   uint32_t, svc_gatts_sys_attr_set(uint16_t conn, const void *attrs, uint16_t len, uint32_t flags))
SVCALL(SVC_BLE_EVT_GET,               uint32_t, svc_ble_evt_get(uint8_t *buf, uint16_t *len))
SVCALL(0x7C,                          uint32_t, svc_gap_device_name_set(const void *perm, const uint8_t *name, uint16_t len))

/* ---- UUIDs ----
 * Use Nordic UART Service (NUS) UUIDs on CU since the SoftDevice's
 * VS UUID model (16-bit substitution at LE [12:13]) can't represent
 * the Flipper's UUID scheme. The host CLI resolves by name, then
 * discovers characteristics by UUID. */

static const uint8_t nus_base[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E
};

#define NUS_SVC_UUID16 0x0001
#define NUS_RX_UUID16  0x0002
#define NUS_TX_UUID16  0x0003

/* ---- State ---- */

static uint8_t  uuid_type;
static uint16_t svc_handle;
static ble_gatts_char_handles_t rx_handles, tx_handles;
static volatile uint16_t conn_handle = BLE_CONN_HANDLE_INVALID;
uint8_t  adv_handle = 0xFF;
static bool     service_ready;

/* Whether the serial GATT service is advertising / connectable. The
 * SoftDevice itself stays enabled for the lifetime of the device (scanning
 * shares it), so "BLE active" cannot mean "SD enabled" - it tracks whether
 * we are advertising. `ble off` clears this and stops advertising; the
 * auto-restart paths (disconnect, adv-set-terminated) honor it so a peer
 * cannot bring advertising back after the user disabled it. */
static bool     serial_enabled;

/* Central-mode pairing state */
static volatile bool     pair_in_progress;
static QueueHandle_t     pair_queue;
#define PAIR_QUEUE_LEN 8

/* ---- Bonding (persisted across reboot so a paired host reconnects without
 * re-pairing). Single bond: only one peripheral link exists at a time. ----
 *
 * The SoftDevice fills `s_keyset` ASYNCHRONOUSLY between sec_params_reply and
 * AUTH_STATUS, so the keyset and its key buffers MUST be static (not stack).
 * On AUTH_STATUS success we copy our distributed enc key (LTK+EDIV+RAND) into
 * s_bond and flag a deferred flash write - the write can't run inside the
 * event handler (it pumps ble_serial_poll re-entrantly). On reconnect the
 * central sends our EDIV/RAND in a SEC_INFO_REQUEST and we reply with the
 * stored LTK. */
#define BLE_BOND_PATH    "/ble_bond.bin"
#define BLE_BOND_MAGIC   0x424E4431  /* "BND1" */

typedef struct {
    uint32_t          magic;
    sd_addr_t         peer;
    ble_gap_enc_key_t enc;   /* our LTK + master_id (EDIV/RAND) */
} ble_bond_t;

static ble_gap_enc_key_t  s_own_enc, s_peer_enc;
static ble_gap_sec_keyset_t s_keyset;
static ble_bond_t         s_bond;          /* valid iff magic == BLE_BOND_MAGIC */
static sd_addr_t          s_conn_peer;     /* peer addr of the live connection */
static volatile bool      bond_save_pending;

static bool bond_valid(void) { return s_bond.magic == BLE_BOND_MAGIC; }

static void bond_load(void)
{
    ble_bond_t b;
    if (hal_storage_read_file(BLE_BOND_PATH, &b, sizeof(b)) == (int)sizeof(b) &&
        b.magic == BLE_BOND_MAGIC)
        s_bond = b;
    else
        s_bond.magic = 0;
}

/* Deferred: called from ble_serial_poll's tail, never inside an event
 * handler (the flash write re-enters ble_serial_poll to keep draining). */
static void bond_save(void)
{
    hal_storage_write_file(BLE_BOND_PATH, &s_bond, sizeof(s_bond));
}

/* Must hold one full framed CliRequest (max ~599 B): with MTU 247 +
 * write-without-response the host bursts a whole FileWriteChunk as several
 * back-to-back write commands, which ble_serial_poll drains into this
 * buffer in one pass before the proto task reads it; a smaller buffer
 * overflows and drops uploads. Heap-allocated, so this costs heap not BSS. */
#define RX_BUF_SIZE 4096
static StreamBufferHandle_t rx_stream;
static volatile int tx_credits = 4;
static uint16_t att_mtu = 23;

/* Advertising data buffers (must persist - SD references them) */
static uint8_t adv_buf[31];
static uint8_t sr_buf[31];

/* ---- Advertising ---- */

static void start_advertising(void)
{
    /* Build adv data: flags + service UUID */
    int pos = 0;
    adv_buf[pos++] = 2;
    adv_buf[pos++] = 0x01; /* AD_TYPE_FLAGS */
    adv_buf[pos++] = 0x06; /* LE General Discoverable + BR/EDR Not Supported */

    /* Device name: "Fantasi <device_name>" */
    char name[28] = "Fantasi ";
    const char *devname = hal_device_name();
    if (devname && devname[0])
        strncat(name, devname, sizeof(name) - strlen(name) - 1);
    uint8_t nlen = (uint8_t)strlen(name);
    if (nlen > 28) nlen = 28;
    adv_buf[pos++] = nlen + 1;
    adv_buf[pos++] = 0x09; /* AD_TYPE_COMPLETE_LOCAL_NAME */
    memcpy(&adv_buf[pos], name, nlen);
    pos += nlen;

    ble_gap_adv_data_t adv_data;
    memset(&adv_data, 0, sizeof(adv_data));
    adv_data.adv_data.p_data = adv_buf;
    adv_data.adv_data.len = (uint16_t)pos;

    ble_gap_adv_params_t params;
    memset(&params, 0, sizeof(params));
    params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    params.interval = 160;   /* 100ms in 625µs units */
    params.duration = 0;     /* forever */
    params.primary_phy = 1;  /* 1 Mbps */

    uint32_t arc = svc_gap_adv_set_configure(&adv_handle, &adv_data, &params);
    if (arc != NRF_SUCCESS) return;
    /* tag 1: the conn_cfg configured with MTU 247. tag 0 (default cfg)
     * over-reserves links here and makes adv_start fail silently. */
    svc_gap_adv_start(adv_handle, 1);
}

/* Scan active flag - when set, ble_serial_poll yields event draining
 * to the scan loop running on the CLI task. */
volatile bool ble_scan_active;

/* ---- SWI2 handler (SoftDevice event signal) ---- */

static volatile bool sd_evt_pending;

void SWI2_EGU2_IRQHandler(void)
{
    sd_evt_pending = true;
}

/* ---- Init ---- */

int ble_serial_init(void)
{
    if (!cu_ble_sd_init()) return -1;

    rx_stream = xStreamBufferCreate(RX_BUF_SIZE, 1);
    if (!rx_stream) return -2;
    pair_queue = xQueueCreate(PAIR_QUEUE_LEN, sizeof(hal_ble_evt_t));

    ble_uuid128_t base;
    memcpy(base.uuid128, nus_base, 16);
    uint32_t rc = svc_uuid_vs_add(&base, &uuid_type);
    if (rc != NRF_SUCCESS) return -3;

    ble_uuid_t svc_uuid = { .uuid = NUS_SVC_UUID16, .type = uuid_type };
    rc = svc_gatts_service_add(1 /* primary */, &svc_uuid, &svc_handle);
    if (rc != NRF_SUCCESS) return -4;

    /* RX characteristic (client writes to device) */
    ble_gatts_char_md_t rx_md;
    memset(&rx_md, 0, sizeof(rx_md));
    rx_md.char_props.write = 1;
    rx_md.char_props.write_wo_resp = 1;

    /* sm=1 lv=3: MITM-authenticated encryption. The client cannot write
     * commands (or, below, subscribe to / read responses) until the link is
     * encrypted by an authenticated passkey pairing - i.e. the PIN is
     * required to use the CLI. lv=1 (open) left the service usable unpaired. */
    ble_gatts_attr_md_t rx_attr_md;
    memset(&rx_attr_md, 0, sizeof(rx_attr_md));
    rx_attr_md.read_perm = (ble_gap_conn_sec_mode_t){ .sm = 1, .lv = 3 };
    rx_attr_md.write_perm = (ble_gap_conn_sec_mode_t){ .sm = 1, .lv = 3 };
    rx_attr_md.vloc = BLE_GATTS_VLOC_STACK;

    ble_uuid_t rx_uuid = { .uuid = NUS_RX_UUID16, .type = uuid_type };
    ble_gatts_attr_t rx_attr;
    memset(&rx_attr, 0, sizeof(rx_attr));
    rx_attr.p_uuid = &rx_uuid;
    rx_attr.p_attr_md = &rx_attr_md;
    rx_attr.max_len = 244;

    rc = svc_gatts_char_add(svc_handle, &rx_md, &rx_attr, &rx_handles);
    if (rc != NRF_SUCCESS) return -5;

    /* TX characteristic (device notifies client) */
    ble_gatts_char_md_t tx_md;
    memset(&tx_md, 0, sizeof(tx_md));
    tx_md.char_props.notify = 1;

    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    cccd_md.read_perm = (ble_gap_conn_sec_mode_t){ .sm = 1, .lv = 1 };
    cccd_md.write_perm = (ble_gap_conn_sec_mode_t){ .sm = 1, .lv = 3 };
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    tx_md.p_cccd_md = &cccd_md;

    ble_gatts_attr_md_t tx_attr_md;
    memset(&tx_attr_md, 0, sizeof(tx_attr_md));
    tx_attr_md.read_perm = (ble_gap_conn_sec_mode_t){ .sm = 1, .lv = 3 };
    tx_attr_md.vloc = BLE_GATTS_VLOC_STACK;

    ble_uuid_t tx_uuid = { .uuid = NUS_TX_UUID16, .type = uuid_type };
    ble_gatts_attr_t tx_attr;
    memset(&tx_attr, 0, sizeof(tx_attr));
    tx_attr.p_uuid = &tx_uuid;
    tx_attr.p_attr_md = &tx_attr_md;
    tx_attr.max_len = 244;

    rc = svc_gatts_char_add(svc_handle, &tx_md, &tx_attr, &tx_handles);
    if (rc != NRF_SUCCESS) return -6;

    /* Set GAP device name */
    {
        ble_gap_conn_sec_mode_t name_perm = { .sm = 1, .lv = 1 };
        const char *devname = "Fantasi";
        svc_gap_device_name_set(&name_perm, (const uint8_t *)devname, 7);
    }

    /* Enable SWI2 so the SD can signal pending BLE events. */
    NVIC_SetPriority(SWI2_EGU2_IRQn, 6);
    NVIC_EnableIRQ(SWI2_EGU2_IRQn);

    bond_load();

    service_ready = true;
    serial_enabled = true;
    start_advertising();
    return 0;
}

/* ---- Event handling (called from ble_serial_poll) ---- */

static void handle_event(const uint8_t *evt_buf, uint16_t evt_len)
{
    (void)evt_len;
    uint16_t evt_id = *(const uint16_t *)&evt_buf[0];
    uint16_t ch     = *(const uint16_t *)&evt_buf[4];

    switch (evt_id) {
    case BLE_GAP_EVT_CONNECTED:
        conn_handle = ch;
        /* peer_addr is the first field of the connected-event params (GAP
         * params sit at offset 8 after the 2-byte conn_handle padding). */
        memcpy(&s_conn_peer, &evt_buf[8], sizeof(s_conn_peer));
        svc_gatts_sys_attr_set(ch, NULL, 0, 0);
        svc_gap_data_length_update(ch, NULL, NULL);
        /* Throughput: request 2M PHY (nRF52840 + most hosts support it,
         * ~2x over-the-air rate) and a fast 15ms connection interval.
         * Both are requests; the central may negotiate them down. */
        {
            uint8_t phys[3] = { 2, 2, 0 }; /* tx=2M, rx=2M */
            svc_gap_phy_update(ch, phys);
            uint16_t cp[4] = { 12, 12, 0, 400 }; /* 15ms fixed, 4s timeout */
            svc_gap_conn_param_update(ch, cp);
        }
        if (pair_in_progress && pair_queue) {
            hal_ble_evt_t pe = { .type = HAL_BLE_EVT_CONNECTED,
                                 .conn_handle = ch, .status = 0 };
            xQueueSend(pair_queue, &pe, 0);
        }
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        conn_handle = BLE_CONN_HANDLE_INVALID;
        tx_credits = 4;
        att_mtu = 23;
        if (pair_in_progress && pair_queue) {
            hal_ble_evt_t pe = { .type = HAL_BLE_EVT_DISCONNECTED,
                                 .conn_handle = ch };
            xQueueSend(pair_queue, &pe, 0);
        }
        /* Resume advertising unless the user turned BLE off; if `ble off`
         * triggered this disconnect, restarting here would re-enable it. */
        if (serial_enabled)
            svc_gap_adv_start(adv_handle, 1);
        break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST: {
        if (pair_in_progress) {
            /* Initiator: already sent params via sd_ble_gap_authenticate.
             * Reply with NULL to accept the responder's params. */
            svc_gap_sec_params_reply(ch, 0, NULL, NULL);
        } else {
            /* Responder: provide our own params + a keyset so the SoftDevice
             * distributes (generates) our LTK for bonding. kdist enc on both
             * sides; only our own enc key is persisted. The key buffers are
             * static - the SD fills them asynchronously before AUTH_STATUS. */
            ble_gap_sec_params_t own;
            memset(&own, 0, sizeof(own));
            own.bond = 1;
            own.mitm = 1;
            own.lesc = 0;
            own.io_caps = 0; /* DISPLAY_ONLY */
            own.min_key_size = 7;
            own.max_key_size = 16;
            own.kdist_own  = 0x01; /* enc: distribute our LTK */
            own.kdist_peer = 0x01; /* enc */

            memset(&s_keyset, 0, sizeof(s_keyset));
            s_keyset.keys_own.p_enc_key  = &s_own_enc;
            s_keyset.keys_peer.p_enc_key = &s_peer_enc;
            svc_gap_sec_params_reply(ch, 0, &own, &s_keyset);
        }
        break;
    }

    case BLE_GAP_EVT_CONNECTED + 3 + 1: /* SEC_INFO_REQUEST (0x14) */
        /* A bonded central is reconnecting: answer with the stored LTK so the
         * link re-encrypts without re-pairing. No bond → reject (re-pair). */
        if (bond_valid())
            svc_gap_sec_info_reply(ch, &s_bond.enc.enc_info, NULL, NULL);
        else
            svc_gap_sec_info_reply(ch, NULL, NULL, NULL);
        break;

    case 0x12: /* CONN_PARAM_UPDATE */
    case 0x22: /* PHY_UPDATE */
    case 0x24: /* DATA_LENGTH_UPDATE */
        break;

    case 0x1F: /* CONN_PARAM_UPDATE_REQUEST (0x10+15) */
        svc_gap_conn_param_update(ch, &evt_buf[8]);
        break;

    case 0x23: /* DATA_LENGTH_UPDATE_REQUEST (0x10+19) */
        svc_gap_data_length_update(ch, NULL, NULL);
        break;

    case 0x21: { /* PHY_UPDATE_REQUEST (0x10+17) */
        uint8_t phys[3] = { 0, 0, 0 }; /* 0 = no preference, accept peer's */
        svc_gap_phy_update(ch, phys);
        break;
    }

    case 0x52: /* BLE_GATTS_EVT_SYS_ATTR_MISSING */
        svc_gatts_sys_attr_set(ch, NULL, 0, 0);
        break;

    case 0x55: { /* BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST */
        uint16_t client_mtu = *(const uint16_t *)&evt_buf[6];
        uint16_t our_mtu = 247;
        svc_gatts_exchange_mtu_reply(ch, our_mtu);
        att_mtu = (client_mtu < our_mtu) ? client_mtu : our_mtu;
        extern void ble_proto_set_mtu(uint16_t);
        ble_proto_set_mtu(att_mtu);
        break;
    }

    case 0x57: { /* BLE_GATTS_EVT_HVN_TX_COMPLETE */
        uint8_t count = evt_buf[6];
        tx_credits += count;
        break;
    }

    case 0x1A: /* CONN_SEC_UPDATE */
        break;

    case 0x15: { /* PASSKEY_DISPLAY - passkey at offset 8 (2-byte padding) */
        char pk[7];
        memcpy(pk, &evt_buf[8], 6);
        pk[6] = '\0';
        uint32_t pkval = (uint32_t)strtoul(pk, NULL, 10);
        fantasi_log(LOG_INFO, "BLE pair code: %s", pk);
        if (pair_in_progress && pair_queue) {
            hal_ble_evt_t pe = { .type = HAL_BLE_EVT_PASSKEY_DISPLAY,
                                 .conn_handle = ch, .passkey = pkval };
            xQueueSend(pair_queue, &pe, 0);
        } else {
            uint8_t match_req = evt_buf[14] & 1;
            if (match_req)
                svc_gap_auth_key_reply(ch, 1, &evt_buf[8]);
        }
        break;
    }

    case 0x17: /* AUTH_KEY_REQUEST */
        if (pair_in_progress && pair_queue) {
            hal_ble_evt_t pe = { .type = HAL_BLE_EVT_PASSKEY_REQUEST,
                                 .conn_handle = ch };
            xQueueSend(pair_queue, &pe, 0);
        }
        break;

    case 0x19: { /* AUTH_STATUS - params at offset 8 (after padding) */
        uint8_t status = evt_buf[8];
        uint8_t bonded = (evt_buf[9] >> 2) & 1; /* error_src:2, bonded:1 */
        pair_in_progress = false;
        fantasi_log(LOG_INFO, "BLE pair %s (0x%02x)",
                    status == 0 ? "ok" : "failed", status);
        if (status == 0 && bonded) {
            /* The SD filled s_own_enc with our LTK+EDIV+RAND during key
             * distribution. Persist it (deferred - see bond_save_pending). */
            s_bond.magic = BLE_BOND_MAGIC;
            s_bond.peer  = s_conn_peer;
            s_bond.enc   = s_own_enc;
            bond_save_pending = true;
        }
        if (pair_queue) {
            hal_ble_evt_t pe = { .type = HAL_BLE_EVT_PAIR_COMPLETE,
                                 .conn_handle = ch, .status = status };
            xQueueSend(pair_queue, &pe, 0);
        }
        break;
    }

    case BLE_GAP_EVT_ADV_SET_TERMINATED:
        if (serial_enabled && conn_handle == BLE_CONN_HANDLE_INVALID)
            start_advertising();
        break;

    case BLE_GATTS_EVT_WRITE: {
        /* GATTS events: no padding after conn_handle.
         * [0:1]evt_id [2:3]evt_len [4:5]conn_handle
         * [6:7]write.handle [8:11]write.uuid [12]op [13]auth
         * [14:15]offset [16:17]len [18:]data */
        uint16_t attr_handle = *(const uint16_t *)&evt_buf[6];
        if (attr_handle == rx_handles.value_handle) {
            uint16_t dlen = *(const uint16_t *)&evt_buf[16];
            const uint8_t *data = &evt_buf[18];
            if (dlen > 0 && rx_stream)
                xStreamBufferSend(rx_stream, data, dlen, 0);
        }
        break;
    }

    default:
        break;
    }
}

/* ---- Transport interface ---- */

/* The task that drives ble_serial_poll() (the BLE proto/CLI task). Recorded so
 * the flash-completion wait can tell whether it is running in this task - only
 * then may it re-pump ble_serial_poll() (the bond-save re-entrancy case).
 * Pumping it from a different task (the USB MSC flush) would run ble_serial_poll
 * concurrently with this task - it is not reentrant across tasks - which hangs
 * the flash wait and stalls USB. See wait_flash_event() in flash_storage.c. */
static volatile TaskHandle_t s_poll_task;

bool ble_serial_on_poll_task(void)
{
    return s_poll_task != NULL && xTaskGetCurrentTaskHandle() == s_poll_task;
}

void ble_serial_poll(void)
{
    s_poll_task = xTaskGetCurrentTaskHandle();

    /* Drain SoftDevice SoC events whenever the SD is resident - this delivers
     * flash-op completions and USB VBUS events. It must run even when serial
     * isn't ready (e.g. `ble off` with the SD still up, or before boot bring-up
     * finishes) so a cable plugged in after boot still enumerates USB. */
    if (cu_ble_sd_is_active())
        cu_soc_drain();

    /* Boot bring-up (honoring the persisted on/off setting) is driven by
     * hal_post_init() → ble_serial_resume(); until that sets service_ready
     * there is nothing to pump, and `ble off` leaves it false on purpose. */
    if (!service_ready)
        return;

    sd_evt_pending = false;

    if (ble_scan_active) return;

    /* Must hold the largest BLE event: a GATTS write of a full MTU-247
     * payload (244 B) produces a ~264 B event, so the buffer must exceed that
     * or svc_ble_evt_get returns NRF_ERROR_DATA_SIZE and the oversized event
     * stalls the drain loop (large writes never delivered). 320 covers MTU 247
     * with margin. */
    __attribute__((aligned(4))) uint8_t evt[320];
    for (;;) {
        uint16_t len = sizeof(evt);
        uint32_t rc = svc_ble_evt_get(evt, &len);
        if (rc != NRF_SUCCESS) break;
        handle_event(evt, len);
    }

    /* Deferred bond write (flagged in the AUTH_STATUS handler). Done here, at
     * the poll's top level rather than inside handle_event, because the flash
     * write re-enters ble_serial_poll to keep draining events. The flag is
     * cleared first so that re-entrant call doesn't recurse into the write. */
    if (bond_save_pending) {
        bond_save_pending = false;
        bond_save();
    }
}

size_t ble_serial_read(uint8_t *buf, size_t len, void *ctx)
{
    (void)ctx;
    return xStreamBufferReceive(rx_stream, buf, len, 0);
}

size_t ble_serial_write(const uint8_t *buf, size_t len, void *ctx)
{
    (void)ctx;
    if (!service_ready || conn_handle == BLE_CONN_HANDLE_INVALID || len == 0)
        return 0;

    uint16_t payload = att_mtu - 3;
    uint16_t chunk = (len > payload) ? payload : (uint16_t)len;

    if (tx_credits <= 0) {
        for (int i = 0; i < 200; i++) {
            ble_serial_poll();
            if (tx_credits > 0) break;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        if (tx_credits <= 0) return 0;
    }

    static uint8_t hvx_buf[244];
    memset(hvx_buf, 0, payload);
    memcpy(hvx_buf, buf, chunk);
    uint16_t hvx_len = payload;

    ble_gatts_hvx_params_t hvx;
    memset(&hvx, 0, sizeof(hvx));
    hvx.handle = tx_handles.value_handle;
    hvx.type = BLE_GATT_HVX_NOTIFICATION;
    hvx.p_len = &hvx_len;
    hvx.p_data = hvx_buf;

    tx_credits--;
    uint32_t rc = svc_gatts_hvx(conn_handle, &hvx);
    if (rc != NRF_SUCCESS)
        return 0;
    return chunk;
}

bool ble_serial_connected(void *ctx)
{
    (void)ctx;
    return service_ready && conn_handle != BLE_CONN_HANDLE_INVALID;
}

/* ---- Central-mode pairing (HAL implementation) ---- */

int ble_pair_setup_security(uint8_t io_cap)
{
    (void)io_cap;
    /* This is the path `ble on` takes (via hal_ble_pair_setup), so it must
     * (re)start advertising, not merely ensure the SoftDevice is up. */
    return ble_serial_resume();
}

/* ---- Serial service on/off (advertising) ---- */

int ble_serial_resume(void)
{
    if (!cu_ble_sd_init()) return -1;
    if (!service_ready)
        return ble_serial_init();   /* registers the service + advertises */
    if (!serial_enabled) {
        serial_enabled = true;
        start_advertising();
    }
    return 0;
}

void ble_serial_stop(void)
{
    serial_enabled = false;
    if (!service_ready) return;
    if (conn_handle != BLE_CONN_HANDLE_INVALID)
        svc_gap_disconnect(conn_handle, 0x13); /* REMOTE_USER_TERMINATED */
    if (adv_handle != 0xFF)
        svc_gap_adv_stop_pair(adv_handle);
}

bool ble_serial_is_active(void)
{
    return serial_enabled;
}

void ble_pair_set_manual(bool on) { pair_in_progress = on; }


int ble_pair_connect(const uint8_t *addr, uint8_t addr_type)
{
    if (!cu_ble_sd_init()) return -1;

    sd_addr_t peer;
    memset(&peer, 0, sizeof(peer));
    peer.addr_type = addr_type;
    memcpy(peer.addr, addr, 6);

    /* Stop advertising - can't connect while connectable advertising */
    extern uint8_t adv_handle;
    if (adv_handle != 0xFF)
        svc_gap_adv_stop_pair(adv_handle);

    /* Use sd_scan_params_t which matches the S140 binary layout */
    sd_scan_params_t sp;
    memset(&sp, 0, sizeof(sp));
    sp.scan_phys = 1;    /* 1 Mbps */
    sp.interval  = 96;   /* 60ms */
    sp.window    = 48;   /* 30ms */
    sp.timeout   = 500;  /* 5s */

    /* Connection params: 4 x uint16_t */
    uint16_t cp[4] = { 24, 40, 0, 400 };

    uint32_t rc = svc_gap_connect(&peer, &sp, cp, 0);
    return (rc == NRF_SUCCESS) ? 0 : -1;
}

int ble_pair_initiate(uint16_t conn_h)
{
    ble_gap_sec_params_t params;
    memset(&params, 0, sizeof(params));
    params.bond = 1;
    params.mitm = 1;
    params.lesc = 0;
    params.io_caps = 2; /* KEYBOARD_ONLY - we enter passkey displayed by peer */
    params.min_key_size = 7;
    params.max_key_size = 16;
    uint32_t rc = svc_gap_authenticate(conn_h, &params);
    return (rc == NRF_SUCCESS) ? 0 : -1;
}

int ble_pair_send_passkey(uint16_t conn_h, uint32_t passkey)
{
    uint8_t pk[6];
    for (int i = 5; i >= 0; i--) {
        pk[i] = '0' + (passkey % 10);
        passkey /= 10;
    }
    uint32_t rc = svc_gap_auth_key_reply(conn_h, 1, pk);
    return (rc == NRF_SUCCESS) ? 0 : -1;
}

int ble_pair_numeric_confirm(uint16_t conn_h, bool accept)
{
    uint8_t key_type = accept ? 1 : 0;
    uint32_t rc = svc_gap_auth_key_reply(conn_h, key_type, NULL);
    return (rc == NRF_SUCCESS) ? 0 : -1;
}

int ble_pair_wait_event(hal_ble_evt_t *evt, uint32_t timeout_ms)
{
    if (!pair_queue) return -1;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    for (;;) {
        ble_serial_poll();
        if (xQueueReceive(pair_queue, evt, pdMS_TO_TICKS(50)) == pdTRUE)
            return 0;
        if (xTaskGetTickCount() >= deadline)
            return -1;
    }
}

int ble_pair_disconnect(uint16_t conn_h)
{
    uint32_t rc = svc_gap_disconnect(conn_h, 0x13); /* REMOTE_USER_TERMINATED */
    return (rc == NRF_SUCCESS) ? 0 : -1;
}
