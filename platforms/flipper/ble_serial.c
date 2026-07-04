#include "ble_serial.h"
#include "ble.h"
#include "display.h"
#include "../../core/log.h"
#include "../../hal/hal.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "semphr.h"
#include <string.h>

/* HCI helpers defined in ble.c */
extern int ble_hci_send(uint16_t opcode, const void *params, uint8_t plen);
extern int ble_hci_send_resp(uint16_t opcode, const void *params, uint8_t plen,
                             uint8_t *resp, uint8_t resp_max, uint8_t *resp_len);

/* ---- UUIDs (match original Flipper serial service) ---- */

static const uint8_t svc_uuid[16] = {
    0x00, 0x00, 0xfe, 0x60, 0xcc, 0x7a, 0x48, 0x2a,
    0x98, 0x4a, 0x7f, 0x2e, 0xd5, 0xb3, 0xe5, 0x8f
};

static const uint8_t rx_uuid[16] = {
    0x00, 0x00, 0xfe, 0x62, 0x8e, 0x22, 0x45, 0x41,
    0x9d, 0x4c, 0x21, 0xed, 0xae, 0x82, 0xed, 0x19
};

static const uint8_t tx_uuid[16] = {
    0x00, 0x00, 0xfe, 0x61, 0x8e, 0x22, 0x45, 0x41,
    0x9d, 0x4c, 0x21, 0xed, 0xae, 0x82, 0xed, 0x19
};

/* ---- Handles ---- */

static uint16_t svc_handle;
static uint16_t rx_char_handle;
static uint16_t tx_char_handle;
static bool     service_ready;

/* ---- Buffers ---- */

/* Must hold the host's pipelined upload window (UPLOAD_WINDOW=6 framed
 * ~482 B chunks ≈ 3 KB) between ISR delivery and the proto task draining it;
 * a smaller buffer overflows when full and drops bytes, corrupting uploads. */
#define RX_BUF_SIZE 4096

static StreamBufferHandle_t rx_stream;
static SemaphoreHandle_t    tx_sem;   /* "WS notification pool has space" signal */

static volatile bool        pair_pending;
static volatile uint16_t    pair_conn;
static volatile uint32_t    pair_passkey;

static volatile bool        conn_param_pending;
static volatile uint16_t    conn_param_handle;
/* Handle of the most-recent connection - what notifications must target. Using
 * the latest (not ble_get_connections, which returns the first active slot) is
 * essential: an aborted attempt + reconnect can leave a stale slot, and
 * notifying the stale handle fails with 0x60 (invalid handle). */
static volatile uint16_t    s_conn_handle = 0xFFFF;

/* ---- ACI opcodes ---- */

#define ACI_GATT_ADD_SERVICE        0xFD02
#define ACI_GATT_ADD_CHAR           0xFD04
#define ACI_GATT_UPDATE_CHAR_VALUE  0xFD06

/* ---- GATT properties / permissions ---- */

#define CHAR_PROP_READ                0x02
#define CHAR_PROP_WRITE_WITHOUT_RESP  0x04
#define CHAR_PROP_WRITE               0x08
#define CHAR_PROP_INDICATE            0x20

#define ATTR_PERM_AUTHEN_READ   0x04
#define ATTR_PERM_AUTHEN_WRITE  0x20

#define GATT_NOTIFY_ATTR_WRITE  0x01

/* ---- Service registration ---- */

int ble_serial_init(void)
{
    rx_stream = xStreamBufferCreate(RX_BUF_SIZE, 1);
    tx_sem    = xSemaphoreCreateBinary();
    if (!rx_stream || !tx_sem) return -1;
    /* tx_sem starts empty: it signals TX-pool-available (given by the
     * 0x0C16 event via ble_serial_on_tx_complete). ble_serial_write only
     * waits on it when the pool is momentarily exhausted mid-stream. */

    /* Add service: 128-bit UUID, primary, 10 attribute slots
     * (1 svc + 2 chars × (decl + value + cccd) + margin) */
    uint8_t svc_cmd[19];
    svc_cmd[0] = 0x02;
    memcpy(&svc_cmd[1], svc_uuid, 16);
    svc_cmd[17] = 0x01;
    svc_cmd[18] = 10;

    uint8_t resp[2];
    uint8_t rlen;
    int rc = ble_hci_send_resp(ACI_GATT_ADD_SERVICE, svc_cmd, 19, resp, 2, &rlen);
    if (rc != 0) return -2;
    svc_handle = (uint16_t)resp[0] | ((uint16_t)resp[1] << 8);

    /* ACI_GATT_ADD_CHAR parameter layout (128-bit UUID):
     * [0:1]   Service_Handle
     * [2]     Char_UUID_Type (0x02 = 128-bit)
     * [3:18]  Char_UUID (16 bytes)
     * [19:20] Char_Value_Length
     * [21]    Char_Properties
     * [22]    Security_Permissions
     * [23]    GATT_Evt_Mask
     * [24]    Enc_Key_Size
     * [25]    Is_Variable
     */

    /* RX characteristic (client writes to device) */
    uint8_t rx_cmd[26];
    rx_cmd[0] = (uint8_t)(svc_handle);
    rx_cmd[1] = (uint8_t)(svc_handle >> 8);
    rx_cmd[2] = 0x02;
    memcpy(&rx_cmd[3], rx_uuid, 16);
    rx_cmd[19] = 0xE6; rx_cmd[20] = 0x01; /* 486 = 0x01E6, matching original */
    rx_cmd[21] = CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP | CHAR_PROP_READ;
    rx_cmd[22] = 0x09; /* AUTHEN_READ | AUTHEN_WRITE, matching original */
    rx_cmd[23] = GATT_NOTIFY_ATTR_WRITE;
    rx_cmd[24] = 0x0A; /* Enc_Key_Size = 10, matching original */
    rx_cmd[25] = 0x01;

    rc = ble_hci_send_resp(ACI_GATT_ADD_CHAR, rx_cmd, 26, resp, 2, &rlen);
    if (rc != 0) return -3;
    rx_char_handle = (uint16_t)resp[0] | ((uint16_t)resp[1] << 8);

    /* TX characteristic (device sends to client via indications) */
    uint8_t tx_cmd[26];
    tx_cmd[0] = (uint8_t)(svc_handle);
    tx_cmd[1] = (uint8_t)(svc_handle >> 8);
    tx_cmd[2] = 0x02;
    memcpy(&tx_cmd[3], tx_uuid, 16);
    tx_cmd[19] = 244; tx_cmd[20] = 0;
    tx_cmd[21] = 0x10; /* NOTIFY */
    tx_cmd[22] = 0x00;
    tx_cmd[23] = 0x00;
    tx_cmd[24] = 0x10;
    tx_cmd[25] = 0x01;

    rc = ble_hci_send_resp(ACI_GATT_ADD_CHAR, tx_cmd, 26, resp, 2, &rlen);
    if (rc != 0) return -4;
    tx_char_handle = (uint16_t)resp[0] | ((uint16_t)resp[1] << 8);

    service_ready = true;
    return 0;
}

/* ---- Event callbacks (called from ble.c ISR context) ---- */

void ble_serial_on_attr_modified(uint16_t conn_handle, uint16_t handle,
                                 const uint8_t *data, uint16_t len)
{
    /* The connection that delivers RX is the one to notify back on - the most
     * reliable source (immune to stale/aborted connection-complete events). */
    s_conn_handle = conn_handle;
    if (handle == rx_char_handle + 1) {
        if (len > 0) {
            BaseType_t woken = pdFALSE;
            xStreamBufferSendFromISR(rx_stream, data, len, &woken);
            portYIELD_FROM_ISR(woken);
        }
    }
}

/* Called from ISR context when the WS frees notification buffers
 * (ACI_GATT_TX_POOL_AVAILABLE, 0x0C16). Unblocks ble_serial_write so the
 * download stream resumes the instant the pool has room. */
void ble_serial_on_tx_complete(void)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(tx_sem, &woken);
    portYIELD_FROM_ISR(woken);
}

/* Called from ISR context on a new peripheral connection. Requesting the
 * faster connection interval must happen in task context (hci_send mutex),
 * so just flag it for ble_serial_poll(). */
void ble_serial_on_connect(uint16_t conn_handle)
{
    conn_param_handle  = conn_handle;
    conn_param_pending = true;
    s_conn_handle      = conn_handle;   /* newest connection = notify target */
}

/* Notification receive ring buffer (central role) */
#define NOTIF_BUF_SIZE 512
static uint8_t notif_buf[NOTIF_BUF_SIZE];
static volatile uint16_t notif_head, notif_tail;

void ble_serial_on_notification(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        notif_buf[notif_head % NOTIF_BUF_SIZE] = data[i];
        notif_head++;
    }
}

uint16_t ble_serial_get_notification(uint8_t *buf, uint16_t max)
{
    uint16_t avail = notif_head - notif_tail;
    if (avail == 0) return 0;
    if (avail > max) avail = max;
    for (uint16_t i = 0; i < avail; i++) {
        buf[i] = notif_buf[notif_tail % NOTIF_BUF_SIZE];
        notif_tail++;
    }
    return avail;
}

void ble_serial_set_pair_pending(uint16_t conn_handle, uint32_t passkey)
{
    pair_conn    = conn_handle;
    pair_passkey = passkey;
    pair_pending = true;
}

void ble_serial_on_disconnect(uint16_t conn_handle)
{
    /* Only clear if it's the active connection disconnecting - a late
     * disconnect for an earlier aborted attempt must not wipe a live handle. */
    if (conn_handle == s_conn_handle) s_conn_handle = 0xFFFF;
}

/* ---- Transport functions (called from BLE CLI task) ---- */

size_t ble_serial_write(const uint8_t *buf, size_t len, void *ctx)
{
    (void)ctx;
    if (!service_ready || len == 0) return 0;

    size_t chunk = len > 244 ? 244 : len;

    /* ACI_GATT_UPDATE_CHAR_VALUE_EXT (0xFD2C):
     * Conn_Handle_To_Notify(2) Service_Handle(2) Char_Handle(2)
     * Update_Type(1) Char_Length(2) Value_Offset(2) Value_Length(1) Value(n) */
    /* Target the most-recent connection (not ble_get_connections, which can
     * return a stale slot after an aborted attempt → 0x60 invalid handle). */
    uint16_t ch = s_conn_handle;
    if (ch == 0xFFFF) return 0;
    uint8_t cmd[12 + 244];
    cmd[0] = (uint8_t)(ch);
    cmd[1] = (uint8_t)(ch >> 8);
    cmd[2] = (uint8_t)(svc_handle);
    cmd[3] = (uint8_t)(svc_handle >> 8);
    cmd[4] = (uint8_t)(tx_char_handle);
    cmd[5] = (uint8_t)(tx_char_handle >> 8);
    cmd[6] = 0x01;  /* Update_Type = notify */
    cmd[7] = (uint8_t)chunk;
    cmd[8] = 0;     /* Char_Length high */
    cmd[9] = 0;     /* Value_Offset low */
    cmd[10] = 0;    /* Value_Offset high */
    cmd[11] = (uint8_t)chunk;  /* Value_Length */
    memcpy(&cmd[12], buf, chunk);

    /* Send the notification. Only INSUFFICIENT_RESOURCES (0x64) is transient -
     * the WS notification pool is momentarily full mid-stream; wait for the
     * TX-pool-available event (0x0C16 → tx_sem) and retry so the byte stream
     * stays intact. Any other non-zero status (e.g. notifications not enabled
     * because the peer never wrote the CCCD, or the link dropping) is NOT
     * transient and must fail fast: retrying a permanent error forever would
     * block the proto task until the watchdog reset the device. */
    int rc = ble_hci_send(0xFD2C, cmd, (uint8_t)(12 + chunk));
    /* 0x64 = INSUFFICIENT_RESOURCES: the WS notification pool (121 blocks) is
     * momentarily full. It drains at the connection rate, so a burst can take
     * up to ~1.5 s to clear. Retry until a slot frees rather than dropping the
     * byte - a dropped notification corrupts the stream (CRC mismatch, or a
     * throughput collapse if it recurs). The retry is bounded by the connection
     * staying up: the supervision timeout clears s_conn_handle if the link
     * dies, so it cannot spin forever. The cap must be large enough that a
     * full-pool burst is never abandoned mid-stream. */
    for (int i = 0; rc == 0x64 && i < 500; i++) {
        if (s_conn_handle == 0xFFFF) return 0;
        xSemaphoreTake(tx_sem, pdMS_TO_TICKS(10));
        rc = ble_hci_send(0xFD2C, cmd, (uint8_t)(12 + chunk));
    }
    return rc == 0 ? chunk : 0;
}

size_t ble_serial_read(uint8_t *buf, size_t len, void *ctx)
{
    (void)ctx;
    return xStreamBufferReceive(rx_stream, buf, len, 0);
}

void ble_serial_poll(void)
{
    if (conn_param_pending) {
        conn_param_pending = false;
        uint16_t ch = conn_param_handle;
        /* ACI_L2CAP_CONNECTION_PARAMETER_UPDATE_REQ (0xFD81):
         * conn_handle(2) interval_min(2) interval_max(2)
         * slave_latency(2) timeout_multiplier(2)
         * Interval 36 (45 ms). A page erase (~20 ms) can only run while the BLE
         * RF is idle for >=25 ms (SHCI_C2_FLASH_EraseActivity, ST flash_driver.c).
         * That idle window must come from the connection interval itself: during
         * a transfer the device sends an ack/notification every event, so it is
         * never idle and slave latency never engages - only a long-enough
         * interval guarantees the gap. 45 ms leaves a ~40 ms gap between events,
         * so a page erase fits cleanly within one interval. The host retransmit
         * (idempotent offset writes) covers any residual timing jitter. Matches
         * the reference Flipper serial profile's 45 ms max interval, latency 0.
         * Supervision timeout 4 s (400 × 10 ms). */
        uint8_t cp[10];
        cp[0] = (uint8_t)ch;   cp[1] = (uint8_t)(ch >> 8);
        cp[2] = 36; cp[3] = 0;        /* interval_min = 36 (45 ms) */
        cp[4] = 36; cp[5] = 0;        /* interval_max = 36 (45 ms) */
        cp[6] = 0;  cp[7] = 0;        /* slave latency = 0 */
        cp[8] = 0x90; cp[9] = 0x01;   /* timeout_multiplier = 400 (4 s) */
        ble_hci_send(0xFD81, cp, sizeof(cp));

        /* HCI LE Set Data Length (0x2022): request 251-byte LL payload so a
         * full 244-byte ATT notification fits in ONE link-layer packet.
         * Without this the connection may negotiate a smaller data length and
         * the WS rejects a 244-byte notification with 0x60 (invalid handle) -
         * the max single-packet value is DLE_payload - 4 (L2CAP) - 3 (ATT). */
        uint8_t dl[6];
        dl[0] = (uint8_t)ch;   dl[1] = (uint8_t)(ch >> 8);
        dl[2] = 0xFB; dl[3] = 0x00;   /* TxOctets = 251 */
        dl[4] = 0x48; dl[5] = 0x08;   /* TxTime   = 2120 µs (251 B @ 1M) */
        ble_hci_send(0x2022, dl, sizeof(dl));
    }

    if (!pair_pending) return;
    pair_pending = false;

    uint16_t ch = pair_conn;
    uint32_t pk = pair_passkey;

    if (pk != 0) {
        /* Numeric comparison request. The peripheral IO capability is
         * DISPLAY_ONLY (see ble.c), so the stack negotiates Passkey Entry and
         * this event must not occur. If it ever does (e.g. a peer/stack that
         * forces Numeric Comparison), REJECT it - never auto-confirm. Blindly
         * confirming would bond an unverified peer (no human compared the
         * codes) and expose the CLI/file service: a remote-code-execution risk.
         * Reply "no" (0x00) to ACI_GAP_NUMERIC_COMPARISON_VALUE_CONFIRM_YESNO. */
        uint8_t conf[] = { (uint8_t)ch, (uint8_t)(ch >> 8), 0x00 };
        ble_hci_send(0xFCA5, conf, 3);
    } else {
        /* Passkey request - generate, display, and respond.
         * With DISPLAY_ONLY IO cap, the host enters this passkey.
         * Output to display + USB serial BEFORE hci_send so the
         * host agent can read it while the HCI command blocks. */
        uint32_t pin = ble_generate_passkey() % 1000000;
        char pkstr[8];
        uint32_t tmp = pin;
        for (int i = 5; i >= 0; i--) {
            pkstr[i] = '0' + (tmp % 10); tmp /= 10;
        }
        pkstr[6] = '\0';
        display_clear();
        display_print(25, 2, "BLE pair");
        display_print(37, 4, pkstr);
        display_flush();
        fantasi_log(LOG_INFO, "BLE pair code: %s", pkstr);
        uint8_t resp_cmd[6];
        resp_cmd[0] = (uint8_t)ch; resp_cmd[1] = (uint8_t)(ch >> 8);
        memcpy(&resp_cmd[2], &pin, 4);
        ble_hci_send(0xFC88, resp_cmd, 6);
    }
}

bool ble_serial_connected(void *ctx)
{
    (void)ctx;
    if (!service_ready || !ble_is_active()) return false;
    ble_conn_info_t conn;
    return ble_get_connections(&conn, 1) > 0;
}
