/* Fantasi / Flipper Zero - BLE subsystem.
 *
 * Self-contained BLE scanning via the STM32WB55's M0+ radio coprocessor.
 * Communicates with CPU2 through IPCC mailbox in shared SRAM2A, using
 * the standard ST transport layer protocol (AN5289).
 *
 * On first scan, boots CPU2, waits for SHCI ready, sends SHCI_C2_BLE_Init
 * in LL+Host mode. ACI GAP discovery commands control the radio.
 */

#include "ble.h"
#include "ble_serial.h"
#include "display.h"
#include "power.h"
#include "../../core/cli.h"
#include "../../hal/hal.h"
#include "../../hal/hal_power.h"
#include "stm32wbxx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  Constants                                                         */
/* ------------------------------------------------------------------ */

#define TL_BLECMD_PKT  0x01
#define TL_SYSCMD_PKT  0x10

#define EVT_CMD_COMPLETE  0x0E
#define EVT_CMD_STATUS    0x0F
#define EVT_LE_META       0x3E
#define EVT_VENDOR        0xFF

#define LE_ADV_REPORT     0x02

#define SHCI_EVT_READY    0x9200

#define SHCI_OPCODE_BLE_INIT        0xFC66
#define SHCI_OPCODE_SET_FLASH_ACT   0xFC73
#define SHCI_OPCODE_FLASH_ERASE_ACT 0xFC69
#define SHCI_OPCODE_FUS_GET_STATE   0xFC52

#define HCI_LE_SET_SCAN_PARAMS  0x200B
#define HCI_LE_SET_SCAN_ENABLE  0x200C

#define IPCC_CH_BLE  (1U << 0)
#define IPCC_CH_SYS  (1U << 1)
#define IPCC_CH_MM   (1U << 3)

#define AD_TYPE_SHORT_NAME  0x08
#define AD_TYPE_FULL_NAME   0x09
#define AD_TYPE_FLAGS       0x01

#define LE_CONNECTION_COMPLETE  0x01
#define EVT_DISCONN_COMPLETE    0x05

#define ACI_GAP_SET_IO_CAPABILITY       0xFC85
#define ACI_GAP_SET_AUTH_REQUIREMENT    0xFC86
#define ACI_GAP_PASS_KEY_RESP           0xFC88
#define ACI_GAP_SET_DISCOVERABLE        0xFC83
#define ACI_GAP_SET_NON_DISCOVERABLE    0xFC81
#define ACI_GAP_CREATE_CONNECTION       0xFC9C
#define ACI_GAP_SEND_PAIRING_REQ        0xFC9F
#define ACI_GAP_TERMINATE               0xFC93
#define ACI_GAP_TERMINATE_GAP_PROC      0xFC9D
#define ACI_GATT_UPDATE_CHAR_VALUE      0xFD06

/* Legacy advertising has 31 bytes total. The stack also contributes Flags and
 * TX Power, leaving 23 bytes for the complete local name. Keep GAP's Device
 * Name characteristic identical to the advertised identity. */
#define BLE_DEVICE_NAME_MAX             23

#define GAP_EVT_PAIRING_COMPLETE        0x0401
#define GAP_EVT_PASS_KEY_REQ            0x0402
#define GAP_EVT_PROC_COMPLETE           0x0407
#define GAP_EVT_NUMERIC_COMPARISON      0x0409

#define SCAN_QUEUE_LEN  16
#define PAIR_QUEUE_LEN   8

/* ------------------------------------------------------------------ */
/*  Linked list (matching ST stm_list - circular doubly-linked)       */
/* ------------------------------------------------------------------ */

typedef struct ln { struct ln *next, *prev; } list_node_t;

static void lst_init(list_node_t *h)
{
    h->next = h;
    h->prev = h;
}

static bool lst_empty(volatile list_node_t *h)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    bool e = (h->next == (list_node_t *)h);
    __set_PRIMASK(pm);
    return e;
}

static void lst_insert_tail(volatile list_node_t *h, list_node_t *n)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    n->next = (list_node_t *)h;
    n->prev = h->prev;
    h->prev = n;
    n->prev->next = n;
    __set_PRIMASK(pm);
}

static void lst_remove_head(volatile list_node_t *h, list_node_t **n)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    *n = h->next;
    (*n)->next->prev = (list_node_t *)h;
    h->next = (*n)->next;
    (*n)->next = *n;
    (*n)->prev = *n;
    __set_PRIMASK(pm);
}

/* ------------------------------------------------------------------ */
/*  Packet structures (packed, matching ST TL binary format)          */
/* ------------------------------------------------------------------ */

typedef struct {
    list_node_t hdr;
    uint8_t  type;
    uint16_t cmdcode;
    uint8_t  plen;
    uint8_t  payload[255];
} __attribute__((packed)) cmd_pkt_t;

typedef struct {
    list_node_t hdr;
    uint8_t  type;
    uint8_t  evtcode;
    uint8_t  plen;
    uint8_t  payload[255];
} __attribute__((packed)) evt_pkt_t;

/* ------------------------------------------------------------------ */
/*  Mailbox table structures                                          */
/* ------------------------------------------------------------------ */

typedef struct {
    uint32_t Version;
} mb_safe_boot_t;

typedef struct {
    uint32_t Version;
    uint32_t MemorySize;
    uint32_t FusInfo;
} mb_fus_info_t;

typedef struct {
    uint32_t Version;
    uint32_t MemorySize;
    uint32_t InfoStack;
    uint32_t Reserved;
} mb_ws_info_t;

typedef struct {
    mb_safe_boot_t SafeBoot;
    mb_fus_info_t  Fus;
    mb_ws_info_t   Ws;
} mb_device_info_t;

typedef struct {
    uint8_t *pcmd_buffer;
    uint8_t *pcs_buffer;
    uint8_t *pevt_queue;
    uint8_t *phci_acl_data_buffer;
} mb_ble_table_t;

typedef struct {
    uint8_t *pcmd_buffer;
    uint8_t *sys_queue;
} mb_sys_table_t;

typedef struct {
    uint8_t  *spare_ble_buffer;
    uint8_t  *spare_sys_buffer;
    uint8_t  *blepool;
    uint32_t  blepoolsize;
    uint8_t  *pevt_free_buffer_queue;
    uint8_t  *traces_evt_pool;
    uint32_t  tracespoolsize;
} mb_mm_table_t;

typedef struct {
    uint8_t *traces_queue;
} mb_traces_table_t;

typedef struct {
    mb_device_info_t  *p_device_info_table;
    mb_ble_table_t    *p_ble_table;
    void              *p_thread_table;
    mb_sys_table_t    *p_sys_table;
    mb_mm_table_t     *p_mem_manager_table;
    mb_traces_table_t *p_traces_table;
    void              *p_mac_802_15_4_table;
    void              *p_zigbee_table;
    void              *p_lld_tests_table;
    void              *p_ble_lld_table;
} mb_ref_table_t;

/* ------------------------------------------------------------------ */
/*  BLE init parameter structure (must be 46 bytes packed)            */
/* ------------------------------------------------------------------ */

typedef struct {
    uint32_t pBleBufferAddress;
    uint32_t BleBufferSize;
    uint16_t NumAttrRecord;
    uint16_t NumAttrServ;
    uint16_t AttrValueArrSize;
    uint8_t  NumOfLinks;
    uint8_t  ExtendedPacketLengthEnable;
    uint8_t  PrWriteListSize;
    uint8_t  MblockCount;
    uint16_t AttMtu;
    uint16_t PeripheralSca;
    uint8_t  CentralSca;
    uint8_t  LsSource;
    uint32_t MaxConnEventLength;
    uint16_t HsStartupTime;
    uint8_t  ViterbiEnable;
    uint8_t  Options;
    uint8_t  HwVersion;
    uint8_t  max_coc_initiator_nbr;
    int8_t   min_tx_power;
    int8_t   max_tx_power;
    uint8_t  rx_model_config;
    uint8_t  max_adv_set_nbr;
    uint16_t max_adv_data_len;
    int16_t  tx_path_compens;
    int16_t  rx_path_compens;
    uint8_t  ble_core_version;
    uint8_t  Options_extension;
} __attribute__((packed)) ble_init_params_t;

_Static_assert(sizeof(ble_init_params_t) == 46, "BLE init params size mismatch");

/* ------------------------------------------------------------------ */
/*  Shared memory in SRAM2A                                           */
/* ------------------------------------------------------------------ */

/* Reference table at SRAM2A_BASE (0x20030000) - CPU2 reads this. */
__attribute__((section(".mapping_table"), used))
static volatile mb_ref_table_t ref_table;

#define SEC_MB1  __attribute__((section(".mb_mem1"), aligned(4)))

SEC_MB1 static mb_device_info_t  device_info_table;
SEC_MB1 static mb_ble_table_t    ble_table;
SEC_MB1 static mb_sys_table_t    sys_table;
SEC_MB1 static mb_mm_table_t     mm_table;
SEC_MB1 static mb_traces_table_t traces_table;

SEC_MB1 static volatile list_node_t evt_queue;
SEC_MB1 static volatile list_node_t sys_evt_queue;
SEC_MB1 static volatile list_node_t free_buf_queue;
SEC_MB1 static volatile list_node_t traces_evt_queue;

/* MB_MEM2 buffers in SRAM1.  SRAM2A is secured from 0x20030400
 * onward (SBRSA=1 with the extended wireless stack), so only the
 * first 1 KB is non-secure.  The reference table and MB_MEM1 tables
 * fit there, but the ~2.4 KB of command/event buffers do not.  CPU2
 * can access SRAM1 because we enable RCC_C2AHB1ENR_SRAM1EN. */
static cmd_pkt_t sys_cmd_buf  __attribute__((aligned(4)));
static cmd_pkt_t ble_cmd_buf  __attribute__((aligned(4)));
static uint8_t   cs_buffer[sizeof(list_node_t) + 3 + 4] __attribute__((aligned(4)));

/* ~10 event buffers. The reference uses 5 (CFG_TLBLE_EVT_QUEUE_LENGTH) and
 * relies on flash↔radio coordination (EraseActivity defers CPU2's radio during
 * an erase, so the host can't transmit into the stall) rather than a giant pool.
 * A small amount of headroom over 5 covers the few writes already in flight when
 * a flash op begins. */
#define EVT_POOL_SIZE  2680
static uint8_t   evt_pool[EVT_POOL_SIZE]                 __attribute__((aligned(4)));
static uint8_t   spare_ble_evt[sizeof(list_node_t) + 3 + 255] __attribute__((aligned(4)));
static uint8_t   spare_sys_evt[sizeof(list_node_t) + 3 + 255] __attribute__((aligned(4)));

/* ------------------------------------------------------------------ */
/*  State + synchronization                                           */
/* ------------------------------------------------------------------ */

static enum { BLE_UNINIT, BLE_READY, BLE_ERROR, BLE_DISABLED } ble_state;

static SemaphoreHandle_t c2_ready_sem;
static SemaphoreHandle_t hci_resp_sem;
static QueueHandle_t     scan_queue;
static QueueHandle_t     pair_queue;

static volatile uint8_t hci_cmd_status;
static volatile uint8_t c2_mode; /* 0 = wireless stack, 1 = FUS */

static uint8_t *hci_resp_buf;
static uint8_t  hci_resp_buf_max;
static volatile uint8_t hci_resp_buf_len;

static ble_conn_info_t ble_conns[BLE_MAX_CONN];
static volatile bool adv_restart_needed;
static volatile bool pair_in_progress;
static int hci_send(uint16_t opcode, const void *params, uint8_t plen);
static int hci_send_locked(uint16_t opcode, const void *params, uint8_t plen);
static TimerHandle_t  adv_timer;
static void ble_start_background_adv(void);

static uint8_t ble_format_device_name(char out[BLE_DEVICE_NAME_MAX + 1])
{
    const char *src = "Fantasi ";
    uint8_t len = 0;
    while (*src && len < BLE_DEVICE_NAME_MAX)
        out[len++] = *src++;

    src = hal_device_name();
    while (src && *src && len < BLE_DEVICE_NAME_MAX)
        out[len++] = *src++;
    out[len] = '\0';
    return len;
}

static void adv_timer_cb(TimerHandle_t t)
{
    ble_start_background_adv();
    /* Connection teardown may outlive the disconnect event. Retry until CPU2
     * accepts SET_DISCOVERABLE. */
    if (adv_restart_needed)
        (void)xTimerStart(t, 0);
}

/* ------------------------------------------------------------------ */
/*  IPCC helpers                                                      */
/* ------------------------------------------------------------------ */

static inline void ipcc_c1_set_flag(uint32_t ch)
{
    IPCC->C1SCR = ch << 16;
}

static inline void ipcc_c1_clear_flag(uint32_t ch)
{
    IPCC->C1SCR = ch;
}

static inline bool ipcc_c1_flag_active(uint32_t ch)
{
    return (IPCC->C1TOC2SR & ch) != 0;
}

static inline bool ipcc_c2_flag_active(uint32_t ch)
{
    return (IPCC->C2TOC1SR & ch) != 0;
}

static inline void ipcc_unmask_rx(uint32_t ch)
{
    IPCC->C1MR &= ~ch;
}

static inline void ipcc_mask_rx(uint32_t ch)
{
    IPCC->C1MR |= ch;
}

static inline void ipcc_unmask_tx(uint32_t ch)   /* TX-free IRQ masks: C1MR[16:21] */
{
    IPCC->C1MR &= ~(ch << 16);
}

static inline void ipcc_mask_tx(uint32_t ch)
{
    IPCC->C1MR |= (ch << 16);
}

/* ------------------------------------------------------------------ */
/*  Return event buffer to CPU2 via memory manager channel            */
/* ------------------------------------------------------------------ */

/* CPU1-private staging queue for buffers waiting to be handed back to CPU2.
 * The shared free_buf_queue is drained by CPU2; CPU1 must not insert into it
 * while CPU2 is draining (MM channel still active) or the unsynchronised list
 * corrupts and event buffers leak - after ~31 events the WS event pool is
 * exhausted and CPU2 silently stops delivering RX writes (the upload stall).
 * Mirror TL_MM_EvtDone/SendFreeBuf: stage locally, flush to the shared queue
 * only when the MM channel is free, and defer the rest to the TX-free IRQ. */
static list_node_t local_free_q;

static void mm_flush_local(void)
{
    while (!lst_empty(&local_free_q)) {
        list_node_t *n;
        lst_remove_head(&local_free_q, &n);
        lst_insert_tail((list_node_t *)&free_buf_queue, n);
    }
    __DSB();
}

static void mm_return_evt(evt_pkt_t *evt)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    lst_insert_tail(&local_free_q, (list_node_t *)evt);
    if (ipcc_c1_flag_active(IPCC_CH_MM)) {
        /* CPU2 is still reclaiming the previous batch - defer; the TX-free IRQ
         * flushes the staged buffers once the channel clears. */
        ipcc_unmask_tx(IPCC_CH_MM);
    } else {
        mm_flush_local();
        ipcc_c1_set_flag(IPCC_CH_MM);
    }
    __set_PRIMASK(pm);
}

/* ------------------------------------------------------------------ */
/*  IPCC interrupt handlers                                           */
/* ------------------------------------------------------------------ */

static void handle_sys_evt(void)
{
    BaseType_t woken = pdFALSE;

    while (!lst_empty(&sys_evt_queue)) {
        list_node_t *node;
        lst_remove_head(&sys_evt_queue, &node);
        evt_pkt_t *evt = (evt_pkt_t *)node;

        if (evt->evtcode == EVT_VENDOR && evt->plen >= 2) {
            uint16_t subevt = (uint16_t)evt->payload[0]
                            | ((uint16_t)evt->payload[1] << 8);
            if (subevt == SHCI_EVT_READY) {
                c2_mode = (evt->plen >= 3) ? evt->payload[2] : 0;
                if (c2_ready_sem)
                    xSemaphoreGiveFromISR(c2_ready_sem, &woken);
            }
        }
        mm_return_evt(evt);
    }
    ipcc_c1_clear_flag(IPCC_CH_SYS);
    portYIELD_FROM_ISR(woken);
}

/* Notification sizing: the WS sends a GATT notification as one LL packet (no
 * fragmentation), so the value must fit in the negotiated data length:
 *   value ≤ MaxTxOctets - 4 (L2CAP) - 3 (ATT) , and also ≤ ATT_MTU - 3.
 * Track both the negotiated ATT MTU and TX data length and feed the smaller to
 * the proto pipe. Defaults (MTU 23, DLE 27 → 20-byte notify) are the safe
 * fallback if neither is negotiated. Mis-sizing → 0x60 (invalid handle). */
static volatile uint16_t s_att_mtu = 23;
static volatile uint16_t s_dle_tx  = 27;

static void update_notify_size(void)
{
    extern void proto_set_mtu(uint16_t);
    uint16_t eff = s_att_mtu;                 /* pipe payload = eff - 3 */
    uint16_t dle_eq = (s_dle_tx > 4) ? (uint16_t)(s_dle_tx - 4) : 23;
    if (dle_eq < eff) eff = dle_eq;
    /* This unit's WS rejects notifications above ~200 B (0x60) regardless of
     * MTU/DLE; cap the payload at 180 so the link is reliable. */
    if (eff > 183) eff = 183;
    proto_set_mtu(eff);
}

static void parse_adv_report(const uint8_t *data, uint8_t len)
{
    if (len < 12 || !scan_queue) return;

    uint8_t num_reports = data[0];
    const uint8_t *p = data + 1;

    for (uint8_t i = 0; i < num_reports && (p - data) < len; i++) {
        ble_scan_result_t r;
        memset(&r, 0, sizeof(r));

        uint8_t evt_type  = *p++;
        (void)evt_type;
        r.addr_type = *p++;
        memcpy(r.addr, p, 6); p += 6;
        uint8_t data_len = *p++;

        if ((p - data) + data_len + 1 > len) break;

        const uint8_t *ad = p;
        const uint8_t *ad_end = p + data_len;
        while (ad < ad_end) {
            uint8_t ad_len = ad[0];
            if (ad_len == 0 || ad + 1 + ad_len > ad_end) break;
            uint8_t ad_type = ad[1];
            if (ad_type == AD_TYPE_FULL_NAME || ad_type == AD_TYPE_SHORT_NAME) {
                uint8_t nlen = ad_len - 1;
                if (nlen > sizeof(r.name) - 1)
                    nlen = sizeof(r.name) - 1;
                memcpy(r.name, ad + 2, nlen);
                r.name[nlen] = '\0';
            }
            ad += 1 + ad_len;
        }
        p += data_len;

        r.rssi = (int8_t)*p++;

        BaseType_t woken = pdFALSE;
        xQueueSendFromISR(scan_queue, &r, &woken);
    }
}

static void post_pair_event(ble_event_t *e, BaseType_t *woken)
{
    if (pair_queue)
        xQueueSendFromISR(pair_queue, e, woken);
}

static void handle_connection_complete(const uint8_t *p, uint8_t len,
                                       BaseType_t *woken)
{
    if (len < 11) return;
    uint8_t  status  = p[0];
    uint16_t handle  = (uint16_t)p[1] | ((uint16_t)p[2] << 8);

    /* New link: reset negotiated sizing to the safe defaults until the MTU
     * exchange and data-length update re-negotiate them. */
    s_att_mtu = 23;
    s_dle_tx  = 27;
    if (status == 0) {
        /* A successful CONNECT_IND ends advertising. Cancel any retry left by
         * an earlier aborted attempt; a stale timer callback also checks the
         * active connection table before touching GAP state. */
        adv_restart_needed = false;
        pwr_inhibit_enter(PWR_CLIENT_BLE_LINK);   /* atomic - ISR-safe */
        for (int i = 0; i < BLE_MAX_CONN; i++) {
            if (!ble_conns[i].active) {
                ble_conns[i].handle    = handle;
                ble_conns[i].addr_type = p[4];
                memcpy(ble_conns[i].addr, &p[5], 6);
                ble_conns[i].active    = true;
                break;
            }
        }
        /* Request the flash-safe connection interval in task context
         * (hci_send cannot run from this ISR). */
        ble_serial_on_connect(handle);
    } else {
        /* Connection attempt failed/aborted (status != 0) - e.g. the central
         * aborted pairing (le-connection-abort-by-local). The peripheral
         * already stopped advertising the instant it received CONNECT_IND, but
         * no usable link exists and no HCI_DISCONNECTION_COMPLETE will follow,
         * so handle_disconnection() never runs. Without restarting advertising
         * here, the device would stay silent (un-discoverable) until reboot. */
        adv_restart_needed = true;
        if (adv_timer)
            xTimerStartFromISR(adv_timer, woken);
    }

    ble_event_t e = {0};
    e.type        = BLE_EVT_CONNECTED;
    e.status      = status;
    e.conn_handle = handle;
    post_pair_event(&e, woken);
}

static void handle_disconnection(const uint8_t *p, uint8_t len,
                                 BaseType_t *woken)
{
    if (len < 4) return;
    uint16_t handle = (uint16_t)p[1] | ((uint16_t)p[2] << 8);

    ble_serial_on_disconnect(handle);

    for (int i = 0; i < BLE_MAX_CONN; i++) {
        if (ble_conns[i].active && ble_conns[i].handle == handle) {
            ble_conns[i].active = false;
            pwr_inhibit_exit(PWR_CLIENT_BLE_LINK);   /* atomic - ISR-safe */
            break;
        }
    }

    bool any_active = false;
    for (int i = 0; i < BLE_MAX_CONN; i++)
        if (ble_conns[i].active) { any_active = true; break; }
    if (!any_active) {
        adv_restart_needed = true;
        if (adv_timer)
            xTimerStartFromISR(adv_timer, woken);
    }

    ble_event_t e = {0};
    e.type        = BLE_EVT_DISCONNECTED;
    e.status      = p[0];
    e.conn_handle = handle;
    e.reason      = p[3];
    post_pair_event(&e, woken);
}

static void handle_gap_vendor_evt(uint16_t subevt, const uint8_t *p,
                                  uint8_t len, BaseType_t *woken)
{
    ble_event_t e = {0};

    switch (subevt) {
    case GAP_EVT_PASS_KEY_REQ: {
        if (len < 2) return;
        uint16_t ch = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
        if (!pair_in_progress) {
            /* No CLI pair command active - auto-handle for BLE serial.
             * Generate passkey, show on display, respond to stack. */
            ble_serial_set_pair_pending(ch, 0);
        } else {
            e.type        = BLE_EVT_PASSKEY_REQUEST;
            e.conn_handle = ch;
            post_pair_event(&e, woken);
        }
        break;
    }

    case GAP_EVT_NUMERIC_COMPARISON: {
        if (len < 6) return;
        uint16_t ch2 = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
        uint32_t pk2 = (uint32_t)p[2] | ((uint32_t)p[3] << 8)
                     | ((uint32_t)p[4] << 16) | ((uint32_t)p[5] << 24);
        if (!pair_in_progress) {
            /* Defer to task context - ISR can't call hci_send (mutex).
             * ble_serial_poll() in the BLE CLI task will respond. */
            ble_serial_set_pair_pending(ch2, pk2);
        } else {
            e.type        = BLE_EVT_PASSKEY_DISPLAY;
            e.conn_handle = ch2;
            e.passkey     = pk2;
            post_pair_event(&e, woken);
        }
        break;
    }

    case GAP_EVT_PAIRING_COMPLETE:
        if (len < 4) return;
        pair_in_progress = false;
        e.type        = BLE_EVT_PAIR_COMPLETE;
        e.conn_handle = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
        e.status      = p[2];
        e.reason      = p[3];
        post_pair_event(&e, woken);
        break;

    default:
        break;
    }
}

static void handle_ble_evt(void)
{
    BaseType_t woken = pdFALSE;

    while (!lst_empty(&evt_queue)) {
        list_node_t *node;
        lst_remove_head(&evt_queue, &node);
        __DSB();
        evt_pkt_t *evt = (evt_pkt_t *)node;

        if (evt->evtcode == EVT_CMD_COMPLETE || evt->evtcode == EVT_CMD_STATUS) {
            if (evt->evtcode == EVT_CMD_COMPLETE && evt->plen >= 4) {
                hci_cmd_status = evt->payload[3];
                if (hci_resp_buf && evt->plen > 4) {
                    uint8_t n = evt->plen - 4;
                    if (n > hci_resp_buf_max) n = hci_resp_buf_max;
                    memcpy(hci_resp_buf, &evt->payload[4], n);
                    hci_resp_buf_len = n;
                }
            } else if (evt->evtcode == EVT_CMD_STATUS && evt->plen >= 1)
                hci_cmd_status = evt->payload[0];
            else
                hci_cmd_status = 0xFF;

            if (hci_resp_sem)
                xSemaphoreGiveFromISR(hci_resp_sem, &woken);

        } else if (evt->evtcode == EVT_DISCONN_COMPLETE) {
            handle_disconnection(evt->payload, evt->plen, &woken);

        } else if (evt->evtcode == EVT_LE_META && evt->plen >= 2) {
            uint8_t subevent = evt->payload[0];
            if (subevent == LE_ADV_REPORT)
                parse_adv_report(evt->payload + 1, evt->plen - 1);
            else if (subevent == LE_CONNECTION_COMPLETE)
                handle_connection_complete(evt->payload + 1, evt->plen - 1, &woken);
            else if (subevent == 0x07 && evt->plen >= 5) {
                /* LE Data Length Change: [1:2] conn, [3:4] MaxTxOctets.
                 * Size notifications to fit the negotiated TX data length. */
                s_dle_tx = (uint16_t)evt->payload[3]
                         | ((uint16_t)evt->payload[4] << 8);
                update_notify_size();
            }

        } else if (evt->evtcode == EVT_VENDOR && evt->plen >= 2) {
            uint16_t subevt = (uint16_t)evt->payload[0]
                            | ((uint16_t)evt->payload[1] << 8);
            if (subevt == 0x0C13 && evt->plen >= 7) {
                /* ACI_GATT_WRITE_PERMIT_REQ_EVENT:
                 * [2:3] conn_handle, [4:5] attr_handle,
                 * [6] data_length (1 byte), [7:] data */
                uint16_t wconn = (uint16_t)evt->payload[2]
                               | ((uint16_t)evt->payload[3] << 8);
                uint16_t wh = (uint16_t)evt->payload[4]
                            | ((uint16_t)evt->payload[5] << 8);
                uint8_t wlen = evt->payload[6];
                ble_serial_on_attr_modified(wconn, wh, &evt->payload[7], wlen);
            } else if (subevt == 0x0C0F) {
                if (evt->plen >= 7) {
                    uint8_t nlen = evt->payload[6];
                    ble_serial_on_notification(&evt->payload[7], nlen);
                }
            } else if (subevt == 0x0C01 && evt->plen >= 10) {
                /* ACI_GATT_ATTRIBUTE_MODIFIED:
                 * [2:3] conn_handle, [4:5] attr_handle,
                 * [6:7] offset, [8:9] data_length, [10:] data */
                uint16_t aconn  = (uint16_t)evt->payload[2]
                                | ((uint16_t)evt->payload[3] << 8);
                uint16_t attr_h = (uint16_t)evt->payload[4]
                                | ((uint16_t)evt->payload[5] << 8);
                uint16_t dlen   = (uint16_t)evt->payload[8]
                                | ((uint16_t)evt->payload[9] << 8);
                ble_serial_on_attr_modified(aconn, attr_h, &evt->payload[10], dlen);
            } else if (subevt == 0x0C02) {
                /* ACI_GATT_SERVER_CONFIRMATION */
                ble_serial_on_tx_complete();
            } else if (subevt == 0x0C16) {
                /* ACI_GATT_TX_POOL_AVAILABLE: notification buffers freed -
                 * resume a download stream that was waiting on the pool. */
                ble_serial_on_tx_complete();
            } else if (subevt == 0x0C03 && evt->plen >= 6) {
                /* ACI_ATT_EXCHANGE_MTU_RESP: [2:3] conn, [4:5] server_rx_mtu.
                 * Feed the negotiated MTU into the notification sizing (also
                 * bounded by the data length - see update_notify_size). */
                s_att_mtu = (uint16_t)evt->payload[4]
                          | ((uint16_t)evt->payload[5] << 8);
                update_notify_size();
            } else {
                handle_gap_vendor_evt(subevt, evt->payload + 2,
                                      evt->plen - 2, &woken);
            }
        }

        mm_return_evt(evt);
    }
    ipcc_c1_clear_flag(IPCC_CH_BLE);
    /* Wake the BLE proto task: it blocks in ble_serial_wait between events. */
    ble_serial_wake_from_isr(&woken);
    portYIELD_FROM_ISR(woken);
}

void IPCC_C1_RX_IRQHandler(void)
{
    if (ipcc_c2_flag_active(IPCC_CH_SYS) && !(IPCC->C1MR & IPCC_CH_SYS))
        handle_sys_evt();
    if (ipcc_c2_flag_active(IPCC_CH_BLE) && !(IPCC->C1MR & IPCC_CH_BLE))
        handle_ble_evt();
}

void IPCC_C1_TX_IRQHandler(void)
{
    /* MM release-buffer channel went free (CPU2 reclaimed the staged batch):
     * hand it any buffers that arrived while it was busy, then re-arm. */
    if (!ipcc_c1_flag_active(IPCC_CH_MM)) {
        uint32_t pm = __get_PRIMASK();
        __disable_irq();
        ipcc_mask_tx(IPCC_CH_MM);
        if (!lst_empty(&local_free_q)) {
            mm_flush_local();
            ipcc_c1_set_flag(IPCC_CH_MM);
        }
        __set_PRIMASK(pm);
    } else {
        IPCC->C1MR |= 0x003F0000;   /* mask any other (unused) TX sources */
    }
}

void HSEM_IRQHandler(void)
{
    /* CPU2 releases semaphores during boot - just ignore. */
}

/* ------------------------------------------------------------------ */
/*  System channel (SHCI) - synchronous command/response              */
/* ------------------------------------------------------------------ */

static int shci_send(uint16_t opcode, const void *params, uint8_t plen)
{
    /* Vote out of deep sleep for the whole CPU2 round-trip: sleeping between
     * command and response risks a missed/misordered SHCI handshake. Phase A
     * never deep-sleeps, but the vote makes Phase B safe. */
    pwr_inhibit_enter(PWR_CLIENT_SD_OP);

    sys_cmd_buf.type    = TL_SYSCMD_PKT;
    sys_cmd_buf.cmdcode = opcode;
    sys_cmd_buf.plen    = plen;
    if (plen && params)
        memcpy(sys_cmd_buf.payload, params, plen);

    __DSB();
    ipcc_c1_set_flag(IPCC_CH_SYS);

    /* Wait up to 33 seconds for CPU2 to process the command. */
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(33000);
    while (ipcc_c1_flag_active(IPCC_CH_SYS)) {
        if (xTaskGetTickCount() >= deadline) {
            pwr_inhibit_exit(PWR_CLIENT_SD_OP);
            return -2;
        }
        taskYIELD();
    }
    pwr_inhibit_exit(PWR_CLIENT_SD_OP);

    uint8_t *rsp = (uint8_t *)&sys_cmd_buf + sizeof(list_node_t);
    /* rsp: [type][evtcode=0x0E][plen][numcmd][cmdcode_lo][cmdcode_hi][status] */
    if (rsp[1] != EVT_CMD_COMPLETE) return -1;
    return (int)rsp[6];
}

/* ------------------------------------------------------------------ */
/*  BLE channel (HCI) - async command, event via interrupt            */
/* ------------------------------------------------------------------ */

static SemaphoreHandle_t hci_mutex;

static int hci_send_locked(uint16_t opcode, const void *params, uint8_t plen)
{
    ble_cmd_buf.type    = TL_BLECMD_PKT;
    ble_cmd_buf.cmdcode = opcode;
    ble_cmd_buf.plen    = plen;
    if (plen && params)
        memcpy(ble_cmd_buf.payload, params, plen);

    hci_cmd_status = 0xFF;
    ipcc_c1_set_flag(IPCC_CH_BLE);

    if (xSemaphoreTake(hci_resp_sem, pdMS_TO_TICKS(5000)) != pdTRUE)
        return -1;

    return (int)hci_cmd_status;
}

static int hci_send(uint16_t opcode, const void *params, uint8_t plen)
{
    if (hci_mutex) xSemaphoreTake(hci_mutex, portMAX_DELAY);
    int rc = hci_send_locked(opcode, params, plen);
    if (hci_mutex) xSemaphoreGive(hci_mutex);
    return rc;
}

static int hci_send_resp(uint16_t opcode, const void *params, uint8_t plen,
                         uint8_t *resp, uint8_t resp_max, uint8_t *resp_len)
{
    if (hci_mutex) xSemaphoreTake(hci_mutex, portMAX_DELAY);
    hci_resp_buf     = resp;
    hci_resp_buf_max = resp_max;
    hci_resp_buf_len = 0;

    int rc = hci_send_locked(opcode, params, plen);

    if (resp_len) *resp_len = hci_resp_buf_len;
    hci_resp_buf = NULL;
    if (hci_mutex) xSemaphoreGive(hci_mutex);
    return rc;
}

int ble_hci_send(uint16_t opcode, const void *params, uint8_t plen)
{
    return hci_send(opcode, params, plen);
}

int ble_hci_send_resp(uint16_t opcode, const void *params, uint8_t plen,
                      uint8_t *resp, uint8_t resp_max, uint8_t *resp_len)
{
    return hci_send_resp(opcode, params, plen, resp, resp_max, resp_len);
}

/* ------------------------------------------------------------------ */
/*  BLE initialisation                                                */
/* ------------------------------------------------------------------ */

static const ble_init_params_t ble_init_params = {
    .pBleBufferAddress        = 0,
    .BleBufferSize            = 0,
    .NumAttrRecord            = 68,
    .NumAttrServ              = 8,
    .AttrValueArrSize         = 1850,
    .NumOfLinks               = 2,
    .ExtendedPacketLengthEnable = 1,
    .PrWriteListSize          = 0x3A,
    .MblockCount              = 0x79,
    .AttMtu                   = 256,
    .PeripheralSca            = 500,
    .CentralSca               = 0,
    .LsSource                 = 1,
    .MaxConnEventLength       = 0xFFFFFFFF,
    .HsStartupTime            = 0x148,
    .ViterbiEnable            = 1,
    .Options                  = 0x00,           /* LL+Host */
    .HwVersion                = 0,
    .max_coc_initiator_nbr    = 32,
    .min_tx_power             = 0,
    .max_tx_power             = 0,
    .rx_model_config          = 1,
    .max_adv_set_nbr          = 1,
    .max_adv_data_len         = 1650,
    .tx_path_compens          = 0,
    .rx_path_compens          = 0,
    .ble_core_version         = 13,
    .Options_extension        = 0,
};

/* C2OPT=1 means flash boot on STM32WB (RM0434 §3.3.13). Normal. */

bool ble_init(void)
{
    if (ble_state == BLE_READY) return true;

    if (ble_state == BLE_DISABLED) {
        ble_state = BLE_READY;
        ble_start_background_adv();
        return true;
    }

    /* After the startup cold-boot reset, C2BOOT is cleared and CPU2
     * is stopped. We always do a fresh boot here. */

    c2_ready_sem = xSemaphoreCreateBinary();
    hci_resp_sem = xSemaphoreCreateBinary();
    hci_mutex    = xSemaphoreCreateMutex();
    scan_queue   = xQueueCreate(SCAN_QUEUE_LEN, sizeof(ble_scan_result_t));
    pair_queue   = xQueueCreate(PAIR_QUEUE_LEN, sizeof(ble_event_t));
    adv_timer    = xTimerCreate("adv", pdMS_TO_TICKS(200), pdFALSE,
                                NULL, adv_timer_cb);
    if (!c2_ready_sem || !hci_resp_sem || !scan_queue || !pair_queue) goto fail;

    /* Clear shared SRAM2A (MB_MEM1 + ref table).  MB_MEM2 buffers are
     * in SRAM1 (.bss) and already zeroed by startup. */
    memset((void *)&ref_table, 0, sizeof(ref_table));
    memset(&device_info_table, 0, sizeof(device_info_table));
    memset(&ble_table, 0, sizeof(ble_table));
    memset(&sys_table, 0, sizeof(sys_table));
    memset(&mm_table, 0, sizeof(mm_table));
    memset(&traces_table, 0, sizeof(traces_table));

    /* ---------- Reference table ---------- */
    ref_table.p_device_info_table = &device_info_table;
    ref_table.p_ble_table         = &ble_table;
    ref_table.p_thread_table      = NULL;
    ref_table.p_sys_table         = &sys_table;
    ref_table.p_mem_manager_table = &mm_table;
    ref_table.p_traces_table      = &traces_table;
    ref_table.p_mac_802_15_4_table = NULL;
    ref_table.p_zigbee_table      = NULL;
    ref_table.p_lld_tests_table   = NULL;
    ref_table.p_ble_lld_table     = NULL;

    /* ---------- Linked lists ---------- */
    lst_init((list_node_t *)&evt_queue);
    lst_init((list_node_t *)&sys_evt_queue);
    lst_init((list_node_t *)&free_buf_queue);
    lst_init(&local_free_q);
    lst_init((list_node_t *)&traces_evt_queue);

    /* ---------- System channel ---------- */
    sys_table.pcmd_buffer = (uint8_t *)&sys_cmd_buf;
    sys_table.sys_queue   = (uint8_t *)&sys_evt_queue;

    /* ---------- Memory manager ---------- */
    mm_table.spare_ble_buffer       = spare_ble_evt;
    mm_table.spare_sys_buffer       = spare_sys_evt;
    mm_table.blepool                = evt_pool;
    mm_table.blepoolsize            = EVT_POOL_SIZE;
    mm_table.pevt_free_buffer_queue = (uint8_t *)&free_buf_queue;
    mm_table.traces_evt_pool        = NULL;
    mm_table.tracespoolsize         = 0;

    /* ---------- Traces ---------- */
    traces_table.traces_queue = (uint8_t *)&traces_evt_queue;

    /* ---------- IPCC + HSEM hardware ---------- */
    RCC->AHB3ENR |= RCC_AHB3ENR_IPCCEN | RCC_AHB3ENR_HSEMEN
                  | RCC_AHB3ENR_AES2EN  | RCC_AHB3ENR_PKAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
    (void)RCC->AHB3ENR;

    /* Enable peripheral clocks for CPU2. */
    RCC->C2AHB3ENR |= RCC_C2AHB3ENR_IPCCEN
                    | RCC_C2AHB3ENR_FLASHEN
                    | RCC_C2AHB3ENR_HSEMEN
                    | RCC_C2AHB3ENR_PKAEN
                    | RCC_C2AHB3ENR_AES2EN
                    | RCC_C2AHB3ENR_RNGEN;
    RCC->C2AHB1ENR |= RCC_C2AHB1ENR_SRAM1EN;
    (void)RCC->C2AHB3ENR;

    /* Tell CPU2 we own the CLK48 domain (HSEM semaphore 5).
     * 1-step lock: reading RLR attempts the lock and returns the
     * result.  LOCK bit set with our COREID = success. */
    while ((HSEM->RLR[5] & HSEM_RLR_LOCK_Msk) == 0) {}

    IPCC->C1CR |= IPCC_C1CR_RXOIE;
    IPCC->C1CR |= IPCC_C1CR_TXFIE;

    /* Mask everything, then unmask system event channel */
    IPCC->C1MR = 0x003F003F;
    ipcc_unmask_rx(IPCC_CH_SYS);

    const uint32_t irq_prio = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1;
    NVIC_SetPriority(IPCC_C1_RX_IRQn, irq_prio);
    NVIC_SetPriority(IPCC_C1_TX_IRQn, irq_prio);
    NVIC_EnableIRQ(IPCC_C1_RX_IRQn);
    NVIC_EnableIRQ(IPCC_C1_TX_IRQn);

    /* ---------- Clock tree for RF subsystem (matches stock FW) ---------- */

    /* SMPS clock source = HSI, prescaler = /1 */
    RCC->SMPSCR = (RCC->SMPSCR & ~(RCC_SMPSCR_SMPSSEL_Msk | RCC_SMPSCR_SMPSDIV_Msk))
                | (1U << RCC_SMPSCR_SMPSSEL_Pos);   /* HSI */

    /* RF wakeup clock = LSE (required by radio sleep/wakeup) */
    RCC->CSR = (RCC->CSR & ~RCC_CSR_RFWKPSEL_Msk)
             | (1U << RCC_CSR_RFWKPSEL_Pos);         /* LSE */

    /* SMPS step-down mode, 1.40V, 80mA startup */
    PWR->CR5 = (PWR->CR5 & ~(PWR_CR5_SMPSVOS_Msk | PWR_CR5_SMPSSC_Msk))
             | (8U << PWR_CR5_SMPSVOS_Pos)
             | (4U << PWR_CR5_SMPSSC_Pos)
             | PWR_CR5_SMPSEN;
    for (volatile uint32_t t = 0; t < 1000000; t++)
        if (PWR->SR2 & PWR_SR2_SMPSF) break;

    /* ---------- HSE (32 MHz) + LSE (32.768 kHz) ---------- */
    /* Capacitor tuning must be set before HSE enable (Flipper hw = 0x26) */
    RCC->HSECR = (RCC->HSECR & ~RCC_HSECR_HSETUNE_Msk)
               | (0x26U << RCC_HSECR_HSETUNE_Pos);
    RCC->CR |= RCC_CR_HSEON;
    for (volatile uint32_t t = 0; t < 1000000; t++)
        if (RCC->CR & RCC_CR_HSERDY) break;
    if (!(RCC->CR & RCC_CR_HSERDY)) goto fail;

    /* LSE needs backup domain write access. */
    PWR->CR1 |= PWR_CR1_DBP;
    while (!(PWR->CR1 & PWR_CR1_DBP)) {}

    if (!(RCC->BDCR & RCC_BDCR_LSERDY)) {
        RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_LSEDRV_Msk)
                   | (3U << RCC_BDCR_LSEDRV_Pos);  /* HIGH drive */
        RCC->BDCR |= RCC_BDCR_LSEON;
        for (volatile uint32_t t = 0; t < 5000000; t++)
            if (RCC->BDCR & RCC_BDCR_LSERDY) break;
    }

    /* Ensure all mailbox writes are visible before booting CPU2. */
    __DSB();

    /* ---------- Boot CPU2 ---------- */
    EXTI->C2EMR2 |= (1U << 9);
    EXTI->RTSR2  |= (1U << 9);

    /* CPU2 may already be running (e.g. after DFU flash without power
     * cycle).  If the device info table is populated, CPU2 sent its
     * ready event before we were listening.  Re-signal it with __SEV
     * and set C2BOOT (no-op if already set). */
    __SEV();
    __WFE();
    PWR->CR4 |= PWR_CR4_C2BOOT;


    /* Poll DeviceInfoTable directly - CPU2 populates it at boot.
     * This works even if the ISR chain has issues. */
    volatile uint32_t *dev_info = (volatile uint32_t *)&device_info_table;
    for (int i = 0; i < 100; i++) {
        if (dev_info[0] != 0 || dev_info[1] != 0)
            goto c2_ok;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    cli_write("ble: CPU2 timeout\r\n");
    goto fail;

c2_ok:

    /* ---------- If FUS booted, start the wireless stack ---------- */
    if (c2_mode == 1) {
        cli_write("ble: FUS running, starting wireless stack...\r\n");
        shci_send(0xFC5A, NULL, 0);  /* SHCI_C2_FUS_StartWs */
        /* FUS_StartWs triggers a reboot - wait for new ready event */
        if (xSemaphoreTake(c2_ready_sem, pdMS_TO_TICKS(10000)) != pdTRUE) {
            cli_write("ble: stack start timeout\r\n");
            goto fail;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    /* ---------- BLE channel ---------- */
    ble_table.pcmd_buffer         = (uint8_t *)&ble_cmd_buf;
    ble_table.pcs_buffer          = cs_buffer;
    ble_table.pevt_queue          = (uint8_t *)&evt_queue;
    ble_table.phci_acl_data_buffer = NULL;
    ipcc_unmask_rx(IPCC_CH_BLE);

    /* ---------- SHCI_C2_BLE_Init ---------- */
    int rc = shci_send(SHCI_OPCODE_BLE_INIT,
                       &ble_init_params, sizeof(ble_init_params));
    if (rc != 0) { cli_printf("ble: BLE_Init rc=%d\r\n", rc); goto fail; }

    /* Use CPU2's default PESD timing protection for flash access. SEM7 mode can
     * leave CPU1 waiting for the semaphore for an entire busy BLE connection,
     * eventually tripping the supervision timeout during a sustained upload.
     * The flash driver polls PESD before every operation (and retains the SEM7
     * compatibility check), matching ST's dual-mode reference algorithm. */
    uint8_t flash_act = 0;  /* FLASH_ACTIVITY_CONTROL_PES */
    (void)shci_send(SHCI_OPCODE_SET_FLASH_ACT, &flash_act, 1);

    /* HCI Reset */
    rc = hci_send(0x0C03, NULL, 0);
    if (rc != 0) { cli_printf("ble: hci_reset rc=%d\r\n", rc); goto fail; }

    /* Derive a unique public address from the STM32 96-bit UID.
     * UID registers at 0x1FFF7590 (word 0/1/2). XOR the three words
     * down to 6 bytes so each device gets a distinct BLE address. */
    {
        volatile uint32_t *uid = (volatile uint32_t *)0x1FFF7590;
        uint32_t w0 = uid[0], w1 = uid[1], w2 = uid[2];
        uint8_t addr_cfg[8];
        addr_cfg[0] = 0x00;  /* offset = public address */
        addr_cfg[1] = 0x06;  /* length = 6 */
        addr_cfg[2] = (uint8_t)(w0 ^ w2);
        addr_cfg[3] = (uint8_t)((w0 >> 8) ^ (w2 >> 8));
        addr_cfg[4] = (uint8_t)((w0 >> 16) ^ (w2 >> 16));
        addr_cfg[5] = (uint8_t)(w1);
        addr_cfg[6] = (uint8_t)(w1 >> 8);
        addr_cfg[7] = (uint8_t)((w1 >> 16) | 0xC0);  /* static random marker */
        hci_send(0xFC0C, addr_cfg, sizeof(addr_cfg));
    }

    /* Set TX power (ACI_HAL_SET_TX_POWER_LEVEL 0xFC0F): high=1, level=0x19 */
    uint8_t txpow[] = { 0x01, 0x19 };
    hci_send(0xFC0F, txpow, sizeof(txpow));

    /* GATT Init (0xFD01) - must be called before GAP Init */
    rc = hci_send(0xFD01, NULL, 0);
    if (rc != 0) { cli_printf("ble: gatt_init rc=%d\r\n", rc); goto fail; }

    /* GAP Init (0xFC8A): role=CENTRAL|PERIPHERAL, no privacy. Keep the Device
     * Name characteristic consistent with the advertised identity. */
    char device_name[BLE_DEVICE_NAME_MAX + 1];
    uint8_t device_name_len = ble_format_device_name(device_name);
    uint8_t gap_params[] = { 0x05, 0x00, device_name_len };
    uint8_t gap_resp[6];
    uint8_t gap_resp_len = 0;
    rc = hci_send_resp(0xFC8A, gap_params, sizeof(gap_params),
                       gap_resp, sizeof(gap_resp), &gap_resp_len);
    if (rc != 0 || gap_resp_len != sizeof(gap_resp)) {
        cli_printf("ble: gap_init rc=%d len=%u\r\n", rc, gap_resp_len);
        goto fail;
    }

    uint16_t gap_service_handle = (uint16_t)gap_resp[0]
                                | ((uint16_t)gap_resp[1] << 8);
    uint16_t name_char_handle = (uint16_t)gap_resp[2]
                              | ((uint16_t)gap_resp[3] << 8);
    uint8_t name_update[6 + BLE_DEVICE_NAME_MAX];
    name_update[0] = (uint8_t)gap_service_handle;
    name_update[1] = (uint8_t)(gap_service_handle >> 8);
    name_update[2] = (uint8_t)name_char_handle;
    name_update[3] = (uint8_t)(name_char_handle >> 8);
    name_update[4] = 0;                    /* value offset */
    name_update[5] = device_name_len;
    memcpy(&name_update[6], device_name, device_name_len);
    rc = hci_send(ACI_GATT_UPDATE_CHAR_VALUE, name_update,
                  (uint8_t)(6 + device_name_len));
    if (rc != 0) {
        cli_printf("ble: device_name rc=%d\r\n", rc);
        goto fail;
    }

    ble_state = BLE_READY;

    /* Set default security for incoming connections (peripheral role).
     * DISPLAY_ONLY forces Passkey Entry: the device generates and displays a
     * random 6-digit passkey that the peer must enter - there is no path that
     * grants access without proving knowledge of that code. DISPLAY_YES_NO must
     * not be used here: it negotiates Numeric Comparison, whose MITM protection
     * relies on a human comparing the two displayed numbers and confirming on
     * the device. A headless device cannot do that meaningfully, and any
     * auto-confirm would let an attacker bond (and reach the CLI/file service)
     * with no secret - a remote-code-execution exposure. Matches the stock
     * Flipper serial profile (GapPairingPinCodeShow → DISPLAY_ONLY) and the
     * Chameleon target (io_caps = DISPLAY_ONLY). */
    uint8_t io_cap = BLE_IO_CAP_DISPLAY_ONLY;
    hci_send(ACI_GAP_SET_IO_CAPABILITY, &io_cap, 1);
    uint8_t auth[] = {
        0x01,       /* bonding mode */
        0x01,       /* MITM protection */
        0x01,       /* secure connections */
        0x00,       /* keypress notification */
        0x07,       /* min encryption key size */
        0x10,       /* max encryption key size */
        0x01,       /* use_fixed_pin = NO → stack fires GAP_EVT_PASS_KEY_REQ */
        0x00, 0x00, 0x00, 0x00, /* fixed_pin (ignored when use_fixed_pin=1) */
        0x00,       /* identity address type */
    };
    hci_send(ACI_GAP_SET_AUTH_REQUIREMENT, auth, sizeof(auth));

    /* Start background advertising so bonded peers can reconnect. */
    ble_start_background_adv();

    ble_serial_init();

    return true;

fail:
    ble_state = BLE_ERROR;
    return false;
}

/* ------------------------------------------------------------------ */
/*  Background advertising                                            */
/* ------------------------------------------------------------------ */

static void ble_start_background_adv(void)
{
    if (ble_state != BLE_READY) return;

    /* This profile intentionally exposes one physical peripheral link and
     * multiplexes independent CLI sessions over it. A delayed retry from a
     * failed/aborted connection attempt must not disturb a link that has since
     * completed successfully. */
    for (int i = 0; i < BLE_MAX_CONN; i++) {
        if (ble_conns[i].active) {
            adv_restart_needed = false;
            return;
        }
    }

    char full[BLE_DEVICE_NAME_MAX + 1];
    uint8_t nlen = ble_format_device_name(full);
    uint8_t name_field_len = nlen + 1;
    uint8_t p[14 + BLE_DEVICE_NAME_MAX];
    uint8_t plen = 13 + name_field_len;
    /* Idle power policy: 160-320 ms while in use, 1000-1200 ms when idle
     * (~6x less radio duty; discovery still works, any activity restores
     * the fast interval via ble_adv_refresh). Units: 0.625 ms. */
    uint16_t adv_min = fz_power_adv_slow() ? 0x0640 : 0x0100;
    uint16_t adv_max = fz_power_adv_slow() ? 0x0780 : 0x0200;
    p[0] = 0x00;                   /* ADV_IND */
    p[1] = (uint8_t)adv_min; p[2] = (uint8_t)(adv_min >> 8);
    p[3] = (uint8_t)adv_max; p[4] = (uint8_t)(adv_max >> 8);
    p[5] = 0x00;                   /* own_addr_type */
    p[6] = 0x00;
    p[7] = name_field_len;
    p[8] = AD_TYPE_FULL_NAME;
    memcpy(&p[9], full, nlen);
    uint8_t off = 8 + name_field_len;
    p[off] = 0x00;
    p[off+1] = 0x00; p[off+2] = 0x00;
    p[off+3] = 0x00; p[off+4] = 0x00;

    /* Clear any stale GAP advertising state first (harmless / ignored if not
     * advertising), then (re)start. The result must be checked: right after a
     * disconnect the stack may still be finishing teardown and SET_DISCOVERABLE
     * returns an error. Clearing adv_restart_needed unconditionally would then
     * leave the device silently non-advertising - never connectable again until
     * reboot. Leave the flag set on failure so the next poll/timer retries
     * until it actually succeeds. */
    hci_send(ACI_GAP_SET_NON_DISCOVERABLE, NULL, 0);
    int rc = hci_send(ACI_GAP_SET_DISCOVERABLE, p, plen);
    adv_restart_needed = (rc != 0);
}

/* Idle-policy hook (platforms/flipper/power.c, timer-task context): re-issue
 * background advertising so the interval matches the current idle state.
 * With a link up the caller skips this - the new interval applies on the
 * next advertising restart after disconnect. */
void ble_adv_refresh(void)
{
    if (ble_state != BLE_READY) return;
    for (int i = 0; i < BLE_MAX_CONN; i++)
        if (ble_conns[i].active) return;
    ble_start_background_adv();
}

void ble_shutdown(void)
{
    if (ble_state != BLE_READY) return;

    for (int i = 0; i < BLE_MAX_CONN; i++) {
        if (ble_conns[i].active)
            ble_pair_disconnect(ble_conns[i].handle);
    }
    hci_send(ACI_GAP_SET_NON_DISCOVERABLE, NULL, 0);
    adv_restart_needed = false;
    if (adv_timer) xTimerStop(adv_timer, 0);
    ble_state = BLE_DISABLED;
}

bool ble_is_active(void)
{
    return ble_state == BLE_READY;
}

/* CPU2 (the wireless coprocessor) is running and can touch the shared flash bus
 * whenever it has been started - which is both BLE_READY (advertising/connected)
 * and BLE_DISABLED (`ble off` only stops advertising; it does not halt CPU2).
 * Flash writes must coordinate with CPU2 (SEM2/SEM7) in both states, else an
 * uncoordinated write races CPU2 and corrupts a double-word. Only BLE_UNINIT
 * (pre-ble_init, e.g. boot-time mkfs) is truly CPU2-free. */
bool ble_cpu2_running(void)
{
    return ble_state == BLE_READY || ble_state == BLE_DISABLED;
}

/* Tell CPU2 a long flash erase is about to start (ON) / has finished (OFF) so it
 * reschedules its connection events around the ~20 ms stall instead of being
 * cut off mid-event. Without this advance notice, a page erase during an upload
 * silently drops the host's in-flight packets (the WS can't service the radio
 * while the flash bus is busy). Mirrors furi_hal_flash's SHCI_C2_FLASH_Erase-
 * Activity. No-op when CPU2 isn't running - then there's no radio to protect. */
int ble_flash_erase_activity(int on)
{
    if (!ble_cpu2_running()) return 0;
    uint8_t v = on ? 0x01 : 0x00;
    return shci_send(SHCI_OPCODE_FLASH_ERASE_ACT, &v, 1);
}

void ble_activate_fus(void)
{
    if (!ble_init()) return;
    shci_send(SHCI_OPCODE_FUS_GET_STATE, NULL, 0);

    /* Second GET_STATE triggers FUS activation (AN5185). CPU2 may reset
     * before responding, so fire-and-forget: write the command buffer
     * and signal IPCC without waiting for the response. */
    sys_cmd_buf.type    = TL_SYSCMD_PKT;
    sys_cmd_buf.cmdcode = SHCI_OPCODE_FUS_GET_STATE;
    sys_cmd_buf.plen    = 0;
    __DSB();
    ipcc_c1_set_flag(IPCC_CH_SYS);
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void ble_resume_adv(void)
{
    if (adv_restart_needed)
        ble_start_background_adv();
}

/* ------------------------------------------------------------------ */
/*  BLE scan                                                          */
/* ------------------------------------------------------------------ */

int ble_scan(ble_scan_cb_t cb, uint32_t duration_ms)
{
    if (!ble_init()) return -1;

    /* Stop advertising before scanning - GAP can't do both */
    hci_send(ACI_GAP_SET_NON_DISCOVERABLE, NULL, 0);

    /* ACI GAP Start General Discovery Proc (0xFC97):
     * scan_interval(2) scan_window(2) own_addr_type(1) filter_dup(1) */
    uint8_t disc_params[] = {
        0x40, 0x00,   /* scan_interval = 0x0040 (40ms) */
        0x30, 0x00,   /* scan_window   = 0x0030 (30ms) */
        0x00,         /* own_address_type = public */
        0x00          /* filter_duplicates = no */
    };
    int sp = hci_send(0xFC97, disc_params, sizeof(disc_params));
    if (sp != 0) { cli_printf("ble: disc rc=%d\r\n", sp); return -1; }

    int count = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(duration_ms);

    while (xTaskGetTickCount() < deadline) {
        ble_scan_result_t r;
        TickType_t remain = deadline - xTaskGetTickCount();
        if (remain > pdMS_TO_TICKS(duration_ms)) break;

        if (xQueueReceive(scan_queue, &r, remain) == pdTRUE) {
            count++;
            if (cb) cb(&r);
        }
    }

    /* ACI GAP Terminate Gap Proc (0xFC9D): procedure=0x02 (general discovery) */
    uint8_t term = 0x02;
    hci_send(0xFC9D, &term, 1);

    /* Drain any leftover results */
    ble_scan_result_t discard;
    while (xQueueReceive(scan_queue, &discard, 0) == pdTRUE) {}

    ble_start_background_adv();
    return count;
}

/* ------------------------------------------------------------------ */
/*  BLE pairing                                                       */
/* ------------------------------------------------------------------ */

int ble_pair_setup_security(uint8_t io_cap)
{
    if (!ble_init()) return -1;

    int rc = hci_send(ACI_GAP_SET_IO_CAPABILITY, &io_cap, 1);
    if (rc != 0) return -1;

    /* bonding(1) mitm(1) sc(1) keypress(1) min_key(1) max_key(1)
     * use_fixed_pin(1) fixed_pin(4) identity_addr(1) = 11 bytes */
    uint8_t auth[] = {
        0x01,       /* bonding enabled */
        0x01,       /* MITM required */
        0x01,       /* Secure Connections optional */
        0x00,       /* keypress notifications off */
        0x07,       /* min encryption key size */
        0x10,       /* max encryption key size (16) */
        0x01,       /* do not use fixed pin - request from app */
        0x00, 0x00, 0x00, 0x00,  /* fixed pin (unused) */
        0x00,       /* public identity address */
    };
    rc = hci_send(ACI_GAP_SET_AUTH_REQUIREMENT, auth, sizeof(auth));
    if (rc != 0) return -1;

    /* Drain stale pair events */
    ble_event_t drain;
    while (xQueueReceive(pair_queue, &drain, 0) == pdTRUE) {}

    return 0;
}

void ble_pair_set_manual(bool on)
{
    pair_in_progress = on;
}

int ble_pair_connect(const uint8_t *addr, uint8_t addr_type)
{
    if (!ble_init()) return -1;

    /* scan_interval(2) scan_window(2) peer_addr_type(1) peer_addr(6)
     * own_addr_type(1) conn_interval_min(2) conn_interval_max(2)
     * conn_latency(2) supervision_timeout(2) min_ce(2) max_ce(2)
     * = 24 bytes */
    uint8_t p[24];
    p[0]  = 0x10; p[1]  = 0x00;   /* scan_interval = 0x0010 (10 ms) */
    p[2]  = 0x10; p[3]  = 0x00;   /* scan_window   = 0x0010 (10 ms) */
    p[4]  = addr_type;
    memcpy(&p[5], addr, 6);       /* peer address (little-endian) */
    p[11] = 0x00;                  /* own_addr_type = public */
    p[12] = 0x18; p[13] = 0x00;   /* conn_interval_min = 0x0018 (30 ms) */
    p[14] = 0x28; p[15] = 0x00;   /* conn_interval_max = 0x0028 (50 ms) */
    p[16] = 0x00; p[17] = 0x00;   /* conn_latency = 0 */
    p[18] = 0xC8; p[19] = 0x00;   /* supervision_timeout = 0x00C8 (2 s) */
    p[20] = 0x00; p[21] = 0x00;   /* min_ce_length */
    p[22] = 0x00; p[23] = 0x00;   /* max_ce_length */

    int rc = hci_send(ACI_GAP_CREATE_CONNECTION, p, sizeof(p));
    return (rc == 0) ? 0 : -1;
}

int ble_pair_initiate(uint16_t conn_handle)
{
    /* conn_handle(2) force_rebond(1) */
    uint8_t p[3];
    p[0] = (uint8_t)(conn_handle & 0xFF);
    p[1] = (uint8_t)(conn_handle >> 8);
    p[2] = 0x01;  /* force fresh pairing even if already bonded */
    return (hci_send(ACI_GAP_SEND_PAIRING_REQ, p, sizeof(p)) == 0) ? 0 : -1;
}

int ble_pair_send_passkey(uint16_t conn_handle, uint32_t passkey)
{
    uint8_t p[6];
    p[0] = (uint8_t)(conn_handle & 0xFF);
    p[1] = (uint8_t)(conn_handle >> 8);
    p[2] = (uint8_t)(passkey & 0xFF);
    p[3] = (uint8_t)((passkey >> 8) & 0xFF);
    p[4] = (uint8_t)((passkey >> 16) & 0xFF);
    p[5] = (uint8_t)((passkey >> 24) & 0xFF);
    return (hci_send(ACI_GAP_PASS_KEY_RESP, p, sizeof(p)) == 0) ? 0 : -1;
}

int ble_pair_numeric_confirm(uint16_t conn_handle, bool accept)
{
    uint8_t p[3];
    p[0] = (uint8_t)(conn_handle & 0xFF);
    p[1] = (uint8_t)(conn_handle >> 8);
    p[2] = accept ? 0x01 : 0x00;
    return (hci_send(0xFCA5, p, sizeof(p)) == 0) ? 0 : -1;
}

int ble_pair_wait_event(ble_event_t *evt, uint32_t timeout_ms)
{
    if (!pair_queue) return -1;
    return (xQueueReceive(pair_queue, evt, pdMS_TO_TICKS(timeout_ms)) == pdTRUE)
           ? 0 : -1;
}

int ble_pair_disconnect(uint16_t conn_handle)
{
    /* conn_handle(2) reason(1) */
    uint8_t p[3];
    p[0] = (uint8_t)(conn_handle & 0xFF);
    p[1] = (uint8_t)(conn_handle >> 8);
    p[2] = 0x13;  /* remote user terminated connection */
    int rc = hci_send(ACI_GAP_TERMINATE, p, sizeof(p));
    /* Disconnection event handler sets adv_restart_needed;
     * the next ble_resume_adv() call will restart advertising. */
    return (rc == 0) ? 0 : -1;
}

uint32_t ble_generate_passkey(void)
{
    /* The RNG is clocked from CLK48 (HSI48), a domain CPU2 (the wireless stack)
     * gates in its low-power states. Touching the RNG while that clock is down
     * BusFaults, so re-assert HSI48 and wait for it before any RNG access; we
     * already hold the CLK48 HSEM (claimed in ble_init), so driving HSI48 from
     * CPU1 is the sanctioned arbitration, and CLK48SEL was routed to HSI48 at
     * boot (system.c) and persists. */
    RCC->CRRCR |= RCC_CRRCR_HSI48ON;
    uint32_t spin;
    for (spin = 0; spin < 100000; spin++)
        if (RCC->CRRCR & RCC_CRRCR_HSI48RDY) break;

    if (RCC->CRRCR & RCC_CRRCR_HSI48RDY) {
        RCC->AHB3ENR |= RCC_AHB3ENR_RNGEN;
        (void)RCC->AHB3ENR;
        RNG->CR |= RNG_CR_RNGEN;

        uint32_t sr = 0;
        for (spin = 0; spin < 100000; spin++) {
            sr = RNG->SR;
            if (sr & (RNG_SR_DRDY | RNG_SR_CECS | RNG_SR_SECS)) break;
        }
        /* Only consume DR on a clean ready - reading an un-ready or clock/seed-
         * errored RNG yields garbage and can fault. */
        if ((sr & RNG_SR_DRDY) && !(sr & (RNG_SR_CECS | RNG_SR_SECS)))
            return RNG->DR % 1000000;
    }

    /* RNG genuinely unavailable (HSI48 never came up / persistent clock error).
     * Practically unreachable, but fall back to on-hand timing entropy rather
     * than crash or return a constant 000000. */
    uint32_t e = xTaskGetTickCount() ^ SysTick->VAL;
    return e % 1000000;
}

int ble_get_connections(ble_conn_info_t *out, int max)
{
    ble_resume_adv();
    int n = 0;
    for (int i = 0; i < BLE_MAX_CONN && n < max; i++) {
        if (ble_conns[i].active)
            out[n++] = ble_conns[i];
    }
    return n;
}

/* ACI_GAP_GET_BONDED_DEVICES (0xFCA3): response is
 * num_devices(1) then per device: addr_type(1) addr(6) */
int ble_get_bonded_devices(ble_bonded_dev_t *out, int max)
{
    if (!ble_init()) return 0;
    ble_resume_adv();

    uint8_t resp[64];
    uint8_t rlen = 0;
    int rc = hci_send_resp(0xFCA3, NULL, 0, resp, sizeof(resp), &rlen);
    if (rc != 0 || rlen < 1) return 0;

    int num = resp[0];
    int n = 0;
    const uint8_t *p = &resp[1];
    for (int i = 0; i < num && n < max && (int)(p - resp) + 7 <= (int)rlen; i++) {
        out[n].addr_type = *p++;
        memcpy(out[n].addr, p, 6); p += 6;
        n++;
    }
    return n;
}

int ble_remove_bond(const uint8_t *addr, uint8_t addr_type)
{
    if (!ble_init()) return -1;
    uint8_t p[7];
    p[0] = addr_type;
    memcpy(&p[1], addr, 6);
    return (hci_send(0xFCA7, p, sizeof(p)) == 0) ? 0 : -1;
}

int ble_clear_all_bonds(void)
{
    if (!ble_init()) return -1;
    return (hci_send(0xFC94, NULL, 0) == 0) ? 0 : -1;
}
