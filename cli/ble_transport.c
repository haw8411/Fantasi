/* BLE GATT transport for the Fantasi host CLI.
 *
 * Uses sd-bus (libsystemd) to interact with BlueZ's D-Bus API.
 * Connects to a Fantasi device - pairing it first if not already bonded (via a
 * KeyboardOnly agent, Passkey Entry: the device displays a 6-digit code on its
 * screen and USB log, the host types it) - resolves the serial service
 * characteristics (auto-detecting the Flipper or Chameleon service UUIDs), and
 * provides read/write over GATT notifications and writes. */

#define _GNU_SOURCE
#include "ble_transport.h"
#include "../proto/ble_mux.h"
#include "fantasi.pb.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <systemd/sd-bus.h>

/* Flipper serial service UUIDs */
#define SVC_UUID_FZ  "8fe5b3d5-2e7f-4a98-2a48-7acc60fe0000"
#define TX_UUID_FZ   "19ed82ae-ed21-4c9d-4145-228e61fe0000"
#define RX_UUID_FZ   "19ed82ae-ed21-4c9d-4145-228e62fe0000"

/* Nordic UART Service UUIDs (CU) */
#define SVC_UUID_NUS "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define TX_UUID_NUS  "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define RX_UUID_NUS  "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

static sd_bus *bus;
static char device_path[256];
static char forced_addr[64];
static char wanted_name[64];   /* restrict discovery to this device name (empty = any) */

void ble_transport_set_addr(const char *addr)
{
    snprintf(forced_addr, sizeof(forced_addr), "%s", addr);
}

void ble_transport_set_name(const char *name)
{
    snprintf(wanted_name, sizeof(wanted_name), "%s", name ? name : "");
}
static char rx_char_path[256];
static char tx_char_path[256];
static bool connected;
static uint16_t write_mtu = 20;
static sd_bus_slot *tx_match_slot;  /* TX-notify match; replaced on reconnect */
static sd_bus_slot *device_match_slot; /* live Device1.Connected changes */
static bool physical_connected;
static bool services_resolved;

typedef enum {
    RECONNECT_IDLE,
    RECONNECT_CONNECT_PENDING,
    RECONNECT_NOTIFY_PENDING,
} reconnect_phase_t;

static reconnect_phase_t reconnect_phase;
static sd_bus_slot *reconnect_call_slot;
static struct {
    int done;
    int result;
} reconnect_call;

/* Ring buffer for incoming indications */
/* Hold a complete in-flight download because BlueZ may deliver many
 * notifications without per-chunk acknowledgement. Regression risk: a smaller
 * ring can drop bytes in rx_push() and desynchronize protobuf framing. */
#define RX_BUF_SIZE 65536
static uint8_t rx_buf[RX_BUF_SIZE];
static size_t  rx_head, rx_tail;
static fantasi_ble_mux_response_rx_t response_rx;
static uint8_t response_frame[2 + CliResponse_size];
static uint32_t response_session;

static size_t rx_available(void)
{
    return (rx_head - rx_tail + RX_BUF_SIZE) % RX_BUF_SIZE;
}

static void rx_push(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        size_t next = (rx_head + 1) % RX_BUF_SIZE;
        if (next == rx_tail) return;
        rx_buf[rx_head] = data[i];
        rx_head = next;
    }
}

static void rx_notification(const uint8_t *data, size_t len)
{
    size_t complete_len = 0;
    fantasi_ble_mux_response_result_t result =
        fantasi_ble_mux_response_accept(&response_rx,
                                        response_frame, sizeof(response_frame),
                                        data, len, &complete_len);
    if (result == FANTASI_BLE_MUX_RESPONSE_RAW && !response_session)
        rx_push(data, len);                    /* old/implicit byte stream */
    else if (result == FANTASI_BLE_MUX_RESPONSE_COMPLETE &&
             (!response_session || response_rx.session == response_session))
        rx_push(response_frame, complete_len); /* exactly one de-duplicated frame */
}

void ble_transport_set_response_session(uint32_t session)
{
    response_session = session;
}

/* ---- D-Bus helpers ---- */

static int get_string_prop(const char *path, const char *iface,
                           const char *prop, char *out, size_t out_len)
{
    sd_bus_error err = SD_BUS_ERROR_NULL;
    char *val = NULL;
    int r = sd_bus_get_property_string(bus, "org.bluez", path, iface,
                                       prop, &err, &val);
    sd_bus_error_free(&err);
    if (r < 0 || !val) return -1;
    snprintf(out, out_len, "%s", val);
    free(val);
    return 0;
}

static int has_uuid_in_array(const char *path, const char *uuid)
{
    sd_bus_error err = SD_BUS_ERROR_NULL;
    sd_bus_message *reply = NULL;
    int r = sd_bus_get_property(bus, "org.bluez", path,
                                "org.bluez.Device1", "UUIDs",
                                &err, &reply, "as");
    sd_bus_error_free(&err);
    if (r < 0) return 0;

    r = sd_bus_message_enter_container(reply, 'a', "s");
    if (r < 0) { sd_bus_message_unref(reply); return 0; }

    const char *u;
    int found = 0;
    while (sd_bus_message_read_basic(reply, 's', &u) > 0) {
        if (strcasecmp(u, uuid) == 0) { found = 1; break; }
    }

    sd_bus_message_unref(reply);
    return found;
}

/* ---- Device discovery ---- */

/* "/org/bluez/hci0/dev_F8_F5_60_D9_51_CE" -> "F8:F5:60:D9:51:CE" */
static void path_to_addr(const char *path, char *out, size_t n)
{
    const char *p = strstr(path, "/dev_");
    if (!p) { if (n) out[0] = '\0'; return; }
    p += 5;
    size_t i = 0;
    for (; *p && i + 1 < n; p++) out[i++] = (*p == '_') ? ':' : *p;
    out[i] = '\0';
}

/* Pick the Fantasi device to use. Returns 0 (device_path set), -1 (none
 * found), or -2 (several found, ambiguous - candidates listed, caller should
 * tell the user to disambiguate with --ble-addr). With several candidates a
 * single bonded one is chosen automatically (the usual "my paired device"). */
static int find_fantasi_device(void)
{
    sd_bus_error err = SD_BUS_ERROR_NULL;
    sd_bus_message *reply = NULL;

    int r = sd_bus_call_method(bus, "org.bluez", "/",
                               "org.freedesktop.DBus.ObjectManager",
                               "GetManagedObjects", &err, &reply, "");
    sd_bus_error_free(&err);
    if (r < 0) return -1;

    r = sd_bus_message_enter_container(reply, 'a', "{oa{sa{sv}}}");
    if (r < 0) { sd_bus_message_unref(reply); return -1; }

    struct { char path[256]; char name[64]; int paired; } cand[8];
    int n = 0;

    while (sd_bus_message_enter_container(reply, 'e', "oa{sa{sv}}") > 0) {
        const char *obj_path;
        sd_bus_message_read_basic(reply, 'o', &obj_path);

        if (!strstr(obj_path, "/dev_")) {
            sd_bus_message_skip(reply, "a{sa{sv}}");
            sd_bus_message_exit_container(reply);
            continue;
        }
        sd_bus_message_skip(reply, "a{sa{sv}}");
        sd_bus_message_exit_container(reply);

        char dev_name[64] = "";
        get_string_prop(obj_path, "org.bluez.Device1", "Name",
                        dev_name, sizeof(dev_name));
        if (strncmp(dev_name, "Fantasi ", 8) != 0 &&
            !has_uuid_in_array(obj_path, SVC_UUID_FZ) &&
            !has_uuid_in_array(obj_path, SVC_UUID_NUS))
            continue;

        /* Name filter (--name): the advertised name is "Fantasi <name>", so match
         * against the suffix. A UUID-only match with no readable name can't be
         * confirmed against the requested name, so drop it. */
        if (wanted_name[0] &&
            (strncmp(dev_name, "Fantasi ", 8) != 0 ||
             strcmp(dev_name + 8, wanted_name) != 0))
            continue;

        if (n < (int)(sizeof(cand) / sizeof(cand[0]))) {
            snprintf(cand[n].path, sizeof(cand[n].path), "%s", obj_path);
            snprintf(cand[n].name, sizeof(cand[n].name), "%s",
                     dev_name[0] ? dev_name : "Fantasi");
            int paired = 0;
            sd_bus_error perr = SD_BUS_ERROR_NULL;
            sd_bus_get_property_trivial(bus, "org.bluez", obj_path,
                    "org.bluez.Device1", "Paired", &perr, 'b', &paired);
            sd_bus_error_free(&perr);
            cand[n].paired = paired;
            n++;
        }
    }
    sd_bus_message_unref(reply);

    if (n == 0) return -1;
    if (n == 1) {
        snprintf(device_path, sizeof(device_path), "%s", cand[0].path);
        return 0;
    }

    /* Several candidates: auto-select if exactly one is bonded. */
    int paired_idx = -1, paired_count = 0;
    for (int i = 0; i < n; i++)
        if (cand[i].paired) { paired_idx = i; paired_count++; }
    if (paired_count == 1) {
        char addr[32];
        path_to_addr(cand[paired_idx].path, addr, sizeof(addr));
        printf("ble: %d Fantasi devices in range; using bonded %s (%s)\n",
               n, cand[paired_idx].name, addr);
        snprintf(device_path, sizeof(device_path), "%s", cand[paired_idx].path);
        return 0;
    }

    fprintf(stderr,
            "ble: %d Fantasi devices in range - pick one with --ble-addr=<addr>:\n", n);
    for (int i = 0; i < n; i++) {
        char addr[32];
        path_to_addr(cand[i].path, addr, sizeof(addr));
        fprintf(stderr, "  %s  %s%s\n", addr, cand[i].name,
                cand[i].paired ? " (paired)" : "");
    }
    return -2;
}

/* ---- Characteristic resolution ---- */

static int find_char_by_uuid(const char *uuid, char *out, size_t out_len)
{
    sd_bus_error err = SD_BUS_ERROR_NULL;
    sd_bus_message *reply = NULL;

    int r = sd_bus_call_method(bus, "org.bluez", "/",
                               "org.freedesktop.DBus.ObjectManager",
                               "GetManagedObjects", &err, &reply, "");
    sd_bus_error_free(&err);
    if (r < 0) return -1;

    r = sd_bus_message_enter_container(reply, 'a', "{oa{sa{sv}}}");
    if (r < 0) { sd_bus_message_unref(reply); return -1; }

    while (sd_bus_message_enter_container(reply, 'e', "oa{sa{sv}}") > 0) {
        const char *obj_path;
        sd_bus_message_read_basic(reply, 'o', &obj_path);
        sd_bus_message_skip(reply, "a{sa{sv}}");
        sd_bus_message_exit_container(reply);

        if (!strstr(obj_path, device_path) || !strstr(obj_path, "/char"))
            continue;

        char char_uuid[64];
        if (get_string_prop(obj_path, "org.bluez.GattCharacteristic1",
                            "UUID", char_uuid, sizeof(char_uuid)) == 0) {
            if (strcasecmp(char_uuid, uuid) == 0) {
                snprintf(out, out_len, "%s", obj_path);
                sd_bus_message_unref(reply);
                return 0;
            }
        }
    }

    sd_bus_message_unref(reply);
    return -1;
}

/* ---- Notification callback ---- */

static int on_properties_changed(sd_bus_message *msg, void *userdata,
                                 sd_bus_error *ret_error)
{
    (void)userdata;
    (void)ret_error;

    const char *iface;
    if (sd_bus_message_read_basic(msg, 's', &iface) < 0) return 0;
    if (strcmp(iface, "org.bluez.GattCharacteristic1") != 0) return 0;

    if (sd_bus_message_enter_container(msg, 'a', "{sv}") < 0) return 0;

    while (sd_bus_message_enter_container(msg, 'e', "sv") > 0) {
        const char *prop;
        sd_bus_message_read_basic(msg, 's', &prop);

        if (strcmp(prop, "Value") == 0) {
            sd_bus_message_enter_container(msg, 'v', "ay");
            const void *data;
            size_t len;
            if (sd_bus_message_read_array(msg, 'y', &data, &len) >= 0)
                rx_notification((const uint8_t *)data, len);
            sd_bus_message_exit_container(msg);
        } else {
            sd_bus_message_skip(msg, "v");
        }

        sd_bus_message_exit_container(msg);
    }

    return 0;
}

static int on_device_properties_changed(sd_bus_message *msg, void *userdata,
                                        sd_bus_error *ret_error)
{
    (void)userdata;
    (void)ret_error;

    const char *iface;
    if (sd_bus_message_read_basic(msg, 's', &iface) < 0) return 0;
    if (strcmp(iface, "org.bluez.Device1") != 0) return 0;
    if (sd_bus_message_enter_container(msg, 'a', "{sv}") < 0) return 0;

    while (sd_bus_message_enter_container(msg, 'e', "sv") > 0) {
        const char *prop;
        if (sd_bus_message_read_basic(msg, 's', &prop) < 0) {
            sd_bus_message_exit_container(msg);
            break;
        }

        if (strcmp(prop, "Connected") == 0) {
            int value = 0;
            if (sd_bus_message_enter_container(msg, 'v', "b") > 0) {
                if (sd_bus_message_read_basic(msg, 'b', &value) >= 0) {
                    physical_connected = value != 0;
                    if (!physical_connected) {
                        connected = false;
                        services_resolved = false;
                    }
                }
                sd_bus_message_exit_container(msg);
            } else {
                sd_bus_message_skip(msg, "v");
            }
        } else if (strcmp(prop, "ServicesResolved") == 0) {
            int value = 0;
            if (sd_bus_message_enter_container(msg, 'v', "b") > 0) {
                if (sd_bus_message_read_basic(msg, 'b', &value) >= 0)
                    services_resolved = value != 0;
                sd_bus_message_exit_container(msg);
            } else {
                sd_bus_message_skip(msg, "v");
            }
        } else {
            sd_bus_message_skip(msg, "v");
        }
        sd_bus_message_exit_container(msg);
    }
    return 0;
}

static int on_reconnect_reply(sd_bus_message *reply, void *userdata,
                              sd_bus_error *ret_error)
{
    (void)userdata;
    (void)ret_error;
    int message_errno = sd_bus_message_get_errno(reply);
    reconnect_call.result = sd_bus_message_is_method_error(reply, NULL)
        ? (message_errno > 0 ? -message_errno : -EIO) : 0;
    if (reconnect_call.result == 0 &&
        reconnect_phase == RECONNECT_CONNECT_PENDING)
        physical_connected = true;
    reconnect_call.done = 1;
    return 1;
}

static void reconnect_call_cancel(void)
{
    if (reconnect_call_slot) {
        sd_bus_slot_unref(reconnect_call_slot);
        reconnect_call_slot = NULL;
    }
    reconnect_phase = RECONNECT_IDLE;
    reconnect_call.done = 0;
    reconnect_call.result = 0;
}

/* Queue a reconnect phase without waiting for its BlueZ reply.  Readline's
 * event hook calls ble_transport_reconnect() periodically; keeping the slot
 * alive lets sd-bus finish the call across those short ticks while `exit`,
 * Ctrl-D, and normal line editing remain immediately available. */
static int reconnect_call_start(reconnect_phase_t phase, const char *path,
                                const char *iface, const char *method,
                                uint64_t timeout_us)
{
    sd_bus_message *message = NULL;
    int r = sd_bus_message_new_method_call(bus, &message, "org.bluez", path,
                                           iface, method);
    if (r >= 0) {
        reconnect_call.done = 0;
        reconnect_call.result = 0;
        reconnect_phase = phase;
        r = sd_bus_call_async(bus, &reconnect_call_slot, message,
                              on_reconnect_reply, NULL, timeout_us);
    }
    sd_bus_message_unref(message);
    if (r < 0) reconnect_call_cancel();
    return r;
}

/* ---- Pairing agent ----
 *
 * Both CU and FZ pair by Passkey Entry: the device generates and displays a
 * 6-digit code (on its screen and USB log), and the host enters it. The host
 * registers a KeyboardOnly agent so BlueZ uses Passkey Entry; RequestPasskey
 * prompts for the code. (FZ's DISPLAY_YES_NO + use_fixed_pin=NO config maps,
 * against a KeyboardOnly initiator, to Passkey-Entry-with-the-device-displaying
 * - identical UX to CU's DisplayOnly device.) */
#define AGENT_PATH "/fantasi/agent"

/* Passkey Entry: the device displays a code (USB log / its screen); prompt. */
static int agent_request_passkey(sd_bus_message *m, void *userdata,
                                 sd_bus_error *err)
{
    (void)userdata; (void)err;
    printf("\nble: the device needs a passkey to pair.\n"
           "ble: read it from the device's screen, or its USB log "
           "(run 'log', look for 'BLE pair code: NNNNNN').\n"
           "Enter passkey: ");
    fflush(stdout);
    char line[32];
    if (!fgets(line, sizeof(line), stdin))
        return sd_bus_error_set(err, "org.bluez.Error.Canceled", "no input");
    uint32_t pk = (uint32_t)strtoul(line, NULL, 10);
    return sd_bus_reply_method_return(m, "u", pk);
}

/* Accept-everything fallbacks for the other Agent1 methods (none should fire
 * for Passkey Entry, but a stray call shouldn't abort the pairing). */
static int agent_reply_empty(sd_bus_message *m, void *userdata, sd_bus_error *err)
{
    (void)userdata; (void)err;
    return sd_bus_reply_method_return(m, "");
}

static const sd_bus_vtable agent_vtable[] = {
    SD_BUS_VTABLE_START(0),
    SD_BUS_METHOD("Release",            "",   "",  agent_reply_empty,    0),
    SD_BUS_METHOD("RequestPasskey",     "o",  "u", agent_request_passkey,0),
    SD_BUS_METHOD("RequestConfirmation","ou", "",  agent_reply_empty,    0),
    SD_BUS_METHOD("RequestAuthorization","o", "",  agent_reply_empty,    0),
    SD_BUS_METHOD("AuthorizeService",   "os", "",  agent_reply_empty,    0),
    SD_BUS_METHOD("Cancel",             "",   "",  agent_reply_empty,    0),
    SD_BUS_VTABLE_END
};

static int  pair_done, pair_ok;
static char pair_err_name[128];   /* D-Bus error name from the last Pair failure */

static int on_pair_reply(sd_bus_message *reply, void *userdata, sd_bus_error *e)
{
    (void)userdata; (void)e;
    pair_err_name[0] = '\0';
    if (sd_bus_message_is_method_error(reply, NULL)) {
        const sd_bus_error *err = sd_bus_message_get_error(reply);
        if (err && err->name)
            snprintf(pair_err_name, sizeof(pair_err_name), "%s", err->name);
        fprintf(stderr, "ble: pair failed: %s\n",
                err && err->message ? err->message : "error");
        pair_ok = 0;
    } else {
        pair_ok = 1;
    }
    pair_done = 1;
    return 0;
}

/* A pairing failure on an unbonded device is most often an asymmetric bond: the
 * device persists its bond across reboots, so if its side still holds an old key
 * the SMP exchange stalls (BlueZ reports AuthenticationTimeout/Failed) and the
 * passkey prompt never fires. The host can't clear the device's bond over BLE -
 * point the user at the device-side `unpair`. */
static void print_stale_bond_hint(void)
{
    bool auth = strstr(pair_err_name, "Authentication") ||
                strstr(pair_err_name, "Timeout") ||
                pair_err_name[0] == '\0';
    if (!auth) return;
    fprintf(stderr,
        "hint: the device may still hold a bond from a previous pairing.\n"
        "      clear it on the device over USB serial, then retry:\n"
        "        fantasi /dev/ttyACM*   ->   unpair\n"
        "      (the host side is already clean once this fails.)\n");
}

/* Register our KeyboardOnly pairing agent and make it the default. Must run
 * BEFORE connecting/pairing: BlueZ derives the adapter IO capability from the
 * default agent at pair time, and a late registration leaves it at the system
 * default (DisplayYesNo → Just Works → an unauthenticated bond the device's
 * MITM-protected characteristics reject). KeyboardOnly forces Passkey Entry. */
static void register_agent(void)
{
    if (sd_bus_add_object_vtable(bus, NULL, AGENT_PATH, "org.bluez.Agent1",
                                 agent_vtable, NULL) < 0)
        return;
    sd_bus_error err = SD_BUS_ERROR_NULL;
    sd_bus_call_method(bus, "org.bluez", "/org/bluez", "org.bluez.AgentManager1",
                       "RegisterAgent", &err, NULL, "os", AGENT_PATH, "KeyboardOnly");
    sd_bus_error_free(&err);
    err = SD_BUS_ERROR_NULL;
    sd_bus_call_method(bus, "org.bluez", "/org/bluez", "org.bluez.AgentManager1",
                       "RequestDefaultAgent", &err, NULL, "o", AGENT_PATH);
    sd_bus_error_free(&err);
}

/* Pair the device if it isn't already bonded. Returns 0 if paired (or already
 * paired), -1 on failure. */
static int ensure_paired(void)
{
    int paired = 0;
    sd_bus_error err = SD_BUS_ERROR_NULL;
    sd_bus_get_property_trivial(bus, "org.bluez", device_path,
                                "org.bluez.Device1", "Paired", &err, 'b', &paired);
    sd_bus_error_free(&err);
    if (paired) return 0;

    printf("ble: pairing...\n");
    /* Pair async so the bus loop can dispatch the agent's RequestPasskey
     * (a blocking call would deadlock - the callback arrives on this bus). */
    pair_done = pair_ok = 0;
    if (sd_bus_call_method_async(bus, NULL, "org.bluez", device_path,
                                 "org.bluez.Device1", "Pair",
                                 on_pair_reply, NULL, "") < 0)
        return -1;
    for (int i = 0; i < 120 && !pair_done; i++) {
        if (sd_bus_process(bus, NULL) > 0) continue;  /* drain ready work */
        sd_bus_wait(bus, 500000);                      /* then block ≤0.5 s */
    }
    return pair_ok ? 0 : -1;
}

/* Connect (if not already), wait for service discovery, ensure the bond,
 * resolve the RX/TX chars and subscribe to notifications. `device_path` must
 * already be set. Shared by the initial open and reconnect; `verbose` gates
 * the progress/error prints (a reconnect retries silently). Returns 0 on
 * success with `connected` set, -1 otherwise (bus left intact for retry). */
static int connect_and_subscribe(bool verbose)
{
    sd_bus_error err = SD_BUS_ERROR_NULL;

    /* Pair before connecting when unbonded: a bare synchronous Connect() on a
     * device with MITM-encrypted chars aborts (le-connection-abort-by-local)
     * because it can't service the agent's passkey prompt mid-call. */
    int paired = 0;
    sd_bus_get_property_trivial(bus, "org.bluez", device_path,
                                "org.bluez.Device1", "Paired", &err, 'b', &paired);
    sd_bus_error_free(&err);
    if (!paired && ensure_paired() < 0) {
        if (verbose) {
            fprintf(stderr, "ble: pairing failed - cannot use encrypted service\n");
            print_stale_bond_hint();
        }
        return -1;
    }

    err = SD_BUS_ERROR_NULL;
    int is_connected = 0;
    sd_bus_get_property_trivial(bus, "org.bluez", device_path,
                                "org.bluez.Device1", "Connected",
                                &err, 'b', &is_connected);
    sd_bus_error_free(&err);

    if (!is_connected) {
        if (verbose) printf("ble: connecting...\n");
        char last_error[256] = "connection attempt failed";
        int last_r = -1;
        for (int attempt = 0; attempt < 5 && !is_connected; attempt++) {
            err = SD_BUS_ERROR_NULL;
            last_r = sd_bus_call_method(bus, "org.bluez", device_path,
                                        "org.bluez.Device1", "Connect",
                                        &err, NULL, "");
            if (last_r >= 0) {
                is_connected = 1;
                sd_bus_error_free(&err);
                break;
            }
            snprintf(last_error, sizeof(last_error), "%s",
                     err.message ? err.message : strerror(-last_r));
            sd_bus_error_free(&err);

            /* Disconnect completion, the peripheral's advertising restart,
             * and BlueZ's cached Device1 state are independent. A Connect can
             * therefore report InProgress/abort while the link completes a
             * moment later, or just before the next advertisement. Poll the
             * property briefly, then retry the connection initiation. */
            for (int wait = 0; wait < 10 && !is_connected; wait++) {
                while (sd_bus_process(bus, NULL) > 0) {}
                usleep(100000);
                sd_bus_error cerr = SD_BUS_ERROR_NULL;
                sd_bus_get_property_trivial(bus, "org.bluez", device_path,
                                            "org.bluez.Device1", "Connected",
                                            &cerr, 'b', &is_connected);
                sd_bus_error_free(&cerr);
            }
        }
        if (!is_connected) {
            if (verbose)
                fprintf(stderr, "ble: connect failed: %s\n", last_error);
            return -1;
        }
    }
    physical_connected = true;

    /* Wait for GATT service discovery (services appear shortly after connect).
     * Stale characteristic objects can remain cached while this is false, so
     * finding their paths is not proof they can accept StartNotify yet. */
    int resolved = 0;
    for (int w = 0; w < 100 && !resolved; w++) {
        sd_bus_error werr = SD_BUS_ERROR_NULL;
        sd_bus_get_property_trivial(bus, "org.bluez", device_path,
                                    "org.bluez.Device1", "ServicesResolved",
                                    &werr, 'b', &resolved);
        sd_bus_error_free(&werr);
        while (sd_bus_process(bus, NULL) > 0) {}
        usleep(100000);
    }
    if (!resolved) {
        if (verbose) fprintf(stderr, "ble: GATT service discovery timed out\n");
        return -1;
    }
    services_resolved = true;

    /* Pair if needed (encrypted chars require a bond). A bonded device - the
     * normal reconnect case - skips this. The KeyboardOnly agent (registered
     * up front in ble_transport_open) drives Passkey Entry. */
    if (ensure_paired() < 0) {
        if (verbose) {
            fprintf(stderr, "ble: pairing failed - cannot use encrypted service\n");
            print_stale_bond_hint();
        }
        return -1;
    }

    /* Resolve RX/TX, accepting either the Flipper (FZ) or Nordic UART (CU)
     * service UUIDs - the host auto-detects whichever the device exposes. */
    if (find_char_by_uuid(RX_UUID_FZ, rx_char_path, sizeof(rx_char_path)) < 0 &&
        find_char_by_uuid(RX_UUID_NUS, rx_char_path, sizeof(rx_char_path)) < 0) {
        if (verbose) fprintf(stderr, "ble: RX characteristic not found\n");
        return -1;
    }
    if (find_char_by_uuid(TX_UUID_FZ, tx_char_path, sizeof(tx_char_path)) < 0 &&
        find_char_by_uuid(TX_UUID_NUS, tx_char_path, sizeof(tx_char_path)) < 0) {
        if (verbose) fprintf(stderr, "ble: TX characteristic not found\n");
        return -1;
    }

    if (verbose) {
        printf("ble: RX=%s\n", rx_char_path);
        printf("ble: TX=%s\n", tx_char_path);
    }

    /* Track the physical Device1 link as a signal, too. The RFID command reads
     * this transport directly and otherwise cannot notice that BlueZ dropped
     * the connection while no write is in progress. */
    if (device_match_slot) {
        sd_bus_slot_unref(device_match_slot);
        device_match_slot = NULL;
    }
    char match[512];
    snprintf(match, sizeof(match),
             "type='signal',sender='org.bluez',path='%s',"
             "interface='org.freedesktop.DBus.Properties',"
             "member='PropertiesChanged'", device_path);
    sd_bus_add_match(bus, &device_match_slot, match,
                     on_device_properties_changed, NULL);

    /* (Re)subscribe to TX notifications. Replace any prior match first so a
     * reconnect can't leave a duplicate handler double-pushing rx bytes. */
    if (tx_match_slot) {
        sd_bus_slot_unref(tx_match_slot);
        tx_match_slot = NULL;
    }
    snprintf(match, sizeof(match),
             "type='signal',sender='org.bluez',path='%s',"
             "interface='org.freedesktop.DBus.Properties',"
             "member='PropertiesChanged'", tx_char_path);
    sd_bus_add_match(bus, &tx_match_slot, match, on_properties_changed, NULL);

    bool subscribed = false;
    char notify_error[256] = "notification subscription failed";
    for (int attempt = 0; attempt < 30 && !subscribed; attempt++) {
        err = SD_BUS_ERROR_NULL;
        int r = sd_bus_call_method(bus, "org.bluez", tx_char_path,
                                   "org.bluez.GattCharacteristic1", "StartNotify",
                                   &err, NULL, "");
        if (r >= 0) {
            subscribed = true;
            sd_bus_error_free(&err);
            break;
        }
        snprintf(notify_error, sizeof(notify_error), "%s",
                 err.message ? err.message : strerror(-r));
        sd_bus_error_free(&err);
        while (sd_bus_process(bus, NULL) > 0) {}
        usleep(100000);
    }
    if (!subscribed) {
        connected = false;
        if (verbose)
            fprintf(stderr, "ble: StartNotify failed: %s\n", notify_error);
        return -1;
    }

    connected = true;
    rx_head = rx_tail = 0;
    memset(&response_rx, 0, sizeof(response_rx));
    response_session = 0;

    /* Query the negotiated ATT MTU from BlueZ */
    sd_bus_error merr = SD_BUS_ERROR_NULL;
    uint16_t mtu = 0;
    if (sd_bus_get_property_trivial(bus, "org.bluez", rx_char_path,
            "org.bluez.GattCharacteristic1", "MTU", &merr, 'q', &mtu) >= 0 && mtu > 3) {
        write_mtu = mtu - 3;
        /* The largest Fantasi RX characteristic is 486 bytes (Flipper); keep
         * each preserved ATT datagram within the firmware's 512-byte receiver. */
        if (write_mtu > 486) write_mtu = 486;
    }
    sd_bus_error_free(&merr);

    if (verbose) printf("ble: connected (MTU %u)\n", write_mtu + 3);
    return 0;
}

/* Advance a dropped-link reconnect without ever waiting inside readline's
 * event hook.  Connect and StartNotify remain owned by persistent async slots;
 * each call here only pumps ready D-Bus work or queues the next phase. */
bool ble_transport_reconnect(void)
{
    if (!bus || !device_path[0]) return false;

    while (sd_bus_process(bus, NULL) > 0) {}
    if (connected) return true;

    if (!physical_connected) {
        if (reconnect_phase == RECONNECT_NOTIFY_PENDING)
            reconnect_call_cancel();
        if (reconnect_phase == RECONNECT_CONNECT_PENDING) {
            if (!reconnect_call.done) return false;
            reconnect_call_cancel();
        }
        (void)reconnect_call_start(RECONNECT_CONNECT_PENDING, device_path,
                                   "org.bluez.Device1", "Connect", 0);
        return false;
    }

    /* A successful Connect reply or Connected=true signal makes any still
     * pending Connect slot redundant.  Service resolution completes through
     * the same Device1 PropertiesChanged match. */
    if (reconnect_phase == RECONNECT_CONNECT_PENDING)
        reconnect_call_cancel();
    if (!services_resolved) return false;

    if (reconnect_phase == RECONNECT_NOTIFY_PENDING) {
        if (!reconnect_call.done) return false;
        bool subscribed = reconnect_call.result >= 0;
        reconnect_call_cancel();
        if (!subscribed) return false;       /* retry on a later hook tick */

        connected = true;
        rx_head = rx_tail = 0;
        memset(&response_rx, 0, sizeof(response_rx));
        response_session = 0;
        return true;
    }

    if (!tx_char_path[0]) return false;
    (void)reconnect_call_start(RECONNECT_NOTIFY_PENDING, tx_char_path,
                               "org.bluez.GattCharacteristic1", "StartNotify",
                               5000000);
    return false;
}

/* ---- Public API ---- */

int ble_transport_open(void)
{
    reconnect_call_cancel();
    physical_connected = false;
    services_resolved = false;
    int r = sd_bus_default_system(&bus);
    if (r < 0) {
        fprintf(stderr, "ble: cannot open system bus: %s\n", strerror(-r));
        return -1;
    }

    /* Register the KeyboardOnly pairing agent up front so BlueZ uses Passkey
     * Entry (not the system default) when we pair below. */
    register_agent();

    if (forced_addr[0]) {
        char a[64];
        snprintf(a, sizeof(a), "%s", forced_addr);
        for (char *p = a; *p; p++) if (*p == ':') *p = '_';
        snprintf(device_path, sizeof(device_path),
                 "/org/bluez/hci0/dev_%s", a);

        /* If BlueZ has never seen this device (fresh boot / no prior scan),
         * Connect has no object to act on. Discover it first. */
        char addr[64] = "";
        if (get_string_prop(device_path, "org.bluez.Device1",
                            "Address", addr, sizeof(addr)) < 0) {
            printf("ble: scanning for %s...\n", forced_addr);
            sd_bus_error serr = SD_BUS_ERROR_NULL;
            sd_bus_call_method(bus, "org.bluez", "/org/bluez/hci0",
                               "org.bluez.Adapter1", "StartDiscovery",
                               &serr, NULL, "");
            sd_bus_error_free(&serr);
            for (int i = 0; i < 30; i++) {   /* ~9 s */
                usleep(300000);
                if (get_string_prop(device_path, "org.bluez.Device1",
                                    "Address", addr, sizeof(addr)) == 0) break;
            }
            serr = SD_BUS_ERROR_NULL;
            sd_bus_call_method(bus, "org.bluez", "/org/bluez/hci0",
                               "org.bluez.Adapter1", "StopDiscovery",
                               &serr, NULL, "");
            sd_bus_error_free(&serr);
        }
    } else {
        int fr = find_fantasi_device();   /* check already-known devices */
        if (fr == -1) {
            /* None cached - scan a full window, THEN decide. Deciding only
             * after discovery (rather than grabbing the first to appear)
             * avoids picking the wrong unit when several are in range. */
            printf("ble: scanning...\n");
            sd_bus_error serr = SD_BUS_ERROR_NULL;
            sd_bus_call_method(bus, "org.bluez", "/org/bluez/hci0",
                               "org.bluez.Adapter1", "StartDiscovery",
                               &serr, NULL, "");
            sd_bus_error_free(&serr);
            for (int i = 0; i < 16; i++) usleep(300000);   /* ~5 s window */
            fr = find_fantasi_device();                    /* decide once */
            serr = SD_BUS_ERROR_NULL;
            sd_bus_call_method(bus, "org.bluez", "/org/bluez/hci0",
                               "org.bluez.Adapter1", "StopDiscovery",
                               &serr, NULL, "");
            sd_bus_error_free(&serr);
            if (fr == -1)
                fprintf(stderr, "ble: no Fantasi device found\n");
        }
        if (fr != 0) {   /* -1 (none) or -2 (ambiguous; list already printed) */
            sd_bus_unref(bus); bus = NULL;
            return -1;
        }
    }

    printf("ble: found %s\n", device_path);

    if (connect_and_subscribe(true) < 0) {
        ble_transport_close();
        return -1;
    }
    return 0;
}

void ble_transport_close(void)
{
    reconnect_call_cancel();
    if (bus) {
        /* StartNotify is owned by this D-Bus sender. Dropping the sender lets
         * BlueZ release exactly this client's notification reference; an
         * explicit StopNotify can race another independent CLI and globally
         * silence the characteristic on BlueZ versions without per-call refs. */
        if (tx_match_slot) {
            sd_bus_slot_unref(tx_match_slot);
            tx_match_slot = NULL;
        }
        if (device_match_slot) {
            sd_bus_slot_unref(device_match_slot);
            device_match_slot = NULL;
        }
        sd_bus_unref(bus);
        bus = NULL;
    }
    connected = false;
    physical_connected = false;
    services_resolved = false;
    response_session = 0;
}

bool ble_transport_connected(void)
{
    /* Process connection changes without a synchronous property query. */
    if (bus)
        while (sd_bus_process(bus, NULL) > 0) {}
    return connected;
}

ssize_t ble_transport_read(void *buf, size_t len)
{
    if (!connected) return -1;

    /* Process any pending D-Bus messages first */
    while (sd_bus_process(bus, NULL) > 0) {}
    if (!connected) return -1;

    size_t avail = rx_available();
    if (avail == 0) return 0;
    if (len > avail) len = avail;

    uint8_t *dst = (uint8_t *)buf;
    for (size_t i = 0; i < len; i++) {
        dst[i] = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
    }
    return (ssize_t)len;
}

static ssize_t ble_write_one(const void *buf, size_t len, bool command)
{
    /* Normally use acknowledged ATT Write Requests. Write Commands have no end-to-end
     * flow control across independent D-Bus clients: each short-lived process
     * can successfully enqueue its write after the previous sender exits even
     * though BlueZ/controller credits have not recovered, silently losing a
     * later session OPEN. The narrowly exposed command path is only for
     * absolute-offset file chunks: their correlated protobuf ACK detects a
     * lost fragment and the caller retries the idempotent write. */
    /* A second process can receive InProgress for the duration of another
     * process's multi-fragment request.  Five seconds covers a full maximum
     * protobuf message at the minimum ATT MTU without making a dead link hang
     * indefinitely. */
    for (int attempt = 0; attempt < 1000; attempt++) {
        sd_bus_error err = SD_BUS_ERROR_NULL;
        sd_bus_message *msg = NULL;

        int r = sd_bus_message_new_method_call(bus, &msg, "org.bluez",
                                               rx_char_path,
                                               "org.bluez.GattCharacteristic1",
                                               "WriteValue");
        if (r < 0) return -1;

        sd_bus_message_append_array(msg, 'y', buf, len);

        sd_bus_message_open_container(msg, 'a', "{sv}");
        sd_bus_message_open_container(msg, 'e', "sv");
        sd_bus_message_append(msg, "s", "type");
        sd_bus_message_open_container(msg, 'v', "s");
        sd_bus_message_append(msg, "s", command ? "command" : "request");
        sd_bus_message_close_container(msg);
        sd_bus_message_close_container(msg);
        sd_bus_message_close_container(msg);

        r = sd_bus_call(bus, msg, 5000000, &err, NULL);
        sd_bus_message_unref(msg);

        if (r >= 0) {
            sd_bus_error_free(&err);
            return (ssize_t)len;
        }

        const char *ename = err.name ? err.name : "";
        const char *emsg = err.message ? err.message : "";
        bool disconnected = strstr(ename, "NotConnected") ||
                            strstr(ename, "DoesNotExist") ||
                            strstr(ename, "UnknownObject") ||
                            strstr(ename, "Disconnected") ||
                            strstr(emsg, "Not connected") ||
                            strstr(emsg, "not connected") ||
                            strstr(emsg, "Disconnected") ||
                            strstr(emsg, "disconnected");
        if (disconnected) {
            connected = false;
            fprintf(stderr, "ble: write failed: %s\n",
                    err.message ? err.message : strerror(-r));
            sd_bus_error_free(&err);
            return -1;
        }
        sd_bus_error_free(&err);
        usleep(5000);   /* let the other D-Bus writer complete, then retry */
    }
    fprintf(stderr, "ble: write failed: transient errors did not clear\n");
    return -1;
}

ssize_t ble_transport_write(const void *buf, size_t len)
{
    if (!connected || !rx_char_path[0]) return -1;

    const uint8_t *p = (const uint8_t *)buf;
    size_t remaining = len;
    size_t packet_mtu = write_mtu;
    if (packet_mtu > FANTASI_BLE_MUX_PACKET_MAX)
        packet_mtu = FANTASI_BLE_MUX_PACKET_MAX;
    while (remaining > 0) {
        size_t chunk = (remaining > packet_mtu) ? packet_mtu : remaining;
        if (ble_write_one(p, chunk, false) < 0) return -1;
        p += chunk;
        remaining -= chunk;
    }
    return (ssize_t)len;
}

static void put_u16le(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
}

static void put_u32le(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
    out[2] = (uint8_t)(value >> 16);
    out[3] = (uint8_t)(value >> 24);
}

static ssize_t ble_write_session(uint32_t session, const void *message,
                                 size_t len, bool command)
{
    size_t packet_mtu = write_mtu;
    if (packet_mtu > FANTASI_BLE_MUX_PACKET_MAX)
        packet_mtu = FANTASI_BLE_MUX_PACKET_MAX;
    if (!connected || !rx_char_path[0] || !session || !message ||
        len == 0 || len > UINT16_MAX ||
        packet_mtu <= FANTASI_BLE_MUX_HEADER_SIZE)
        return -1;

    /* Host memory is not firmware memory, so one MTU-sized scratch allocation
     * is preferable to burdening every device session with a receive buffer. */
    uint8_t *packet = malloc(packet_mtu);
    if (!packet) return -1;
    packet[0] = FANTASI_BLE_MUX_MAGIC_0;
    packet[1] = FANTASI_BLE_MUX_MAGIC_1;
    packet[2] = FANTASI_BLE_MUX_MAGIC_2;
    packet[3] = FANTASI_BLE_MUX_MAGIC_3;
    put_u32le(packet + 4, session);
    put_u16le(packet + 8, (uint16_t)len);

    const uint8_t *src = message;
    size_t payload_cap = packet_mtu - FANTASI_BLE_MUX_HEADER_SIZE;
    size_t offset = 0;
    while (offset < len) {
        size_t chunk = len - offset;
        if (chunk > payload_cap) chunk = payload_cap;
        put_u16le(packet + 10, (uint16_t)offset);
        memcpy(packet + FANTASI_BLE_MUX_HEADER_SIZE, src + offset, chunk);
        if (ble_write_one(packet, FANTASI_BLE_MUX_HEADER_SIZE + chunk,
                          command) < 0) {
            free(packet);
            return -1;
        }
        offset += chunk;
    }
    free(packet);
    return (ssize_t)len;
}

ssize_t ble_transport_write_session(uint32_t session,
                                    const void *message, size_t len)
{
    return ble_write_session(session, message, len, false);
}

ssize_t ble_transport_write_session_command(uint32_t session,
                                            const void *message, size_t len)
{
    return ble_write_session(session, message, len, true);
}

int ble_transport_fd(void)
{
    return bus ? sd_bus_get_fd(bus) : -1;
}

void ble_transport_process(void)
{
    if (bus)
        while (sd_bus_process(bus, NULL) > 0) {}
}
