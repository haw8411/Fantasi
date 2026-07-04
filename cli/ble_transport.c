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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

void ble_transport_set_addr(const char *addr)
{
    snprintf(forced_addr, sizeof(forced_addr), "%s", addr);
}
static char rx_char_path[256];
static char tx_char_path[256];
static bool connected;
static uint16_t write_mtu = 20;
static sd_bus_slot *tx_match_slot;  /* TX-notify match; replaced on reconnect */

/* Ring buffer for incoming indications */
/* Large enough to absorb a full download burst: BlueZ can deliver many queued
 * notifications in a single sd_bus_process() pump, and the device streams the
 * `cat` download back-to-back with no per-chunk ACK. Regression risk: if this
 * ring is too small, a burst can overrun it between drains; rx_push() drops
 * bytes on overflow, which desyncs the 2-byte length-prefixed protobuf stream
 * and truncates the file. Size it to hold a whole in-flight transfer. */
#define RX_BUF_SIZE 65536
static uint8_t rx_buf[RX_BUF_SIZE];
static size_t  rx_head, rx_tail;

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
                rx_push((const uint8_t *)data, len);
            sd_bus_message_exit_container(msg);
        } else {
            sd_bus_message_skip(msg, "v");
        }

        sd_bus_message_exit_container(msg);
    }

    return 0;
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
        err = SD_BUS_ERROR_NULL;
        int r = sd_bus_call_method(bus, "org.bluez", device_path,
                                   "org.bluez.Device1", "Connect", &err, NULL, "");
        if (r < 0) {
            if (verbose)
                fprintf(stderr, "ble: connect failed: %s\n",
                        err.message ? err.message : strerror(-r));
            sd_bus_error_free(&err);
            return -1;
        }
        sd_bus_error_free(&err);
    }

    /* Wait for GATT service discovery (services appear shortly after connect). */
    for (int w = 0; w < 60; w++) {
        int resolved = 0;
        sd_bus_error werr = SD_BUS_ERROR_NULL;
        sd_bus_get_property_trivial(bus, "org.bluez", device_path,
                                    "org.bluez.Device1", "ServicesResolved",
                                    &werr, 'b', &resolved);
        sd_bus_error_free(&werr);
        if (resolved) break;
        usleep(100000);
    }

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

    /* (Re)subscribe to TX notifications. Replace any prior match first so a
     * reconnect can't leave a duplicate handler double-pushing rx bytes. */
    if (tx_match_slot) {
        sd_bus_slot_unref(tx_match_slot);
        tx_match_slot = NULL;
    }
    char match[512];
    snprintf(match, sizeof(match),
             "type='signal',sender='org.bluez',path='%s',"
             "interface='org.freedesktop.DBus.Properties',"
             "member='PropertiesChanged'", tx_char_path);
    sd_bus_add_match(bus, &tx_match_slot, match, on_properties_changed, NULL);

    err = SD_BUS_ERROR_NULL;
    int r = sd_bus_call_method(bus, "org.bluez", tx_char_path,
                               "org.bluez.GattCharacteristic1", "StartNotify",
                               &err, NULL, "");
    if (r < 0 && verbose)
        fprintf(stderr, "ble: StartNotify failed: %s\n",
                err.message ? err.message : strerror(-r));
    sd_bus_error_free(&err);

    connected = true;
    rx_head = rx_tail = 0;

    /* Query the negotiated ATT MTU from BlueZ */
    sd_bus_error merr = SD_BUS_ERROR_NULL;
    uint16_t mtu = 0;
    if (sd_bus_get_property_trivial(bus, "org.bluez", rx_char_path,
            "org.bluez.GattCharacteristic1", "MTU", &merr, 'q', &mtu) >= 0 && mtu > 3)
        write_mtu = mtu - 3;
    sd_bus_error_free(&merr);

    if (verbose) printf("ble: connected (MTU %u)\n", write_mtu + 3);
    return 0;
}

/* Re-establish a dropped link (e.g. after the device reboots). Quiet: returns
 * false if the device isn't reachable yet so the caller can retry later. */
bool ble_transport_reconnect(void)
{
    if (!bus || !device_path[0]) return false;
    return connect_and_subscribe(false) == 0;
}

/* ---- Public API ---- */

int ble_transport_open(void)
{
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
        sd_bus_unref(bus); bus = NULL;
        return -1;
    }
    return 0;
}

void ble_transport_close(void)
{
    if (bus) {
        sd_bus_error err = SD_BUS_ERROR_NULL;
        if (tx_char_path[0]) {
            sd_bus_call_method(bus, "org.bluez", tx_char_path,
                               "org.bluez.GattCharacteristic1", "StopNotify",
                               &err, NULL, "");
            sd_bus_error_free(&err);
        }
        sd_bus_unref(bus);
        bus = NULL;
    }
    connected = false;
}

bool ble_transport_connected(void)
{
    /* Reflect the live link state: BlueZ clears Device1.Connected when the
     * device drops (e.g. a reboot), which our cached flag wouldn't catch. */
    if (connected && bus && device_path[0]) {
        int c = 0;
        sd_bus_error err = SD_BUS_ERROR_NULL;
        if (sd_bus_get_property_trivial(bus, "org.bluez", device_path,
                "org.bluez.Device1", "Connected", &err, 'b', &c) >= 0 && !c)
            connected = false;
        sd_bus_error_free(&err);
    }
    return connected;
}

ssize_t ble_transport_read(void *buf, size_t len)
{
    if (!connected) return -1;

    /* Process any pending D-Bus messages first */
    while (sd_bus_process(bus, NULL) > 0) {}

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

static ssize_t ble_write_one(const void *buf, size_t len)
{
    /* Write-without-response ("command"): BlueZ doesn't wait for an ATT
     * response, so writes pipeline within a connection event instead of
     * one round-trip each. It has no ATT-level flow control, though, so the
     * pipelined upload's burst can outrun the controller's TX buffer and
     * BlueZ returns a transient error - retry with a short backoff (which
     * paces us to the link rate) rather than failing the transfer. A real
     * disconnect is not retryable and fails fast. */
    for (int attempt = 0; attempt < 80; attempt++) {
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
        sd_bus_message_append(msg, "s", "command");
        sd_bus_message_close_container(msg);
        sd_bus_message_close_container(msg);
        sd_bus_message_close_container(msg);

        r = sd_bus_call(bus, msg, 5000000, &err, NULL);
        sd_bus_message_unref(msg);

        if (r >= 0) {
            sd_bus_error_free(&err);
            return (ssize_t)len;
        }

        bool disconnected = err.name &&
            (strstr(err.name, "NotConnected") ||
             strstr(err.name, "DoesNotExist") ||
             strstr(err.name, "Disconnected"));
        if (disconnected) {
            fprintf(stderr, "ble: write failed: %s\n",
                    err.message ? err.message : strerror(-r));
            sd_bus_error_free(&err);
            return -1;
        }
        sd_bus_error_free(&err);
        usleep(2000);   /* let the TX buffer drain, then retry */
    }
    fprintf(stderr, "ble: write failed: transient errors did not clear\n");
    return -1;
}

ssize_t ble_transport_write(const void *buf, size_t len)
{
    if (!connected || !rx_char_path[0]) return -1;

    const uint8_t *p = (const uint8_t *)buf;
    size_t remaining = len;
    while (remaining > 0) {
        size_t chunk = (remaining > write_mtu) ? write_mtu : remaining;
        if (ble_write_one(p, chunk) < 0) return -1;
        p += chunk;
        remaining -= chunk;
    }
    return (ssize_t)len;
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
