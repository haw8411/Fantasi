/* `rfid` - host side of the on-demand RFID app.
 *
 * Streams the driver into /ramfs, launches it as an async app session, then
 * services it live over the protobuf channel: prints the app's output, forwards
 * the terminal's keystrokes (and ^C) to it, and answers each `module_request` by
 * streaming that feature module into /ramfs just in time. On exit it deletes the
 * driver + any modules it streamed, so nothing is pre-staged and nothing
 * persists. Requires the protobuf transport (WebUSB/BLE) - the async protocol
 * needs the device's RX loop free, which the raw serial CLI can't provide. Run
 * the CLI with --usb or --ble. */
#include "cli_internal.h"
#include "theme.h"
#include "mfc_crypto.h"
#include "mfc_mfkey.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <elf.h>

static void cmd_rfid(const char *arg)
{
    (void)arg;
    fprintf(stderr, "rfid: needs the protobuf transport - start the CLI with --usb or --ble\n");
}

#ifdef HAS_PROTO
#include <pb_encode.h>
#include <pb_decode.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <poll.h>
#include <signal.h>
#include <readline/readline.h>
#include <readline/history.h>
#include "ble_transport.h"
#ifdef HAS_USB_VENDOR
#include "usb_transport.h"
#endif

/* Built app ELFs live under build/apps as <name>.<arch>.elf. The arch is chosen
 * per connected device: Cortex-M (CU/Flipper) load the .cm4 ELFs, the Proxmark3's
 * ARM7TDMI loads the .arm7 ELFs. Set FANTASI_APP_DIR to load them from elsewhere -
 * e.g. an app repo built out of tree (~/FantasiApps/build) - without copying ELFs in. */
#define APP_DIR    "build/apps"
#define DRIVER_RAM "/ramfs/rfid"
#define MFC_CFG_RAM "/ramfs/mfc_cfg"
#define MFC_EMU_RAM "/ramfs/mfc_emu.bin"
#define MFC_REQ_RAM "/ramfs/.mfcrreq"
#define MFC_KEY_RAM "/ramfs/.mfckey"

/* App-ELF directory: $FANTASI_APP_DIR if set (and non-empty), else the build/apps default. */
static const char *app_dir(void)
{
    const char *d = getenv("FANTASI_APP_DIR");
    return (d && *d) ? d : APP_DIR;
}

static const struct { const char *name, *ram; } RFID_MODS[] = {
    { "hf",  "/ramfs/rfid_hf" },
    { "lf",  "/ramfs/rfid_lf" },
    { "raw", "/ramfs/rfid_raw" },
    { "sniff", "/ramfs/rfid_sniff" },
    { "t5577", "/ramfs/rfid_t5577" },
    { "t5577_dump", "/ramfs/rfid_t5577d" },   /* whole-tag dump for bare `read t5577` (one round-trip) */
    { "mfc_collect", "/ramfs/rfid_mfcc" },   /* MIFARE Classic nonce collector */
    { "mfc_read",    "/ramfs/rfid_mfcr" },   /* MIFARE Classic dictionary reader */
    { "mfc_block",   "/ramfs/rfid_mfcb" },   /* MIFARE Classic single-block reader */
    { "mfc_emu",     "/ramfs/rfid_mfce" },   /* MIFARE Classic tag emulation */
};
#define NMODS ((int)(sizeof RFID_MODS / sizeof RFID_MODS[0]))

static const char *s_arch = "cm4";      /* set from the device: cm4 or arm7 */
static uint32_t rq_id = 5000;
static volatile sig_atomic_t g_rfid_sigstop;
static void rfid_on_sigstop(int s) { (void)s; g_rfid_sigstop = 1; }

/* "<app_dir>/<name>.<arch>.elf" (app_dir = $FANTASI_APP_DIR or build/apps). */
static const char *elf_path(const char *name)
{
    static char buf[256];
    snprintf(buf, sizeof buf, "%s/%s.%s.elf", app_dir(), name, s_arch);
    return buf;
}

static ssize_t tp_write(const void *b, size_t n)
{
#ifdef HAS_USB_VENDOR
    if (use_usb) return usb_transport_write(b, n);
#endif
    return ble_transport_write(b, n);
}
static ssize_t tp_read(void *b, size_t n)
{
#ifdef HAS_USB_VENDOR
    if (use_usb) return usb_transport_read(b, n);
#endif
    /* Poll before pumping D-Bus; mirrors main.c's proto_recv. */
    int bfd = ble_transport_fd();
    if (bfd >= 0) { struct pollfd p = { .fd = bfd, .events = POLLIN }; poll(&p, 1, 20); }
    ble_transport_process();
    return ble_transport_read(b, n);
}

/* WebUSB mux READs address bytes within one response mailbox.  Once this
 * private RFID parser has decoded a complete frame it must release that
 * mailbox and rewind the host offset, exactly as main.c's usb_recv_proto()
 * does.  BLE is a byte stream and needs no corresponding operation. */
static void tp_frame_consumed(void)
{
#ifdef HAS_USB_VENDOR
    if (use_usb) usb_transport_frame_consumed();
#endif
}

static int send_req(CliRequest *req)
{
    if (proto_session_id) {
        req->has_session = true;
        req->session = proto_session_id;
    }
    static uint8_t buf[2 + CliRequest_size];
    pb_ostream_t s = pb_ostream_from_buffer(buf + 2, sizeof buf - 2);
    if (!pb_encode(&s, CliRequest_fields, req)) return -1;
    uint16_t len = (uint16_t)s.bytes_written;
    buf[0] = len & 0xFF; buf[1] = len >> 8;
#ifdef HAS_USB_VENDOR
    if (!use_usb && proto_session_id)
#else
    if (proto_session_id)
#endif
        return (req->which_payload == CliRequest_file_write_tag
                ? ble_transport_write_session_command(proto_session_id,
                                                      buf + 2, len)
                : ble_transport_write_session(proto_session_id, buf + 2, len)) == len
             ? 0 : -1;
    return tp_write(buf, 2 + len) == (ssize_t)(2 + len) ? 0 : -1;
}

static int rfid_readline_heartbeat(void)
{
    static time_t last;
    time_t now = time(NULL);
    if (proto_session_id && now != (time_t)-1 && now - last >= 10) {
        last = now;
        CliRequest ping = CliRequest_init_zero;
        ping.id = ++rq_id;
        ping.which_payload = CliRequest_session_ping_tag;
        ping.payload.session_ping = true;
        (void)send_req(&ping);
    }
    return 0;
}

/* Frame accumulator: pull one CliResponse if a whole frame is buffered, else read
 * more (a short blocking read). Returns 1 (got resp), 0 (none yet), -1 (error). */
static uint8_t rx_acc[8192];
static size_t  rx_len;

static bool response_sane(const CliResponse *response)
{
    if (!response->id || (response->has_session && !response->session))
        return false;
    switch (response->which_payload) {
    case CliResponse_output_tag:
    case CliResponse_file_data_tag:
    case CliResponse_dir_entry_tag:
    case CliResponse_error_tag:
    case CliResponse_module_request_tag:
        return true;
    default:
        return false;
    }
}

static int recv_resp(CliResponse *resp)
{
    /* The app can spend minutes streaming or waiting for a module response;
     * those are still live host sessions even though readline is not active. */
    rfid_readline_heartbeat();
    uint8_t chunk[1024];
    ssize_t n = tp_read(chunk, sizeof chunk);
    if (n < 0) return -1;
    /* Do not turn an empty app mailbox into an EP0 denial-of-service.  The
     * PM3 USB task outranks the app/session tasks; issuing the next control
     * READ immediately can keep that task continuously runnable, preventing
     * the very producer we are waiting for from running.  One host-side
     * millisecond leaves bulk transfers untouched and gives every target a
     * natural scheduling gap while an RFID command is genuinely idle. */
    if (n == 0) usleep(1000);
    if (n > 0) {
        size_t copy = (size_t)n;
        if (copy > sizeof(rx_acc) - rx_len) {
            /* Retain only a maximum-frame suffix; the scanner below can recover
             * at the next complete response boundary. */
            size_t keep = 2u + CliResponse_size;
            if (rx_len > keep) {
                memmove(rx_acc, rx_acc + rx_len - keep, keep);
                rx_len = keep;
            }
            if (copy > sizeof(rx_acc) - rx_len) copy = sizeof(rx_acc) - rx_len;
        }
        memcpy(rx_acc + rx_len, chunk, copy);
        rx_len += copy;
    }

    for (size_t pos = 0; pos + 2 <= rx_len; pos++) {
        uint16_t ml = (uint16_t)rx_acc[pos] | ((uint16_t)rx_acc[pos + 1] << 8);
        if (!ml || ml > CliResponse_size || pos + 2u + ml > rx_len) continue;
        CliResponse candidate = CliResponse_init_zero;
        pb_istream_t st = pb_istream_from_buffer(rx_acc + pos + 2, ml);
        if (!pb_decode(&st, CliResponse_fields, &candidate) ||
            !response_sane(&candidate))
            continue;
        if (proto_session_id) {
            if (!candidate.has_session || candidate.session != proto_session_id)
                continue;
        } else if (candidate.has_session) {
            continue;
        }
        size_t consumed = pos + 2u + ml;
        memmove(rx_acc, rx_acc + consumed, rx_len - consumed);
        rx_len -= consumed;
        tp_frame_consumed();
        *resp = candidate;
        return 1;
    }
    size_t keep = 2u + CliResponse_size;
    if (rx_len > keep) {
        memmove(rx_acc, rx_acc + rx_len - keep, keep);
        rx_len = keep;
    }
    return 0;
}

static int publish_file(const char *staged, const char *final)
{
    CliRequest req = CliRequest_init_zero;
    if (strlen(staged) >= sizeof req.payload.file_rename.src ||
        strlen(final) >= sizeof req.payload.file_rename.dst) return -1;
    req.id = ++rq_id;
    req.which_payload = CliRequest_file_rename_tag;
    memcpy(req.payload.file_rename.src, staged, strlen(staged) + 1);
    memcpy(req.payload.file_rename.dst, final, strlen(final) + 1);
    if (send_req(&req) < 0) return -1;
    for (int spin = 0; spin < 3000; spin++) {
        CliResponse r;
        int rc = recv_resp(&r);
        if (rc < 0) return -1;
        if (rc == 1 && r.id == req.id)
            return r.which_payload == CliResponse_error_tag ? -1 : 0;
        if (rc == 0) usleep(1000);
    }
    return -1;
}

/* An in-place module uses its RAMFS ELF buffer as text/rodata/data and then reuses
 * the now-dead ELF metadata tail for NOBITS sections. Return the smallest buffer
 * which also contains the end of every allocated section. Most ELFs already have
 * enough metadata tail; padding only the shortfall avoids a second module-sized
 * allocation in the PM3 loader (raw.cm4 currently needs just 180 extra bytes). */
static uint32_t inplace_elf_size(FILE *f, uint32_t file_size)
{
    Elf32_Ehdr eh;
    if (fseek(f, 0, SEEK_SET) != 0 || fread(&eh, 1, sizeof eh, f) != sizeof eh ||
        memcmp(eh.e_ident, ELFMAG, SELFMAG) != 0 ||
        eh.e_ident[EI_CLASS] != ELFCLASS32 || eh.e_ident[EI_DATA] != ELFDATA2LSB ||
        eh.e_type != ET_REL || eh.e_machine != EM_ARM ||
        eh.e_shentsize != sizeof(Elf32_Shdr) || !eh.e_shnum ||
        (uint64_t)eh.e_shoff + (uint64_t)eh.e_shnum * sizeof(Elf32_Shdr) > file_size) {
        fseek(f, 0, SEEK_SET);
        return file_size;
    }

    uint32_t need = file_size;
    for (unsigned i = 0; i < eh.e_shnum; i++) {
        Elf32_Shdr sh;
        if (fseek(f, (long)eh.e_shoff + (long)i * sizeof sh, SEEK_SET) != 0 ||
            fread(&sh, 1, sizeof sh, f) != sizeof sh) {
            need = file_size;
            break;
        }
        if (sh.sh_flags & SHF_ALLOC) {
            uint64_t end = (uint64_t)sh.sh_offset + sh.sh_size;
            if (end > UINT32_MAX) { need = file_size; break; }
            if (end > need) need = (uint32_t)end;
        }
    }
    fseek(f, 0, SEEK_SET);
    return need;
}

/* Stream local -> /ramfs path. Fires FileWriteChunks; when drain_acks, waits for
 * their acks here (used before the session loop is running to absorb them). */
static int upload_ram_ex(const char *ram, const char *local, int drain_acks, int inplace)
{
    (void)drain_acks;   /* every chunk is correlated and drained here */
    char staged[64];
    int sl = snprintf(staged, sizeof staged, "%s.part", ram);
    if (sl < 0 || (size_t)sl >= sizeof staged) return -1;
    FILE *f = fopen(local, "rb");
    if (!f) { fprintf(stderr, "\nrfid: cannot open %s\n", local); return -1; }
    fseek(f, 0, SEEK_END); long sz = ftell(f); fseek(f, 0, SEEK_SET);
    if (sz < 0 || (uint64_t)sz > UINT32_MAX) { fclose(f); return -1; }
    uint32_t total = inplace ? inplace_elf_size(f, (uint32_t)sz) : (uint32_t)sz;
    /* BLE's ATT writes and notifications otherwise alternate one full
     * round-trip per 480-byte chunk. Keep exactly one additional request in
     * flight so its fragments travel while the prior ACK is pending. That is
     * at most one extra transient encoded request (609-byte protocol maximum,
     * plus a small job/allocator header), still below 1 KB, with no idle-session
     * RAM cost. WebUSB - including PM3's seven-byte EP0 CHUNK path - retains
     * the conservative window of one.
     *
     * Absolute offsets make a window rewind idempotent. A later in-window ACK
     * before the oldest means the oldest write/ACK was lost: discard the
     * logical window and resend from the last contiguous ACK. Unrelated stale
     * cleanup/input responses are ignored, not mistaken for upload progress. */
#define RFID_UPLOAD_CHUNK  480u
#define RFID_UPLOAD_WINDOW 2
#define RFID_UPLOAD_RETRIES 30
    int window = RFID_UPLOAD_WINDOW;
#ifdef HAS_USB_VENDOR
    if (use_usb) window = 1;
#endif
    uint32_t chunks = total ? (total + RFID_UPLOAD_CHUNK - 1) / RFID_UPLOAD_CHUNK : 1;
    uint32_t acked = 0, sent = 0;
    uint32_t inflight[RFID_UPLOAD_WINDOW];
    int inf_head = 0, inf_count = 0, retries = 0, error = 0;

    while (acked < chunks && !error) {
        while (sent < chunks && inf_count < window) {
            uint32_t off = sent * RFID_UPLOAD_CHUNK;
            uint32_t left = total > off ? total - off : 0;
            size_t n = left < RFID_UPLOAD_CHUNK ? left : RFID_UPLOAD_CHUNK;
            CliRequest req = CliRequest_init_zero;
            req.id = ++rq_id;
            req.which_payload = CliRequest_file_write_tag;
            FileWriteChunk *fw = &req.payload.file_write;
            memcpy(fw->path, staged, strlen(staged) + 1);
            fw->offset = off;
            memset(fw->data.bytes, 0, n);
            if (off < (uint32_t)sz) {
                size_t from_file = (uint32_t)sz - off;
                if (from_file > n) from_file = n;
                if (fseek(f, (long)off, SEEK_SET) != 0 ||
                    fread(fw->data.bytes, 1, from_file, f) != from_file) {
                    error = 1;
                    break;
                }
            }
            fw->data.size = (pb_size_t)n;
            fw->last = sent + 1 == chunks;
            fw->has_total = true;
            fw->total = total;

            if (send_req(&req) < 0) {
                if (++retries > RFID_UPLOAD_RETRIES) error = 1;
                usleep(1000);
                break;
            }
            inflight[(inf_head + inf_count) % RFID_UPLOAD_WINDOW] = req.id;
            inf_count++;
            sent++;
        }
        if (error) break;
        if (inf_count == 0) continue;

        int advanced = 0, rewind = 0;
        for (int spin = 0; spin < 100 && !advanced && !rewind; spin++) {
            CliResponse r;
            int rc = recv_resp(&r);
            if (rc < 0) { error = 1; break; }
            if (rc == 0) { usleep(1000); continue; }

            int pos = -1;
            for (int i = 0; i < inf_count; i++)
                if (r.id == inflight[(inf_head + i) % RFID_UPLOAD_WINDOW]) {
                    pos = i;
                    break;
                }
            if (pos < 0) continue;                         /* unrelated stale response */
            if (r.which_payload == CliResponse_error_tag) { error = 1; break; }
            if (pos > 0) { rewind = 1; break; }            /* hole before a later ACK */

            inf_head = (inf_head + 1) % RFID_UPLOAD_WINDOW;
            inf_count--;
            acked++;
            retries = 0;
            advanced = 1;
        }
        if (error) break;
        if (!advanced) rewind = 1;                         /* bounded ACK timeout */
        if (rewind) {
            if (++retries > RFID_UPLOAD_RETRIES) { error = 1; break; }
            sent = acked;
            inf_head = inf_count = 0;
        }
    }
    fclose(f);
    if (error) return -1;
    return publish_file(staged, ram);
}

static int upload_ram(const char *ram, const char *local, int drain_acks)
{
    return upload_ram_ex(ram, local, drain_acks, 0);
}

static void serve_module(const char *name)
{
    /* Bitstream request "fpga/<res>": stream the compressed bitstream into the
     * device's persistent /fpga/<res>.bit.z, where the driver caches it (built by
     * platforms/proxmark3/Makefile via tools/fpga_lzss.py), then flag completion. */
    if (strncmp(name, "fpga/", 5) == 0) {
        char res[24];
        if (strlen(name + 5) >= sizeof res) { fprintf(stderr, "\nrfid: bad bitstream '%s'\n", name); return; }
        strcpy(res, name + 5);
        char local[64], dev[48];
        snprintf(local, sizeof local, "build/fpga/%s.bit.z", res);
        snprintf(dev, sizeof dev, "/fpga/%s.bit.z", res);
        if (upload_ram(dev, local, 0) < 0)
            fprintf(stderr, "\nrfid: host has no bitstream '%s'\n", res);
        return;
    }
    for (int i = 0; i < NMODS; i++)
        if (strcmp(RFID_MODS[i].name, name) == 0) {
            upload_ram_ex(RFID_MODS[i].ram, elf_path(name), 0, 1);
            return;
        }
    fprintf(stderr, "\nrfid: host has no module '%s'\n", name);
}

static void delete_ram(const char *ram)
{
    CliRequest req = CliRequest_init_zero;
    req.id = ++rq_id; req.which_payload = CliRequest_file_delete_tag;
    strncpy(req.payload.file_delete.path, ram, sizeof req.payload.file_delete.path - 1);
    /* Consume each terminal response before queuing another cleanup request.
     * Missing files remain a best-effort terminal result. */
    if (send_req(&req) < 0) return;
    for (int spin = 0; spin < 200; spin++) {
        CliResponse r;
        int rc = recv_resp(&r);
        if (rc < 0) return;
        if (rc == 1 && r.id == req.id) return;
        if (rc == 0) usleep(1000);
    }
}

static void send_input(const uint8_t *d, size_t n)
{
    while (n) {
        CliRequest req = CliRequest_init_zero;
        size_t chunk = n;
        if (chunk > sizeof req.payload.app_input.bytes)
            chunk = sizeof req.payload.app_input.bytes;
        req.id = ++rq_id;
        req.which_payload = CliRequest_app_input_tag;
        memcpy(req.payload.app_input.bytes, d, chunk);
        req.payload.app_input.size = (pb_size_t)chunk;
        if (send_req(&req) < 0) return;
        d += chunk;
        n -= chunk;
    }
}

/* Pick the ELF arch for the connected device: run `version` and look for the
 * device id. The Proxmark3 ("device pm3") is ARM7TDMI and loads .arm7 ELFs;
 * everything else is Cortex-M and keeps the .cm4 default. */
static void detect_arch(void)
{
    CliRequest req = CliRequest_init_zero;
    req.id = ++rq_id; req.which_payload = CliRequest_command_tag;
    strncpy(req.payload.command, "version", sizeof req.payload.command - 1);
    /* Drain any stale frames the transport buffered before correlating. */
    uint8_t junk[512];
    for (int i = 0; i < 50; i++) { if (tp_read(junk, sizeof junk) <= 0) break; }
    /* A discarded WebUSB frame leaves the stateless transport offset at its
     * end.  Release it idempotently and rewind before the version response. */
    tp_frame_consumed();
    rx_len = 0;

    /* A preceding RFID run queues best-effort delete replies. If that bounded
     * device queue is still full, consume one stale response and retry until
     * the architecture probe itself has definitely entered the session. */
    bool submitted = false;
    for (int attempt = 0; attempt < 64 && !submitted; attempt++) {
        if (send_req(&req) == 0) {
            submitted = true;
            break;
        }
        CliResponse stale;
        int rc = recv_resp(&stale);
        if (rc < 0) break;
        if (rc == 0) usleep(1000);
    }
    if (!submitted) return;

    char out[512]; size_t olen = 0;
    for (int spin = 0; spin < 5000; spin++) {
        CliResponse r;
        int rc = recv_resp(&r);
        if (rc < 0) break;
        if (rc == 0) { usleep(1000); continue; }
        if (r.id != req.id) continue;       /* stale cleanup/heartbeat reply */
        if (r.which_payload == CliResponse_output_tag) {
            size_t l = strlen(r.payload.output);
            if (olen + l < sizeof out) { memcpy(out + olen, r.payload.output, l); olen += l; }
        }
        if (!r.has_next) break;
    }
    out[olen] = 0;
    if (strstr(out, "device pm3")) s_arch = "arm7";
}

/* ---- sniff-trace beautifier -----------------------------------------------
 * The device streams one compact, serial-safe text line per frame:
 *     <R|C> <start_us> <end_us> <hex>[!] <hex>[!] ...      (! = parity mismatch)
 * plus a leading "L<couple> f<fld> o<overrun> v<valid> m<backlog>" health line.
 * The wire stays plain text (a dumb serial terminal still shows a usable trace);
 * the rich host does the heavy lifting the device shouldn't - a per-frame CRC_A
 * check and pm3-style protocol annotations - and lays it out as a table. Non-
 * frame app output (prompts, search results, messages) passes straight through. */

static uint16_t crc16_a(const uint8_t *d, int n)   /* ISO14443-A CRC_A: poly 0x8408, init 0x6363 */
{
    uint32_t crc = 0x6363;
    for (int i = 0; i < n; i++) {
        uint8_t b = (uint8_t)(d[i] ^ (crc & 0xFF));
        b = (uint8_t)(b ^ (b << 4));
        crc = (crc >> 8) ^ ((uint32_t)b << 8) ^ ((uint32_t)b << 3) ^ ((uint32_t)b >> 4);
    }
    return (uint16_t)(crc & 0xFFFF);
}

/* ---- host-side, modular-in-C protocol annotators -------------------------------
 * The device module decodes the RF layer (NFC-A/B/F/V, LF, ...) into frames; the
 * host names them. Annotators are stateful: a MIFARE AUTH handshake, a cascade, or
 * an mfkey64 key recovery spans several frames, so the renderer zero-inits `ctx`
 * (ctx_size bytes) at each trace start via begin(), then calls line() per frame. A
 * protocol annotator (ann_mfc, ann_ul, ...) chains to its RF-layer base (ann_nfca)
 * first, then refines. The active one is chosen per protocol from the registry below;
 * today only the NFC-A base is wired, so the output is unchanged. */
typedef struct annotator {
    size_t ctx_size;                    /* per-trace state the host allocates + zeroes */
    void (*begin)(void *ctx);           /* reset state at trace start (NULL = stateless) */
    void (*line)(void *ctx, char dir, const uint8_t *b, int n, char *out, size_t cap);
} annotator_t;

/* Shared NFC-A base namer (port of PM3 client applyIso14443a, cmdhflist.c): reader command
 * and card response naming. Every NFC-A protocol annotator chains to this as its fallback.
 * `n` includes the 2-byte CRC on standard frames (as it appears in the trace). Stateless. */
static void nfca_base(char dir, const uint8_t *b, int n, char *out, size_t cap)
{
    out[0] = 0;
    if (dir == 'R') {
        switch (b[0]) {
            case 0x52: snprintf(out, cap, "WUPA"); return;
            case 0x26: snprintf(out, cap, "REQA"); return;
            case 0x93: case 0x95: case 0x97: {
                int cl = (b[0] - 0x93) / 2 + 1;
                if      (n >= 2 && b[1] == 0x70)                   snprintf(out, cap, "SELECT cl%d", cl);
                else if (n >= 2 && (b[1] == 0x20 || b[1] == 0x50)) snprintf(out, cap, "ANTICOLL cl%d", cl);
                else                                               snprintf(out, cap, "cascade cl%d", cl);
                return;
            }
            case 0x50: snprintf(out, cap, "HALT"); return;
            case 0xE0: snprintf(out, cap, "RATS"); return;
            case 0x30: snprintf(out, cap, "READ blk %d",   n >= 2 ? b[1] : 0); return;
            case 0x3A: snprintf(out, cap, "FAST_READ %d-%d", n >= 3 ? b[1] : 0, n >= 3 ? b[2] : 0); return;
            case 0xA0: snprintf(out, cap, "WRITE blk %d",  n >= 2 ? b[1] : 0); return;   /* 14a / ULC compat */
            case 0xA2: snprintf(out, cap, "WRITE page %d", n >= 2 ? b[1] : 0); return;   /* UL write */
            case 0xC0: snprintf(out, cap, "DEC blk %d",    n >= 2 ? b[1] : 0); return;
            case 0xC1: snprintf(out, cap, "INC blk %d",    n >= 2 ? b[1] : 0); return;
            case 0xC2: snprintf(out, cap, "RESTORE blk %d",n >= 2 ? b[1] : 0); return;
            case 0xB0: snprintf(out, cap, "TRANSFER blk %d",n >= 2 ? b[1] : 0); return;
            case 0x39: snprintf(out, cap, "READ_CNT %d",   n >= 2 ? b[1] : 0); return;
            case 0xA5: snprintf(out, cap, "INCR_CNT %d",   n >= 2 ? b[1] : 0); return;
            case 0x3C: snprintf(out, cap, "READ_SIG"); return;
            case 0x3E: snprintf(out, cap, "CHECK_TEARING %d", n >= 2 ? b[1] : 0); return;
            case 0x4B: snprintf(out, cap, "VCSL"); return;
            case 0x60:
                if (n > 3) snprintf(out, cap, "AUTH-A blk %d", n >= 2 ? b[1] : 0);
                else       snprintf(out, cap, "GET_VERSION");
                return;
            case 0x61: snprintf(out, cap, "AUTH-B blk %d", n >= 2 ? b[1] : 0); return;
            default:
                if ((b[0] & 0xF0) == 0xD0) snprintf(out, cap, "PPS");
                return;
        }
    } else {                                       /* card -> reader */
        if      (n == 2) snprintf(out, cap, "ATQA");
        else if (n == 5) snprintf(out, cap, "UID+BCC");
        else if (n == 3) snprintf(out, cap, "SAK");
        else if (n >  0) snprintf(out, cap, "response");
    }
}

static void ann_reset(void *ctx) { *(int *)ctx = 0; }   /* begin() for the stateful annotators */

/* nfca: the generic RF layer - just the base namer. */
static void ann_nfca_line(void *ctx, char dir, const uint8_t *b, int n, char *out, size_t cap)
{ (void)ctx; nfca_base(dir, b, n, out, cap); }
static const annotator_t ann_nfca = { 0, NULL, ann_nfca_line };

/* mfc: MIFARE Classic. Stateful crypto-1 AUTH tracking (nt -> nr ar -> at), the exchange
 * mfkey/mfkey64 recovers a key from. State in *ctx: 0 idle, 1 expect nt, 2 expect nr/ar, 3 expect at. */
static void ann_mfc_line(void *ctx, char dir, const uint8_t *b, int n, char *out, size_t cap)
{
    int *st = ctx, R = (dir == 'R');
    if (R && n >= 1 && (b[0] == 0x52 || b[0] == 0x26)) *st = 0;      /* WUPA/REQA restarts a card */
    switch (*st) {
        case 1: if (!R && n == 4) { snprintf(out, cap, "AUTH: nt");           *st = 2; return; } *st = 0; break;
        case 2: if ( R && n == 8) { snprintf(out, cap, "AUTH: nr ar (enc)");  *st = 3; return; } *st = 0; break;
        case 3: if (!R && n == 4) { snprintf(out, cap, "AUTH: at (enc)");     *st = 0; return; } *st = 0; break;
    }
    if (R && (b[0] == 0x60 || b[0] == 0x61) && n > 3) {
        snprintf(out, cap, "AUTH key%c(blk %d)", b[0] == 0x60 ? 'A' : 'B', n >= 2 ? b[1] : 0);
        *st = 1; return;
    }
    nfca_base(dir, b, n, out, cap);
}
static const annotator_t ann_mfc = { sizeof(int), ann_reset, ann_mfc_line };

/* ul: MIFARE Ultralight (EV1). Base + password auth. */
static void ann_ul_line(void *ctx, char dir, const uint8_t *b, int n, char *out, size_t cap)
{
    (void)ctx;
    if (dir == 'R' && b[0] == 0x1B) { snprintf(out, cap, "PWD_AUTH"); return; }
    if (dir == 'C' && n == 4)       { snprintf(out, cap, "PACK / data"); return; }
    nfca_base(dir, b, n, out, cap);
}
static const annotator_t ann_ul = { 0, NULL, ann_ul_line };

/* ulc: Ultralight C. Base + the 3DES mutual-auth handshake (1A -> ek(RndB) -> AF ek(RndA||RndB') -> ek(RndA')). */
static void ann_ulc_line(void *ctx, char dir, const uint8_t *b, int n, char *out, size_t cap)
{
    int *st = ctx, R = (dir == 'R');
    if (R && b[0] == 0x1A)                 { snprintf(out, cap, "3DES AUTH");             *st = 1; return; }
    if (*st == 1 && !R && b[0] == 0xAF)    { snprintf(out, cap, "AUTH: ek(RndB)");        *st = 2; return; }
    if (*st == 2 &&  R && b[0] == 0xAF)    { snprintf(out, cap, "AUTH: ek(RndA||RndB')"); *st = 3; return; }
    if (*st == 3 && !R)                    { snprintf(out, cap, "AUTH: ek(RndA') done");  *st = 0; return; }
    nfca_base(dir, b, n, out, cap);
}
static const annotator_t ann_ulc = { sizeof(int), ann_reset, ann_ulc_line };

/* ulaes: Ultralight AES. Same handshake shape as UL-C but AES; the 0x1A carries a key slot. */
static void ann_ulaes_line(void *ctx, char dir, const uint8_t *b, int n, char *out, size_t cap)
{
    int *st = ctx, R = (dir == 'R');
    if (R && b[0] == 0x1A)              { snprintf(out, cap, "AES AUTH (key %d)", n >= 2 ? b[1] : 0); *st = 1; return; }
    if (*st == 1 && !R && b[0] == 0xAF) { snprintf(out, cap, "AUTH: E(RndB)");        *st = 2; return; }
    if (*st == 2 &&  R && b[0] == 0xAF) { snprintf(out, cap, "AUTH: E(RndA||RndB')"); *st = 3; return; }
    if (*st == 3 && !R)                 { snprintf(out, cap, "AUTH: E(RndA') done");  *st = 0; return; }
    nfca_base(dir, b, n, out, cap);
}
static const annotator_t ann_ulaes = { sizeof(int), ann_reset, ann_ulaes_line };

/* mfp: MIFARE Plus (SL1/SL3). Names the MFP-distinctive verbs; base handles the rest. */
static void ann_mfp_line(void *ctx, char dir, const uint8_t *b, int n, char *out, size_t cap)
{
    (void)ctx;
    if (dir == 'R') switch (b[0]) {
        case 0x70: snprintf(out, cap, "AUTH FIRST");     return;
        case 0x72: snprintf(out, cap, "AUTH CONTINUE");  return;
        case 0x76: snprintf(out, cap, "FOLLOWING AUTH"); return;
        case 0x78: snprintf(out, cap, "RESET AUTH");     return;
        case 0xA4: snprintf(out, cap, "ISO SELECT");     return;
        case 0x44: snprintf(out, cap, "SET CONFIG SL1"); return;
        case 0xAF: snprintf(out, cap, "NEXT FRAME");     return;
    }
    nfca_base(dir, b, n, out, cap);
}
static const annotator_t ann_mfp = { 0, NULL, ann_mfp_line };

/* mfdes: DESFire. Names native command bytes (unwrapping the ISO7816 90/00 envelope); base for the 14a layer. */
static void ann_mfdes_line(void *ctx, char dir, const uint8_t *b, int n, char *out, size_t cap)
{
    (void)ctx;
    if (dir == 'R' && n >= 1) {
        uint8_t c = (n >= 2 && b[0] == 0x90) ? b[1] : b[0];   /* unwrap ISO7816 native wrapper */
        switch (c) {
            case 0x5A: snprintf(out, cap, "SELECT APP");       return;
            case 0x60: snprintf(out, cap, "GET VERSION");      return;
            case 0x6A: snprintf(out, cap, "GET APP IDS");      return;
            case 0x6F: snprintf(out, cap, "GET FILE IDS");     return;
            case 0x0A: snprintf(out, cap, "AUTH (native)");    return;
            case 0x1A: snprintf(out, cap, "AUTH (ISO)");       return;
            case 0xAA: snprintf(out, cap, "AUTH (AES)");       return;
            case 0x71: snprintf(out, cap, "AUTH EV2 first");   return;
            case 0xBD: snprintf(out, cap, "READ DATA");        return;
            case 0x3D: snprintf(out, cap, "WRITE DATA");       return;
            case 0xCA: snprintf(out, cap, "CREATE APP");       return;
            case 0xC4: snprintf(out, cap, "CHANGE KEY");       return;
            case 0x45: snprintf(out, cap, "GET KEY SETTINGS"); return;
            case 0x51: snprintf(out, cap, "GET UID");          return;
            case 0x6E: snprintf(out, cap, "GET FREE MEM");     return;
            case 0xAF: snprintf(out, cap, "NEXT FRAME");       return;
        }
    }
    nfca_base(dir, b, n, out, cap);
}
static const annotator_t ann_mfdes = { 0, NULL, ann_mfdes_line };

/* The trace renderer's active annotator + its per-trace scratch, reset at each trace start.
 * Set per protocol when a sniff/raw runs (grammar migration); defaults to the NFC-A base. */
static const annotator_t *g_ann = &ann_nfca;
static unsigned char g_ann_ctx[128];    /* must be >= every annotator's ctx_size */
static void ann_begin(void) { if (g_ann->begin) g_ann->begin(g_ann_ctx); }

static int hexnib(char c)
{ return (c>='0'&&c<='9')?c-'0':(c>='A'&&c<='F')?c-'A'+10:(c>='a'&&c<='f')?c-'a'+10:-1; }

static int fixed_hex(const char *s, size_t n)
{
    if (strlen(s) != n) return 0;
    for (size_t i = 0; i < n; i++) if (hexnib(s[i]) < 0) return 0;
    return 1;
}

static int s_tbl_hdr;         /* 0 = (re)print the column header before the next frame row */
static int s_raw;             /* 1 = raw-trace mode (no timing columns), set by a `T` header */
static int g_rfid_ready;      /* set when the device app printed its "rfid> " prompt = ready for a line */
static unsigned s_samp_ns = 375;   /* sample period (ns); set from the L-header's p<ns> field */

/* Frame timestamps arrive as raw sample counts (the device never divides); convert to us here. */
static unsigned long samp_us(unsigned long s) { return (unsigned long)((unsigned long long)s * s_samp_ns / 1000); }

/* One frame -> one table row. Two sources share this: the passive SNIFF (with sample-derived us
 * timestamps) and the RAW command's trace (no timing - a synchronous exchange). s_raw selects the
 * layout; CRC_A, the UID/BCC special case, and protocol naming are common to both. */
/* ---- collect mfc sniff: recover keys from sniffed reader<->card auths via mfkey64 ------------------- */
static int g_mfc_sniff;                          /* 1 while `collect mfc sniff` is capturing */
static uint32_t g_ms_uid, g_ms_nt, g_ms_nr, g_ms_ar;
static int g_ms_state, g_ms_blk, g_ms_kt, g_ms_have_uid; /* auth parser state + selected card */
static uint64_t g_ms_keys[256]; static int g_ms_nkeys;

static uint32_t ms_be32(const uint8_t *b) { return (uint32_t)b[0] << 24 | (uint32_t)b[1] << 16 | (uint32_t)b[2] << 8 | b[3]; }

/* Feed one sniffed frame to the auth parser: R 60/61 (auth req) -> C nt -> R nr_enc|ar_enc -> C at_enc.
 * On a complete auth, run mfkey64 and remember the recovered key (deduped). */
static void mfc_sniff_frame(char dir, const uint8_t *b, int n)
{
    if (dir == 'R' && n == 1 && (b[0] == 0x52 || b[0] == 0x26)) {
        g_ms_state = 0; g_ms_have_uid = 0;                       /* a newly activated card needs a new CUID */
    }
    if (dir == 'C' && n == 5 && (b[0] ^ b[1] ^ b[2] ^ b[3]) == b[4]) {
        g_ms_state = 0; g_ms_have_uid = b[0] != 0x88;            /* UID+BCC; 0x88 is an incomplete cascade */
        if (g_ms_have_uid) g_ms_uid = ms_be32(b);
    }
    if (dir == 'R' && n == 9 && (b[0] == 0x93 || b[0] == 0x95 || b[0] == 0x97) && b[1] == 0x70 &&
        (b[2] ^ b[3] ^ b[4] ^ b[5]) == b[6]) {
        uint16_t crc = crc16_a(b, 7);
        if ((uint8_t)crc == b[7] && (uint8_t)(crc >> 8) == b[8]) {
            g_ms_state = 0; g_ms_have_uid = b[2] != 0x88;        /* full SELECT also carries the CUID */
            if (g_ms_have_uid) g_ms_uid = ms_be32(b + 2);
        }
    }
    switch (g_ms_state) {
        case 0: if (dir == 'R' && n == 4 && (b[0] == 0x60 || b[0] == 0x61)) { g_ms_blk = b[1]; g_ms_kt = b[0] & 1; g_ms_state = 1; } break;
        case 1: if (dir == 'C' && n == 4) { g_ms_nt = ms_be32(b); g_ms_state = 2; } else g_ms_state = 0; break;
        case 2: if (dir == 'R' && n == 8) { g_ms_nr = ms_be32(b); g_ms_ar = ms_be32(b + 4); g_ms_state = 3; } else g_ms_state = 0; break;
        case 3:
            if (g_ms_have_uid && dir == 'C' && n == 4) {
                uint64_t key;
                if (mc_mfkey64(g_ms_uid, g_ms_nt, g_ms_nr, g_ms_ar, ms_be32(b), &key) == 0) {
                    int dup = 0; for (int i = 0; i < g_ms_nkeys; i++) if (g_ms_keys[i] == key) dup = 1;
                    if (!dup && g_ms_nkeys < 256) {
                        g_ms_keys[g_ms_nkeys++] = key;
                        printf("\033[32m  + recovered key %012llX (sec %d key %c)\033[0m\r\n",
                               (unsigned long long)key, g_ms_blk / 4, g_ms_kt ? 'B' : 'A');
                    }
                }
            }
            g_ms_state = 0;
            break;
    }
}

/* On sniff stop: append the recovered keys to /nfc/mfc.dict as uppercase ASCII hex (one key per line - the
 * standard dictionary format the read module consumes). Download the existing dict, add the new (deduped)
 * keys, upload it back (the edit-command protobuf pattern), so keys accumulate across sniffs. Prints use
 * \r\n - the terminal is in raw mode during the sniff stream. */
static void mfc_sniff_finalize(void)
{
    g_mfc_sniff = 0; g_ms_state = 0; g_ms_have_uid = 0;
    if (g_ms_nkeys == 0) { printf("collect: no keys recovered (no complete auth sniffed)\r\n"); return; }

    char tmp[] = "/tmp/fantasi-mfcdict-XXXXXX";
    int fd = mkstemp(tmp);
    if (fd < 0) { printf("collect: mkstemp failed\r\n"); return; }
    FILE *dl = fdopen(fd, "w+"); if (!dl) { close(fd); unlink(tmp); printf("collect: temp-file error\r\n"); return; }
    if (proto_download("/nfc/mfc.dict", dl) < 0) { rewind(dl); if (ftruncate(fd, 0) != 0) { /* fresh if no existing dict */ } }

    uint64_t existing[512]; int nex = 0;                   /* dedup against keys already in the dict */
    rewind(dl); char ln[64];
    while (nex < 512 && fgets(ln, sizeof ln, dl)) { uint64_t k = strtoull(ln, NULL, 16); if (k) existing[nex++] = k; }
    fseek(dl, 0, SEEK_END);
    int added = 0;
    for (int i = 0; i < g_ms_nkeys; i++) {
        int dup = 0; for (int j = 0; j < nex; j++) if (existing[j] == g_ms_keys[i]) dup = 1;
        if (!dup) { fprintf(dl, "%012llX\n", (unsigned long long)g_ms_keys[i]); added++; }
    }
    fflush(dl);
    if (proto_upload(tmp, "/nfc/mfc.dict") < 0) printf("collect: failed to write /nfc/mfc.dict\r\n");
    else printf("collect: recovered %d key(s), %d new -> /nfc/mfc.dict\r\n", g_ms_nkeys, added);
    fclose(dl); unlink(tmp);
}

static void sniff_frame(const char *line)
{
    static const char *D = "-------------------------------------";
    char dir = line[0];
    char *p = (char *)line + 2;
    unsigned long start = 0, end = 0;
    if (!s_raw) { start = samp_us(strtoul(p, &p, 10)); end = samp_us(strtoul(p, &p, 10)); }
    uint8_t b[32] = {0}; int n = 0; uint32_t par = 0;
    while (*p && n < 32) {
        while (*p == ' ') p++;
        int hi = hexnib(p[0]), lo = hi < 0 ? -1 : hexnib(p[1]);
        if (lo < 0) break;
        b[n] = (uint8_t)((hi << 4) | lo); p += 2;
        if (*p == '!') { par |= (1u << n); p++; }
        n++;
    }
    if (!s_tbl_hdr) {
        if (s_raw) {
            printf("  %-3s | %-3s | %-15s | %s\r\n", "Src", "CRC", "Annotation", "Data");
            printf("  %.3s-+-%.3s-+-%.15s-+-%.25s\r\n", D, D, D, D);
        } else {
            printf("  %8s | %8s | %-3s | %-3s | %-15s | %s\r\n",
                   "Start", "End", "Src", "CRC", "Annotation", "Data (us; ! = parity err)");
            printf("  %.8s-+-%.8s-+-%.3s-+-%.3s-+-%.15s-+-%.25s\r\n", D, D, D, D, D, D);
        }
        s_tbl_hdr = 1;
    }
    const char *crc = "   ";
    if (dir == 'C' && n == 5)                     /* a UID+BCC reply is checked by a BCC (XOR), not a CRC */
        crc = ((b[0]^b[1]^b[2]^b[3]) == b[4]) ? "ok " : "bad";
    else if (n >= 3) {                            /* CRC-bearing frames self-identify: a random 2-byte
                                                   * suffix almost never matches, so "ok" == has valid CRC */
        uint16_t c = crc16_a(b, n - 2);
        crc = ((c & 0xFF) == b[n-2] && (c >> 8) == b[n-1]) ? "ok " : "bad";
    }
    char ann[24]; g_ann->line(g_ann_ctx, dir, b, n, ann, sizeof ann);
    const char *col = (dir == 'R') ? "\033[36m" : "\033[33m";   /* reader cyan, card yellow */
    const char *bad = (crc[0] == 'b') ? "\033[31m" : "";
    if (s_raw)
        printf("%s  %-3s | %s%-3s%s | %-15s | ", col, dir == 'R' ? "Rdr" : "Tag", bad, crc, bad[0] ? col : "", ann);
    else
        printf("%s  %8lu | %8lu | %-3s | %s%-3s%s | %-15s | ",
               col, start, end, dir == 'R' ? "Rdr" : "Tag", bad, crc, bad[0] ? col : "", ann);
    for (int i = 0; i < n; i++) {
        if (par & (1u << i)) printf("\033[31m%02X!\033[0m%s", b[i], col);
        else                 printf("%02X ", b[i]);
    }
    printf("\033[0m\r\n");
    if (g_mfc_sniff) mfc_sniff_frame(dir, b, n);            /* `collect mfc sniff`: recover keys via mfkey64 */
}

static void sniff_header(const char *line)   /* "L<couple> [f<fld>] v<valid> [o<ov>] [m<bl>] p<ns>". couple 0/1/2
                                              * = measured strong/fair/weak; >=3 = unknown (device can't sense the
                                              * field - e.g. the Chameleon has no HF envelope tap, so it reports 9). */
{
    int couple = line[1] - '0', fld = 0, ov = 0, vf = 0; unsigned long bl = 0;
    for (const char *p = line; *p; p++)
        if      (*p == 'f') fld = atoi(p+1);
        else if (*p == 'o') ov = atoi(p+1);
        else if (*p == 'v') vf = atoi(p+1);
        else if (*p == 'm') bl = strtoul(p+1, NULL, 10);
        else if (*p == 'p' && p[1] >= '0' && p[1] <= '9') s_samp_ns = (unsigned)atoi(p+1);
    const char *over = ov ? "  \033[31m(OVERRUN - trace incomplete)\033[0m" : "";
    if (couple >= 3)                              /* field strength not measurable on this device - don't fake it */
        printf("\r\n\033[1m── capture ──\033[0m coupling \033[90munknown\033[0m, %d frame%s%s\r\n",
               vf, vf == 1 ? "" : "s", over);
    else {
        const char *cc = couple <= 0 ? "\033[32mstrong" : couple == 1 ? "\033[33mfair" : "\033[31mweak";
        printf("\r\n\033[1m── capture ──\033[0m field %d, coupling %s\033[0m, %d frame%s, peak backlog %lu%s\r\n",
               fld, cc, vf, vf == 1 ? "" : "s", bl, over);
    }
    s_raw = 0; s_tbl_hdr = 0;                     /* sniff layout; re-print the column header for this capture */
    ann_begin();                                  /* new trace -> reset the active annotator's state */
}

/* A `T` line heads a RAW-command trace (the driver's do_raw/do_trace). Switch the frame table to the
 * no-timing raw layout and print a light banner. */
static void raw_header(void)
{
    printf("\r\n\033[1m── raw trace ──\033[0m\r\n");
    s_raw = 1; s_tbl_hdr = 0;
    ann_begin();                                  /* new trace -> reset the active annotator's state */
}

/* Line-buffer the app's output stream and route each complete line: frame -> table row, L<..> -> capture
 * banner, everything else -> passthrough. A partial NON-frame remainder (the "rfid> " prompt has no
 * newline) is flushed immediately; a partial frame/header line stays buffered for its continuation. */
static void sniff_emit(const char *s)
{
    static char lb[1024]; static int lbl = 0;
    while (*s) {
        while (*s && *s != '\n') { if (lbl < (int)sizeof lb - 1) lb[lbl++] = *s; s++; }
        if (*s != '\n') break;                    /* chunk ended mid-line */
        s++;
        if (lbl > 0 && lb[lbl-1] == '\r') lbl--;
        lb[lbl] = 0;
        char c = lb[0];
        if ((c == 'R' || c == 'C') && lb[1] == ' ' && hexnib(lb[2]) >= 0) sniff_frame(lb);   /* hex may start A-F */
        else if (c == 'L' && lb[1] >= '0' && lb[1] <= '9') sniff_header(lb);
        else if (c == 'T' && lb[1] == '\0') raw_header();
        else { fputs(lb, stdout); fputs("\r\n", stdout); }   /* raw tty: newline needs the carriage return */
        lbl = 0;
    }
    if (lbl > 0) {
        lb[lbl] = 0;
        if (!strcmp(lb, "rfid> "))                         /* the app's prompt: a ready cue, not shown - */
            { if (g_mfc_sniff) mfc_sniff_finalize();       /* sniff ended -> recover keys to /nfc/mfc.dict */
              g_rfid_ready = 1; lbl = 0; }                 /* readline draws its own prompt host-side */
        else if (lb[0] != 'R' && lb[0] != 'C' && lb[0] != 'L' && lb[0] != 'T')
            { fputs(lb, stdout); lbl = 0; }
    }
    fflush(stdout);
}

/* Readline TAB completion for the rfid> prompt (installed only while an interactive rfid session runs):
 * the command word completes against the rfid commands, and field/trace sub-commands complete too. The
 * generator (called by rl_completion_matches) walks whichever candidate list rfid_completion selects. */
/* ============================ the rfid registry ============================
 * One source of truth the host derives help, `list`, completion and (later) protocol
 * -> module/annotator resolution from - so adding a protocol is one row, not four
 * hand-kept lists. Two axes: RF category (the device sniff/raw module, one per NFC
 * tech) x protocol (the host annotator). Rendering + help + list all stay off-device. */

/* mirror of FANTASI_RFID_CAP_* (apps/app_rfid.h) - the host doesn't include the device header */
#define CAP_HF_READ (1u<<0)
#define CAP_HF_EMU  (1u<<1)
#define CAP_LF_READ (1u<<2)
#define CAP_LF_EMU  (1u<<3)

/* `list` is a 3-level tree: band (carrier) > sub-category (RF tech within HF, or the
 * carrier frequency within LF/UHF) > protocol. Both band and sub-category
 * tokens are filterable (`list hf`, `list nfca`, `list 125`). */
enum { BAND_LF, BAND_HF, BAND_UHF };       /* enum value == index into BAND[]; this is the list order */
static const struct { const char *tok, *name; } BAND[] = {
    { "lf",  "LF" },                       /* frequency lives in the sub-category (125-500 kHz) */
    { "hf",  "HF \xC2\xB7 13.56 MHz" },    /* single carrier -> freq in the band header */
    { "uhf", "UHF" },
};
#define NBAND ((int)(sizeof BAND / sizeof BAND[0]))

/* Sub-categories. HF splits by NFC tech (each is a distinct RF layer / sniffer); LF and
 * UHF are single clustered ranges (LF is software-tunable 125-500 kHz but one frontend). */
enum { SC_NFCA, SC_NFCB, SC_NFCF, SC_NFCV, SC_LF, SC_UHF };
static const struct { const char *tok, *name; int band; } SUB[] = {
    { "nfca", "NFC A \xC2\xB7 ISO14443-A", BAND_HF },
    { "nfcb", "NFC B \xC2\xB7 ISO14443-B", BAND_HF },
    { "nfcf", "NFC F \xC2\xB7 FeliCa",     BAND_HF },
    { "nfcv", "NFC V \xC2\xB7 ISO15693",   BAND_HF },
    { "lf",   "125-500 kHz",  BAND_LF },
    { "uhf",  "860-960 MHz",  BAND_UHF },
};
#define NSUB ((int)(sizeof SUB / sizeof SUB[0]))

enum {
    OP_READ = 1, OP_SNIFF = 2, OP_RAW = 4, OP_EMU = 8,
    OP_WRITE = 16, OP_COLLECT = 32,
};
static const struct { uint32_t bit; const char *name; } OPS[] = {
    { OP_READ, "read" }, { OP_SNIFF, "sniff" }, { OP_RAW, "raw" },
    { OP_WRITE, "write" }, { OP_EMU, "emulate" }, { OP_COLLECT, "collect" },
};
#define NOPS ((int)(sizeof OPS / sizeof OPS[0]))

/* Protocols: what the user types after a verb. `label` names the specific protocol (the
 * sub-category header already carries the RF layer, so the generic base protocol is just
 * "generic"). `ann` is the host annotator (protocols like mfc chain to their
 * base). The RF-layer bases and the NFC-A protocol rows (mfc, ul, ...) are all wired. */
static const struct {
    const char *token, *label;
    int sub; uint32_t ops, cap;
    const annotator_t *ann;
} RFID_REG[] = {
    { "nfca",  "generic",             SC_NFCA,  OP_SNIFF | OP_RAW,           CAP_HF_READ, &ann_nfca  },
    { "mfc",   "MIFARE Classic",      SC_NFCA,  OP_SNIFF | OP_RAW | OP_READ | OP_EMU | OP_COLLECT,
                                                                           CAP_HF_READ, &ann_mfc   },
    { "mfp",   "MIFARE Plus",         SC_NFCA,  OP_SNIFF | OP_RAW,           CAP_HF_READ, &ann_mfp   },
    { "ul",    "Ultralight",          SC_NFCA,  OP_SNIFF | OP_RAW | OP_READ, CAP_HF_READ, &ann_ul    },
    { "ulc",   "Ultralight C",        SC_NFCA,  OP_SNIFF | OP_RAW | OP_READ, CAP_HF_READ, &ann_ulc   },
    { "ulaes", "Ultralight AES",      SC_NFCA,  OP_SNIFF | OP_RAW,           CAP_HF_READ, &ann_ulaes },
    { "mfdes", "DESFire",             SC_NFCA,  OP_SNIFF | OP_RAW,           CAP_HF_READ, &ann_mfdes },
    { "t5577", "T5577",               SC_LF,    OP_READ | OP_WRITE,          CAP_LF_READ, &ann_nfca  },
};
#define NREG ((int)(sizeof RFID_REG / sizeof RFID_REG[0]))

/* Per-command option legends + examples, surfaced by `help <command>`. Kept beside the verb table so the
 * two stay in sync. A NULL-terminated list means "none". Order-free flags mirror the parse_rw/raw/collect
 * handlers above - keep them accurate. */
typedef struct { const char *flag, *desc; } rfid_hopt;

static const rfid_hopt HO_search[]  = { { "[band]", "scan just one band (hf or lf); omit to scan every band" }, { NULL, NULL } };
static const char *const EX_search[] = { "search", "search hf", "search lf", NULL };

static const rfid_hopt HO_read[] = {
    { "<protocol>", "tag type to read (see `list`)" },
    { "-b <block>", "read just this block; omit to dump every block" },
    { "-k <key>",   "authentication key or password" },
    { "-s [file]",  "save the dump as JSON (auto-named if no file given)" },
    { NULL, NULL } };
static const char *const EX_read[] = {
    "read mfc                 dump a MIFARE Classic",
    "read mfc -b 4 -k FFFFFFFFFFFF",
    "read t5577 -b 4          read block 4 of an LF T5577",
    "read mfc -s dump.json    dump and save as JSON",
    NULL };

static const rfid_hopt HO_write[] = {
    { "<protocol>", "tag type to write (see `list`)" },
    { "-b <block>", "block number to write" },
    { "-d <hex>",   "data to write" },
    { "-k <key>",   "key / password" },
    { NULL, NULL } };
static const char *const EX_write[] = { "write t5577 -b 4 -d 1A2B3C4D", NULL };

static const rfid_hopt HO_emulate[] = {
    { "<dump.json>", "a dump saved by `read -s`" }, { NULL, NULL } };
static const char *const EX_emulate[] = { "emulate mycard.json", NULL };

static const rfid_hopt HO_sniff[] = {
    { "<protocol>", "tag type to decode the capture as (see `list`)" }, { NULL, NULL } };
static const char *const EX_sniff[] = { "sniff mfc     watch a reader<->card auth (any key stops)", NULL };

static const rfid_hopt HO_trace[] = { { "clear", "clear the trace instead of showing it" }, { NULL, NULL } };
static const char *const EX_trace[] = { "trace", "trace clear", NULL };

static const rfid_hopt HO_collect[] = {
    { "<protocol>", "tag type to collect from (see `list`)" },
    { "card",       "collect nonces from a presented card (Hardnested)" },
    { "sniff",      "recover keys from a live auth (mfkey64)" },
    { "reader",     "impersonate a card to a reader (not implemented yet)" },
    { "-u <UID>",   "target UID (4- or 7-byte hex)" },
    { "-k <key>",   "seed a known key (6 hex bytes)" },
    { NULL, NULL } };
static const char *const EX_collect[] = { "collect mfc card", "collect mfc sniff", NULL };

static const rfid_hopt HO_raw[] = {
    { "<protocol>", "tag type (see `list`)" },
    { "-c",         "append a CRC to the frame" },
    { "-k",         "keep the field on afterwards (for follow-ups)" },
    { "-s",         "select the tag first (anticollision)" },
    { "<hex>",      "frame bytes to send" },
    { NULL, NULL } };
static const char *const EX_raw[] = { "raw mfc -s -c 3000    select a card, then read block 0 with CRC", NULL };

static const rfid_hopt HO_field[] = {
    { "on",     "turn the reader carrier on and leave it on" },
    { "off",    "turn the carrier off" },
    { "status", "report whether the carrier is on" },
    { NULL, NULL } };
static const char *const EX_field[] = { "field on", "field status", NULL };

static const rfid_hopt HO_list[] = {
    { "[band|sub]", "restrict to a band (hf/lf) or sub-category" }, { NULL, NULL } };
static const char *const EX_list[] = { "list", "list hf", NULL };

static const rfid_hopt HO_help[] = {
    { "[command]", "detailed help for one command; omit for this list" }, { NULL, NULL } };
static const char *const EX_help[] = { "help", "help read", NULL };

/* The verb vocabulary - drives help + top-level completion. `local` verbs are host-side and coloured
 * yellow: help/list are host-rendered, read/write/collect/emulate are host abstractions over device
 * primitives. The rest dispatch straight to the device app. `section` groups them in `help`. */
enum { SEC_OPS, SEC_ANALYZE, SEC_GENERAL };
static const char *const RFID_SECTIONS[] = { "operations", "sniff & analyze", "general" };
#define NSECT ((int)(sizeof RFID_SECTIONS / sizeof RFID_SECTIONS[0]))

typedef struct {
    const char *name, *args, *help;
    bool local;
    int  section;
    const rfid_hopt *opts;
    const char *const *examples;
} rfid_verb_t;

static const rfid_verb_t RFID_VERBS[] = {
    { "search",  "[band]",                                         "scan for nearby tags (bare = every band)",     false, SEC_OPS,     HO_search,  EX_search  },
    { "read",    "<protocol> [-b <block>] [-k <key>] [-s [file]]", "read one block, or dump a whole tag",          true,  SEC_OPS,     HO_read,    EX_read    },
    { "write",   "<protocol> -b <block> -d <hex> [-k <key>]",      "write a block to a tag",                       true,  SEC_OPS,     HO_write,   EX_write   },
    { "emulate", "<dump.json>",                                    "emulate a card from a saved dump",             true,  SEC_OPS,     HO_emulate, EX_emulate },
    { "sniff",   "<protocol>",                                     "passively watch a reader<->card exchange",     false, SEC_ANALYZE, HO_sniff,   EX_sniff   },
    { "trace",   "[clear]",                                        "show or clear the captured frame trace",       false, SEC_ANALYZE, HO_trace,   EX_trace   },
    { "collect", "<protocol> <card|reader|sniff> [-u <UID>] [-k <key>]", "capture key material (nonces, sniffed keys)", true, SEC_ANALYZE, HO_collect, EX_collect },
    { "raw",     "<protocol> [-c] [-k] [-s] <hex>",                "send a raw frame to a protocol",               false, SEC_ANALYZE, HO_raw,     EX_raw     },
    { "field",   "on|off|status",                                  "turn the reader carrier on/off, or report it", false, SEC_GENERAL, HO_field,   EX_field   },
    { "list",    "[band|sub]",                                     "list supported protocols",                     true,  SEC_GENERAL, HO_list,    EX_list    },
    { "help",    "[command]",                                      "show this list, or details for one command",   true,  SEC_GENERAL, HO_help,    EX_help    },
    { "exit",    "",                                               "leave the rfid app",                           false, SEC_GENERAL, NULL,       NULL       },
};
#define NVERBS ((int)(sizeof RFID_VERBS / sizeof RFID_VERBS[0]))

/* A cyan "--------  ----- title -----" rule, matching `list`'s sub-category headers. */
static void rfid_section_hdr(const char *title)
{
    int width = 44, tl = (int)strlen(title), dashes = width - tl - 2;
    if (dashes < 2) dashes = 2;
    int l = dashes / 2, r = dashes - l;
    printf("  \033[36m--------  ");
    for (int i = 0; i < l; i++) putchar('-');
    printf(" %s ", title);
    for (int i = 0; i < r; i++) putchar('-');
    printf("\033[0m\n");
}

static void rfid_help(void)
{
    int maxw = 0;                                          /* widest verb name -> summaries line up */
    for (int i = 0; i < NVERBS; i++) { int w = (int)strlen(RFID_VERBS[i].name); if (w > maxw) maxw = w; }
    printf("commands:  (`help <command>` for options + examples)\n");
    for (int sec = 0; sec < NSECT; sec++) {
        rfid_section_hdr(RFID_SECTIONS[sec]);
        for (int i = 0; i < NVERBS; i++) {
            if (RFID_VERBS[i].section != sec) continue;
            int pad = maxw + 2 - (int)strlen(RFID_VERBS[i].name);
            if (RFID_VERBS[i].local) printf("  " C_YELLOW "%s" C_RESET "%*s%s\n", RFID_VERBS[i].name, pad, "", RFID_VERBS[i].help);
            else                     printf("  %s%*s%s\n", RFID_VERBS[i].name, pad, "", RFID_VERBS[i].help);
        }
    }
    printf("\n  \033[90ma <protocol> is a tag type (mfc, ul, t5577, ...); `list` shows all\033[0m\n");
}

/* `help <command>`: description, usage, an option legend, and examples (pm3-style). */
static void rfid_help_cmd(const char *verb)
{
    char v[32]; snprintf(v, sizeof v, "%s", verb ? verb : "");
    char *p = v; while (*p == ' ') p++;
    size_t l = strlen(p); while (l && p[l-1] == ' ') p[--l] = 0;
    if (!strcmp(p, "?")) p = "help";
    int idx = -1;
    for (int i = 0; i < NVERBS; i++) if (!strcmp(RFID_VERBS[i].name, p)) { idx = i; break; }
    if (idx < 0) { printf("rfid: no such command '%s' (try `help`)\n", p); return; }
    const rfid_verb_t *c = &RFID_VERBS[idx];
    printf("%s\n\nusage:\n    %s%s%s\n", c->help, c->name, c->args[0] ? " " : "", c->args);
    if (c->opts) {
        printf("\noptions:\n");
        for (const rfid_hopt *o = c->opts; o->flag; o++) printf("    %-16s %s\n", o->flag, o->desc);
    }
    if (c->examples) {
        printf("\nexamples:\n");
        for (const char *const *e = c->examples; *e; e++) printf("    %s\n", *e);
    }
}

/* If `t` is a bare known verb (typed with no arguments) that requires one, return its arg-syntax string
 * so the dispatcher can show `usage: <verb> <args>` instead of forwarding it (which would just draw an
 * "unknown command"). A verb needs an argument when its syntax starts with '<' (or it's `field`); verbs
 * whose syntax is optional ('[...]') or empty (search, list, trace, help, exit) are valid bare. */
static const char *bare_verb_needs_arg(const char *t)
{
    for (int i = 0; i < NVERBS; i++)
        if (!strcmp(t, RFID_VERBS[i].name)) {
            const char *args = RFID_VERBS[i].args;
            if (args[0] == '<' || !strcmp(RFID_VERBS[i].name, "field")) return args;
            return NULL;                                   /* bare is a valid invocation */
        }
    return NULL;                                           /* not a known verb -> let the device answer */
}

/* Transitional: map a sub-category + op to today's module ELF key. Folds away once the
 * modules are renamed to the <sub>_<op> convention (nfca_sniff, lf_read, ...). */
static const char *shared_op_module(int ri, uint32_t op)
{
    int sub = RFID_REG[ri].sub;
    if (SUB[sub].band == BAND_HF)
        return op == OP_SNIFF ? "sniff" : op == OP_RAW ? "raw" : NULL;
    if (SUB[sub].band == BAND_LF && !strcmp(RFID_REG[ri].token, "t5577") &&
        (op == OP_READ || op == OP_WRITE)) return "t5577";
    return NULL;
}
static int op_built(int i, uint32_t op)
{
    const char *m = shared_op_module(i, op);
    if (m) return access(elf_path(m), F_OK) == 0;

    /* Independent operations follow the feature-module naming convention.
     * A future ul_read/UHF module therefore becomes available without another
     * protocol-specific branch in this general list renderer. */
    const char *suffix = op == OP_READ ? "read" : op == OP_WRITE ? "write" :
                         op == OP_EMU ? "emu" : op == OP_COLLECT ? "collect" :
                         op == OP_SNIFF ? "sniff" : op == OP_RAW ? "raw" : NULL;
    if (!suffix) return 0;
    char derived[96];
    snprintf(derived, sizeof derived, "%s_%s", RFID_REG[i].token, suffix);
    return access(elf_path(derived), F_OK) == 0;
}

/* `list [band|sub-category]` - host-rendered from the registry as a band > sub-category >
 * protocol tree. Availability is DYNAMIC: a protocol is "available" only if its module ELF is
 * built for the connected arch (per-device caps() is the next refinement). A bare filter
 * that names a band shows the whole band; naming a sub-category shows just that one. */
static void rfid_list(const char *filter)
{
    if (filter && !*filter) filter = NULL;
    for (int bd = 0; bd < NBAND; bd++) {
        int band_sel = !filter || !strcmp(filter, BAND[bd].tok);
        if (!band_sel) {                                   /* not this band by name - is the filter a sub-cat in it? */
            int hit = 0;
            for (int s = 0; s < NSUB; s++) if (SUB[s].band == bd && !strcmp(filter, SUB[s].tok)) hit = 1;
            if (!hit) continue;
        }
        printf("\033[1m%s\033[0m\n", BAND[bd].name);       /* band header */
        for (int s = 0; s < NSUB; s++) {
            if (SUB[s].band != bd) continue;
            if (!band_sel && strcmp(filter, SUB[s].tok)) continue;   /* filter names one sub-cat: skip the others */
            printf("  \033[36m%s\033[0m\n", SUB[s].name);  /* sub-category header (cyan) */
            int shown = 0;
            for (int i = 0; i < NREG; i++) {
                if (RFID_REG[i].sub != s) continue;
                char ops[256] = "";
                for (int o = 0; o < NOPS; o++)
                    if (RFID_REG[i].ops & OPS[o].bit) {
                        int built = op_built(i, OPS[o].bit);
                        strncat(ops, built ? "\033[32m" : "\033[90m",
                                sizeof ops - strlen(ops) - 1);
                        strncat(ops, OPS[o].name, sizeof ops - strlen(ops) - 1);
                        if (!built) strncat(ops, "*", sizeof ops - strlen(ops) - 1);
                        strncat(ops, "\033[0m", sizeof ops - strlen(ops) - 1);
                        strncat(ops, " ", sizeof ops - strlen(ops) - 1);
                    }
                printf("    %-6s %-24s %s\n", RFID_REG[i].token, RFID_REG[i].label, ops);
                shown = 1;
            }
            if (!shown) printf("    \033[90m(none yet)\033[0m\n");
        }
    }
    printf("  \033[90m* not built for this target architecture\033[0m\n");
}

static const char *RFID_FIELD[] = { "on", "off", "status" };
static const char *RFID_TRACE[] = { "clear" };
static const char **g_cand; static int g_nc;

static char *rfid_gen(const char *text, int state)
{
    static int i; static size_t tl;
    if (state == 0) { i = 0; tl = strlen(text); }
    while (i < g_nc) { const char *n = g_cand[i++]; if (!strncmp(n, text, tl)) return strdup(n); }
    return NULL;
}

static char **rfid_completion(const char *text, int start, int end)
{
    (void)end;
    /* Completion candidates derived from the registry: verbs, list scopes (band+sub-cat), and
     * protocols filtered by which op each supports - so `raw <TAB>` offers only raw-capable protocols. */
    static const char *verbs[NVERBS], *scopes[NBAND + NSUB];
    static const char *tsniff[NREG], *traw[NREG], *tread[NREG], *twrite[NREG];
    static int nsniff = 0, nraw = 0, nread = 0, nwrite = 0, init = 0;
    if (!init) {
        for (int i = 0; i < NVERBS; i++) verbs[i]  = RFID_VERBS[i].name;
        for (int i = 0; i < NBAND;  i++) scopes[i] = BAND[i].tok;            /* list/search scope = band ... */
        for (int i = 0; i < NSUB;   i++) scopes[NBAND + i] = SUB[i].tok;     /* ... or (list) sub-category */
        for (int i = 0; i < NREG;   i++) {
            if (RFID_REG[i].ops & OP_SNIFF) tsniff[nsniff++] = RFID_REG[i].token;
            if (RFID_REG[i].ops & OP_RAW)   traw[nraw++]     = RFID_REG[i].token;
            if (RFID_REG[i].ops & OP_READ)  tread[nread++]   = RFID_REG[i].token;   /* read scope */
            if (RFID_REG[i].ops & OP_WRITE) twrite[nwrite++] = RFID_REG[i].token;
        }
        init = 1;
    }
    /* `emulate` consumes a local host path, so let readline's normal filename
     * completer handle its arguments. Every other branch is vocabulary owned
     * by this sub-CLI and must not fall through to unrelated host filenames. */
    if (start > 0 && !strncmp(rl_line_buffer, "emulate ", 8)) {
        rl_attempted_completion_over = 0;
        return NULL;
    }
    rl_attempted_completion_over = 1;
    if (start == 0)                                  { g_cand = verbs;      g_nc = NVERBS; }
    else if (!strncmp(rl_line_buffer, "field ",  6)) { g_cand = RFID_FIELD; g_nc = 3; }
    else if (!strncmp(rl_line_buffer, "trace ",  6)) { g_cand = RFID_TRACE; g_nc = 1; }
    else if (!strncmp(rl_line_buffer, "sniff ",  6)) { g_cand = tsniff;     g_nc = nsniff; }
    else if (!strncmp(rl_line_buffer, "raw ",    4)) { g_cand = traw;       g_nc = nraw; }
    else if (!strncmp(rl_line_buffer, "write ",  6)) { g_cand = twrite;     g_nc = nwrite; }
    else if (!strncmp(rl_line_buffer, "read ",   5)) { g_cand = tread;      g_nc = nread; }
    else if (!strncmp(rl_line_buffer, "search ", 7)) { g_cand = scopes;     g_nc = NBAND; }   /* bands only */
    else if (!strncmp(rl_line_buffer, "list ",   5)) { g_cand = scopes;     g_nc = NBAND + NSUB; }
    else return NULL;
    return rl_completion_matches(text, rfid_gen);
}

/* Resolve a sniff/raw <protocol>: set the active annotator for the trace that follows, and rewrite a
 * protocol token (mfc, ulc, ...) to the RF-category token the device parses - the device only ever sees
 * categories; the host owns the protocol layer. `search` is not here: it scans by band (hf/lf), so its
 * argument passes to the device verbatim rather than being mapped from a protocol. Writes to `out` and
 * Returns: 0 forward `line` unchanged, 1 forward `out` (rewritten), 2 handled here (an error was
 * printed - do not forward). Interactive path only. */
static int rfid_resolve(const char *line, char *out, size_t outsz)
{
    const char *verb; uint32_t need; int set_ann;
    if      (!strncmp(line, "sniff ",  6)) { verb = "sniff";  need = OP_SNIFF; set_ann = 1; }
    else if (!strncmp(line, "raw ",    4)) { verb = "raw";    need = OP_RAW;   set_ann = 1; }
    else return 0;

    const char *rest = line + strlen(verb);
    while (*rest == ' ') rest++;
    const char *e = rest; while (*e && *e != ' ') e++;     /* first token = candidate protocol */
    int tlen = (int)(e - rest);

    for (int i = 0; i < NREG; i++) {
        if ((int)strlen(RFID_REG[i].token) == tlen && !strncmp(rest, RFID_REG[i].token, tlen)) {
            if (!(RFID_REG[i].ops & need)) {               /* protocol exists but not for this verb */
                printf("%s: %s doesn't support %s (see `list`)\n", verb, RFID_REG[i].token, verb);
                return 2;                                  /* handled - don't send garbage to the device */
            }
            /* HF techs share the one nfca module -> rewrite to "nfca"; LF protocols each have their
             * own device module (lf read, t5577 write) -> keep the token so the device routes it. */
            const char *devcat = (SUB[RFID_REG[i].sub].band == BAND_HF) ? "nfca" : RFID_REG[i].token;
            if (set_ann) g_ann = RFID_REG[i].ann;
            snprintf(out, outsz, "%s %s%s", verb, devcat, e);   /* e keeps the leading space + rest */
            return 1;
        }
    }
    if (set_ann) g_ann = &ann_nfca;                        /* bare category / no protocol -> base annotator */
    return 0;
}

/* Send one already-resolved line to the device app (append the CR the app's line reader wants). */
static void send_dev_line(const char *snd)
{
    size_t ll = strlen(snd);
    char *sn = malloc(ll + 2);
    if (sn) { memcpy(sn, snd, ll); sn[ll] = '\r'; sn[ll + 1] = 0;
              send_input((const uint8_t *)sn, ll + 1); free(sn); }
}

/* Parsed read/write arguments: a required protocol token followed by flags (order-free):
 *   -b <block>   block number (absent on read => dump all blocks)
 *   -d <hex>     data word to write (write only)
 *   -k <key>     optional MFC key or T5577 password
 *   -s [path]    save a read as JSON (read only) */
typedef struct {
    int  ri;                     /* registry index of the protocol */
    int  block;                  /* -1 if -b absent */
    char data[64];               /* -d value (write) */
    char key[64];                /* -k value */
    int  have_data, have_key;
    char save[256];              /* -s path ("" with have_save = default name in cwd) */
    int  have_save;
} rw_args_t;

/* Copy the next space-delimited token of *s into buf; advance *s past it. Returns token length. */
static int next_tok(const char **s, char *buf, size_t sz)
{
    while (**s == ' ') (*s)++;
    const char *v = *s; while (**s && **s != ' ') (*s)++;
    int n = (int)(*s - v); if (n >= (int)sz) n = (int)sz - 1;
    memcpy(buf, v, n); buf[n] = 0;
    return n;
}

#define READ_USAGE  "usage: read <protocol> [-b <block>] [-k <key>] [-s [file]]   (omit -b to dump all; -s saves JSON)"
#define WRITE_USAGE "usage: write <protocol> -b <block> -d <hex> [-k <key>]"

/* Resolve the protocol token then parse -b/-d/-k flags of `rest` into `a`. `verb` is "read"/"write" for
 * messages; `need` is the op the protocol must advertise; `usage` is shown when args are missing. Returns 0
 * ok, -1 on error (prints why). */
static int parse_rw(const char *rest, rw_args_t *a, const char *verb, uint32_t need, const char *usage)
{
    a->block = -1; a->have_data = a->have_key = a->have_save = 0; a->data[0] = a->key[0] = a->save[0] = 0; a->ri = -1;

    char tok[64];
    if (next_tok(&rest, tok, sizeof tok) == 0) { printf("%s\n", usage); return -1; }
    for (int i = 0; i < NREG; i++)
        if (!strcmp(tok, RFID_REG[i].token)) { a->ri = i; break; }
    if (a->ri < 0) { printf("%s: unknown protocol (see `list`)\n", verb); return -1; }
    if (!(RFID_REG[a->ri].ops & need)) { printf("%s: %s doesn't support %s (see `list`)\n", verb, tok, verb); return -1; }

    while (*rest) {
        char flag[64];
        if (next_tok(&rest, flag, sizeof flag) == 0) break;
        if (flag[0] != '-' || flag[1] == 0 || flag[2] != 0) { printf("%s: unexpected '%s' (use -b/-d/-k/-s)\n", verb, flag); return -1; }
        if (flag[1] == 's') {                                  /* -s [file] : the path is optional */
            a->have_save = 1;
            const char *peek = rest; char val[256];
            if (next_tok(&peek, val, sizeof val) > 0 && val[0] != '-') { snprintf(a->save, sizeof a->save, "%s", val); rest = peek; }
            continue;
        }
        char val[64];
        if (next_tok(&rest, val, sizeof val) == 0) { printf("%s: -%c needs a value\n", verb, flag[1]); return -1; }
        switch (flag[1]) {
            case 'b': {
                char *end; long b = strtol(val, &end, 0);
                if (*end || b < 0 || b > 255) { printf("%s: invalid block number\n", verb); return -1; }
                a->block = (int)b; break;
            }
            case 'd': snprintf(a->data, sizeof a->data, "%s", val); a->have_data = 1; break;
            case 'k': snprintf(a->key,  sizeof a->key,  "%s", val); a->have_key  = 1; break;
            default:  printf("%s: unknown flag -%c\n", verb, flag[1]); return -1;
        }
    }
    return 0;
}

/* `write <protocol> -b <block> -d <hex> [-k <key>]`: normalize flags into the device app's corresponding
 * write verb. Writes the device line to `out`, returns 1, or prints why and returns 0. */
static int rfid_write(const char *rest, char *out, size_t outsz)
{
    rw_args_t a;
    if (parse_rw(rest, &a, "write", OP_WRITE, WRITE_USAGE) != 0) return 0;
    if (a.have_save) { printf("write: -s is only valid with read\n"); return 0; }
    if (a.block < 0 || !a.have_data) { printf("%s\n", WRITE_USAGE); return 0; }
    if (!strcmp(RFID_REG[a.ri].token, "t5577")) {
        if (a.block > 7) { printf("write: T5577 block must be 0-7\n"); return 0; }
        size_t n = strlen(a.data);
        int valid = n == 8;
        for (size_t i = 0; valid && i < n; i++) valid = hexnib(a.data[i]) >= 0;
        if (!valid) {
            printf("write: T5577 data must be exactly 8 hex digits\n");
            return 0;
        }
        if (a.have_key && !fixed_hex(a.key, 8)) {
            printf("write: T5577 password must be exactly 8 hex digits\n");
            return 0;
        }
    }
    /* T5577 passwords are accepted today so scripts do not need to change when
     * password-mode downlink lands. The device currently performs the same
     * unpassworded write and intentionally receives no key field. */
    snprintf(out, outsz, "write %s %d %s", RFID_REG[a.ri].token, a.block, a.data);
    return 1;
}

/* `read <protocol> [-b <block>] [-k <key>]`: with -b, normalize flags into the device app's read verb.
 * Without -b return 2 so the caller can render a whole-tag dump. Scoped to LF block-read protocols
 * (currently T5577). Returns 0 on error. */
static int rfid_read(const char *rest, char *out, size_t outsz, rw_args_t *a_out)
{
    rw_args_t a;
    if (parse_rw(rest, &a, "read", OP_READ, READ_USAGE) != 0) return 0;
    if (a.have_data) { printf("read: -d is only valid with write\n"); return 0; }
    if (!strcmp(RFID_REG[a.ri].token, "mfc")) {
        if (a.block > 63) { printf("read: MIFARE Classic 1K block must be 0-63\n"); return 0; }
        if (a.have_key && !fixed_hex(a.key, 12)) {
            printf("read: MIFARE Classic key must be exactly 12 hex digits\n");
            return 0;
        }
        if (a_out) *a_out = a;
        return 3;
    }
    if (SUB[RFID_REG[a.ri].sub].band != BAND_LF) {
        printf("read: %s has no block read (see `list`)\n", RFID_REG[a.ri].token); return 0;
    }
    if (a.block > 7) { printf("read: T5577 block must be 0-7\n"); return 0; }
    if (a.have_key && !fixed_hex(a.key, 8)) {
        printf("read: T5577 password must be exactly 8 hex digits\n");
        return 0;
    }
    if (a_out) *a_out = a;
    if (a.block < 0) { if (a_out) a_out->have_key = a.have_key; return 2; }   /* bare read -> dump all blocks */
    /* See rfid_write(): accept but intentionally omit the not-yet-wired T5577
     * password. No warning is emitted because -k is a supported future field,
     * not an accidental/unknown option. */
    snprintf(out, outsz, "read %s %d", RFID_REG[a.ri].token, a.block);
    return 1;
}

/* ============================ themed bare-read dump ============================ */

#define CAPTURE_SILENT        0
#define CAPTURE_LIVE          1
#define CAPTURE_READ_PROGRESS 2

static int s_read_progress_active;
static int s_read_progress_tty;

/* Rich read commands buffer the module's machine-readable dump until the final
 * table can be rendered. Keep the user informed without echoing that raw dump:
 * a terminal gets one carriage-return-updated line; redirected output gets one
 * stable "reading..." line instead of a stream of progress records. */
static void read_progress_begin(const char *detail)
{
    s_read_progress_active = 1;
    s_read_progress_tty = isatty(STDOUT_FILENO);
    if (s_read_progress_tty)
        printf("\rreading: %s\033[K", detail ? detail : "card");
    else
        printf("reading...\n");
    fflush(stdout);
}

static void read_progress_update(const char *line, size_t len)
{
    static const char prefix[] = "reading:";
    while (len && (line[len - 1] == '\r' || line[len - 1] == '\n')) len--;
    if (!s_read_progress_active || !s_read_progress_tty ||
        len < sizeof prefix - 1 || memcmp(line, prefix, sizeof prefix - 1) != 0)
        return;
    printf("\r%.*s\033[K", (int)len, line);
    fflush(stdout);
}

static void read_progress_end(void)
{
    if (s_read_progress_active && s_read_progress_tty) {
        printf("\r\033[K");
        fflush(stdout);
    }
    s_read_progress_active = 0;
}

/* Send `cmd` to the app session and CAPTURE its output into `buf` (up to `sz`), serving module fetches
 * inline and stopping at the trailing "rfid> " prompt (stripped). Returns 0, -1 on link loss, or -2 when
 * SIGINT/SIGTERM asks the outer app loop to stop this blocking command. */
static int app_capture(uint32_t sid, const char *cmd, char *buf, size_t sz, int live)
{
    send_dev_line(cmd);
    size_t len = 0, shown = 0; if (sz) buf[0] = 0;
    for (;;) {
        if (g_rfid_sigstop) return -2;
        CliResponse resp;
        int r = recv_resp(&resp);
        if (r < 0) return -1;
        if (g_rfid_sigstop) return -2;
        if (r == 0) continue;                              /* partial frame - keep reading */
        if (resp.id != sid) continue;
        if (resp.which_payload == CliResponse_output_tag && resp.payload.output[0]) {
            for (const char *o = resp.payload.output; *o && len + 1 < sz; o++) buf[len++] = *o;
            buf[len] = 0;
            /* Echo complete progress lines and retain a partial trailing line,
             * which may be the prompt. */
            if (live == CAPTURE_LIVE) {
                size_t end = len;
                while (end > shown && buf[end - 1] != '\n') end--;   /* end = just past the last '\n' */
                if (end > shown) { fwrite(buf + shown, 1, end - shown, stdout); fflush(stdout); shown = end; }
            } else if (live == CAPTURE_READ_PROGRESS) {
                size_t end = len;
                while (end > shown && buf[end - 1] != '\n') end--;
                while (shown < end) {
                    size_t eol = shown;
                    while (eol < end && buf[eol] != '\n') eol++;
                    read_progress_update(buf + shown, eol - shown);
                    shown = eol + (eol < end);
                }
            }
        } else if (resp.which_payload == CliResponse_module_request_tag) {
            serve_module(resp.payload.module_request);
        } else if (resp.which_payload == CliResponse_error_tag) {
            return -1;
        }
        if (len >= 6 && !memcmp(buf + len - 6, "rfid> ", 6)) {
            len -= 6; buf[len] = 0;
            if (live == CAPTURE_LIVE && len > shown) {
                fwrite(buf + shown, 1, len - shown, stdout);
                fflush(stdout);
            }
            return 0;
        }
    }
}

/* ---- framed themed box (host-rendered; mirrors tools/mfc_output.py) ---- */
#define BOX_INNER 58                              /* default interior width; render_mfc widens it */
static int box_inner = BOX_INNER;
#define A_RST  "\033[0m"
#define A_BOLD "\033[1m"
#define A_DIM  "\033[2m"

static int box_vlen(const char *s)   /* visible columns: skip ANSI CSI...m, count UTF-8 chars once */
{
    int n = 0;
    for (const unsigned char *p = (const unsigned char *)s; *p; p++) {
        if (*p == 0x1b) { while (*p && *p != 'm') p++; if (!*p) break; }
        else if ((*p & 0xC0) != 0x80) n++;              /* count lead bytes, skip UTF-8 continuation */
    }
    return n;
}

static void box_rule(const char *l, const char *m, const char *r)   /* top/sep/bottom */
{
    const theme_t *t = g_theme; int color = theme_color();
    if (color) printf("%s%s", t->bg, t->border);
    printf("%s", l);
    for (int i = 0; i < box_inner + 2; i++) printf("─");
    printf("%s%s\n", r, color ? A_RST : ""); (void)m;
}
static void box_top(void)    { box_rule("┌", "", "┐"); }
static void box_sep(void)    { box_rule("├", "", "┤"); }
static void box_bottom(void) { box_rule("└", "", "┘"); }

static void box_row(const char *content)
{
    const theme_t *t = g_theme;
    int pad = box_inner - box_vlen(content); if (pad < 0) pad = 0;
    if (!theme_color()) { printf("│ %s%*s │\n", content, pad, ""); return; }
    /* reassert the fill + neutral fg after every reset, so the background never drops mid-row */
    char buf[2048]; size_t o = 0;
    for (const char *s = content; *s && o + 40 < sizeof buf; ) {
        if (!strncmp(s, A_RST, 4)) { o += (size_t)snprintf(buf + o, sizeof buf - o, "%s%s%s", A_RST, t->bg, t->neutral); s += 4; }
        else buf[o++] = *s++;
    }
    buf[o] = 0;
    printf("%s%s│%s%s %s%*s %s│%s\n", t->bg, t->border, t->bg, t->neutral, buf, pad, "", t->border, A_RST);
}

/* Render an all-blocks T5577 dump as a themed box. blk[i]/ok[i] for blocks 0..n-1 (block 0 = config). */
static void render_t5577(const uint32_t *blk, const int *ok, int n)
{
    const theme_t *t = g_theme;
    static const int rates[8] = { 8, 16, 32, 40, 50, 64, 100, 128 };
    uint32_t cfg = blk[0]; int cfg_ok = ok[0];
    int rate   = cfg_ok ? rates[(cfg >> 18) & 7] : 0;
    int mod    = cfg_ok ? (int)((cfg >> 12) & 0x1F) : -1;
    int maxblk = cfg_ok ? (int)((cfg >> 5) & 7) : n - 1;
    if (maxblk >= n) maxblk = n - 1;
    const char *modname = mod == 0x08 ? "Manchester" : mod == 0 ? "Direct" :
                          mod == 0x10 ? "Bi-phase"   : mod == 0x01 ? "PSK1"  : "modulated";
    char s[256];

    printf("\n");
    box_top();
    snprintf(s, sizeof s, "%s%sT5577%s  %sATA5577 · 125 kHz LF%s", t->accent, A_BOLD, A_RST, A_DIM, A_RST);
    box_row(s);
    box_sep();
    snprintf(s, sizeof s, "%-8s %s%08X%s", "Config", t->accent, cfg, A_RST);
    box_row(s);
    if (cfg_ok) {
        snprintf(s, sizeof s, "%-8s %sRF/%d%s  %s%s%s  %s%d data block%s%s",
                 "Encode", t->accent, rate, A_RST, t->accent, modname, A_RST,
                 A_DIM, maxblk, maxblk == 1 ? "" : "s", A_RST);
        box_row(s);
    }
    box_sep();
    snprintf(s, sizeof s, "%sblk   hex data       role%s", A_DIM, A_RST);
    box_row(s);
    int got = 0;
    int np0 = n < 8 ? n : 8;
    for (int b = 0; b < np0; b++) {                        /* page 0: config (0) + user blocks 1-7 */
        char hex[24];
        if (ok[b]) { snprintf(hex, sizeof hex, "%02X %02X %02X %02X",
                            (blk[b] >> 24) & 0xFF, (blk[b] >> 16) & 0xFF, (blk[b] >> 8) & 0xFF, blk[b] & 0xFF); got++; }
        else       snprintf(hex, sizeof hex, "-- -- -- --");
        const char *mark = (b == 0) ? "▸" : " ";
        /* block 0 = config; 1..maxblk = the data the tag modulates by default; the rest exist but are unused. */
        const char *role = (b == 0) ? "config" : (b <= maxblk) ? "data" : "unused";
        const char *hcol = !ok[b] ? A_DIM : (b == 0) ? t->alert : (b <= maxblk) ? t->accent : A_DIM;
        snprintf(s, sizeof s, "%s%s%s%s%2d%s   %s%s%s   %s%s%s",
                 t->alert, mark, A_RST, A_DIM, b, A_RST, hcol, hex, A_RST, A_DIM, role, A_RST);
        box_row(s);
    }
    if (n > 8) {                                           /* page 1: traceability (1-2) + analog option (3) */
        box_sep();
        snprintf(s, sizeof s, "%spage 1   hex data       role%s", A_DIM, A_RST);
        box_row(s);
        for (int i = 8; i < n; i++) {
            int pb = i - 7;                                /* array idx 8/9/10 -> page-1 block 1/2/3 */
            char hex[24];
            if (ok[i]) { snprintf(hex, sizeof hex, "%02X %02X %02X %02X",
                                (blk[i] >> 24) & 0xFF, (blk[i] >> 16) & 0xFF, (blk[i] >> 8) & 0xFF, blk[i] & 0xFF); got++; }
            else       snprintf(hex, sizeof hex, "-- -- -- --");
            const char *role = (pb <= 2) ? "trace" : "analog";
            const char *hcol = !ok[i] ? A_DIM : t->accent;
            snprintf(s, sizeof s, "%s %s%s%2d%s   %s%s%s   %s%s%s",
                     t->alert, A_RST, A_DIM, pb, A_RST, hcol, hex, A_RST, A_DIM, role, A_RST);
            box_row(s);
        }
    }
    box_sep();
    snprintf(s, sizeof s, "%s%s✓ read complete%s   %s%d/%d blocks%s",
             got == n ? t->accent : t->alert, A_BOLD, A_RST, A_DIM, got, n, A_RST);
    box_row(s);
    box_bottom();
    printf("\n");
}

/* Bare `read <protocol>`: read block 0 (config), learn how many data blocks it declares, read those too,
 * and render the themed dump. Each read is a synchronous captured `raw <token> <block>`. */
/* ============================ dump save (JSON) ============================ */

/* Write a Proxmark-semi-compatible dump JSON to `path`: lowercase keys, NO "Card" array, "created":"fantasi",
 * "protocol":<the read protocol token>, "version":1, and a "blocks" object mapping "<n>" -> "<uppercase hex>".
 * blockhex[i] == "" is skipped (block not read). Returns 0 on success, -1 on error. Consumed by `emulate`. */
static int write_dump_json(const char *protocol, char (*blockhex)[40], int nblocks, const char *path,
                           const char *prng)
{
    FILE *f = fopen(path, "w");
    if (!f) { printf("save: cannot write '%s': %s\n", path, strerror(errno)); return -1; }
    fprintf(f, "{\n  \"created\": \"fantasi\",\n  \"protocol\": \"%s\",\n  \"version\": 1,\n", protocol);
    if (prng) fprintf(f, "  \"prng\": \"%s\",\n", prng);          /* MIFARE Classic PRNG class -> emulate nonce */
    fprintf(f, "  \"blocks\": {\n");
    int first = 1;
    for (int i = 0; i < nblocks; i++) {
        if (!blockhex[i][0]) continue;
        fprintf(f, "%s    \"%d\": \"%s\"", first ? "" : ",\n", i, blockhex[i]);
        first = 0;
    }
    fprintf(f, "\n  }\n}\n");
    fclose(f);
    printf("saved -> %s\n", path);
    return 0;
}

static void rfid_read_all(uint32_t sid, const rw_args_t *a)
{
    (void)a;
    /* Dump all 8 page-0 blocks (block 0 = config, 1-7 = user memory) plus page-1 blocks 1-3 (traceability +
     * analog-option register). The config's MAX-BLOCK field only sets how many the tag modulates in its
     * default read, not how many exist, so a dump reads every physical block. Layout: blk[0..7] = page 0
     * blocks 0-7; blk[8..10] = page 1 blocks 1,2,3.
     *
     * One device command: the t5577_dump module calibrates once and reads every block in a single module
     * invocation, so the whole tag comes back in one round-trip instead of 11 (each of which re-fetched the
     * module and intermittently stalled on the USB transport). It prints one line per block, which we parse:
     *   t5577: p0 b0 = 00148040   (or "= --------" when a block didn't decode). */
    uint32_t blk[11] = {0}; int ok[11] = {0};
    char cap[1024];
    read_progress_begin("preparing T5577 reader");
    int cr = app_capture(sid, "read t5577", cap, sizeof cap, CAPTURE_READ_PROGRESS);
    if (cr != 0) {
        read_progress_end();
        printf("\nread: %s\n", cr == -2 ? "cancelled" : "link lost");
        return;
    }
    read_progress_end();

    for (const char *p = cap; (p = strstr(p, "t5577: p")) != NULL; p++) {
        int page = p[8] - '0';                             /* "p<page> b<block> = <hex>" */
        const char *bp = strstr(p, " b");
        const char *eq = strstr(p, "= ");
        if (!bp || !eq || eq < bp) continue;
        int block = atoi(bp + 2);
        if (eq[2] == '-') continue;                        /* "--------" -> undecodable, leave ok=0 */
        int idx = (page == 0) ? block : 7 + block;         /* p0 b0-7 -> 0-7; p1 b1-3 -> 8-10 */
        if (idx < 0 || idx >= 11) continue;
        blk[idx] = (uint32_t)strtoul(eq + 2, NULL, 16);
        ok[idx] = 1;
    }
    if (!ok[0]) { printf("\nread: could not read the config block (block 0)\n"); return; }
    render_t5577(blk, ok, 11);

    if (a->have_save) {                                    /* JSON: pm3 t55x7 numbering 0-11 (8 = config alias) */
        char bh[12][40];
        for (int i = 0; i < 8; i++) { if (ok[i]) snprintf(bh[i], 40, "%08X", (unsigned)blk[i]); else bh[i][0] = 0; }
        snprintf(bh[8], 40, "%08X", (unsigned)blk[0]);     /* page-1 block 0 aliases the page-0 config */
        for (int i = 1; i <= 3; i++) { if (ok[7 + i]) snprintf(bh[8 + i], 40, "%08X", (unsigned)blk[7 + i]); else bh[8 + i][0] = 0; }
        char path[300];
        if (a->save[0]) snprintf(path, sizeof path, "%s", a->save);
        else snprintf(path, sizeof path, "lf-t55xx-%08X-%08X-dump.json", (unsigned)blk[1], (unsigned)blk[2]);
        write_dump_json(RFID_REG[a->ri].token, bh, 12, path, NULL);   /* LF t55: no PRNG */
    }
}

/* ---- MIFARE Classic whole-card dump (mirrors tools/mfc_output.py) ---- */
#define MFC_BLOCKS  64                             /* MIFARE Classic 1K: 16 sectors x 4 blocks */
#define MFC_SECTORS 16

/* One block row: "▸ NN  hh hh .. hh  |ascii|", colored by role. The sector trailer (b==3) is field-split
 * into Key A / access bits / Key B; the manufacturer block (0) and trailers get an alert ▸ mark. */
static void mfc_block_row(int gb, const uint8_t *d, int ok)
{
    const theme_t *t = g_theme;
    int sec_blk = gb & 3;
    int trailer = (sec_blk == 3), manuf = (gb == 0);
    const char *mark = (manuf || trailer) ? "▸" : " ";
    const char *mcol = (manuf || trailer) ? t->alert : A_DIM;

    char hex[384] = "", asc[64] = ""; int ho = 0, ao = 0;   /* trailer emits a color escape per byte (~288 B) */
    if (!ok) {
        for (int i = 0; i < 16; i++) ho += snprintf(hex + ho, sizeof hex - ho, "%s-- ", A_DIM);
        snprintf(hex + ho, sizeof hex - ho, "%s", A_RST);
        ao += snprintf(asc + ao, sizeof asc - ao, "%s................%s", A_DIM, A_RST);
    } else if (trailer) {                          /* Key A (accent) | access (alert) | Key B (neutral) */
        for (int i = 0; i < 6; i++)  ho += snprintf(hex + ho, sizeof hex - ho, "%s%02X %s", t->accent, d[i], A_RST);
        for (int i = 6; i < 10; i++) ho += snprintf(hex + ho, sizeof hex - ho, "%s%02X %s", t->alert,  d[i], A_RST);
        for (int i = 10; i < 16; i++)ho += snprintf(hex + ho, sizeof hex - ho, "%s%02X %s", t->neutral, d[i], A_RST);
        for (int i = 0; i < 16; i++) ao += snprintf(asc + ao, sizeof asc - ao, "%c", (d[i] >= 32 && d[i] < 127) ? d[i] : '.');
    } else {
        const char *c = manuf ? t->alert : t->accent;
        ho += snprintf(hex + ho, sizeof hex - ho, "%s", c);
        for (int i = 0; i < 16; i++) ho += snprintf(hex + ho, sizeof hex - ho, "%02X ", d[i]);
        ho += snprintf(hex + ho, sizeof hex - ho, "%s", A_RST);
        for (int i = 0; i < 16; i++) ao += snprintf(asc + ao, sizeof asc - ao, "%c", (d[i] >= 32 && d[i] < 127) ? d[i] : '.');
    }
    char s[512];
    snprintf(s, sizeof s, "%s%s%s%s%3d%s  %s %s|%s|%s", mcol, mark, A_RST, A_DIM, gb, A_RST, hex, t->accent, asc, A_RST);
    box_row(s);
}

/* Render a full MIFARE Classic 1K dump as a themed box. blk[64]/ok[64]
 * hold each block and keyA/keyB hold all 32 recovered sector keys. */
static void render_mfc(const uint8_t uid[4], uint8_t sak, const uint8_t atqa[2],
                       const uint8_t blk[][16], const int *ok, char keyA[][16], char keyB[][16],
                       const char *prng)
{
    const theme_t *t = g_theme;
    char s[512];
    const int nsec = MFC_SECTORS, nblk = MFC_BLOCKS;
    int total = 0; for (int i = 0; i < nblk; i++) if (ok[i]) total++;

    box_inner = 74;
    printf("\n");
    box_top();
    snprintf(s, sizeof s, "%s%sMIFARE Classic%s  %s13.56 MHz · ISO14443-A · %s%s", t->accent, A_BOLD, A_RST,
             A_DIM, "1K", A_RST);
    box_row(s);
    box_sep();
    snprintf(s, sizeof s, "%-5s %s%s%02X:%02X:%02X:%02X%s   %sSAK %02X · ATQA %02X%02X · PRNG %s%s", "UID",
             t->alert, A_BOLD, uid[0], uid[1], uid[2], uid[3], A_RST, A_DIM, sak, atqa[1], atqa[0],
             (prng && prng[0]) ? prng : "?", A_RST);
    box_row(s);
    box_sep();
    snprintf(s, sizeof s, "%sblk   hex bytes%*sascii%s", A_DIM, 41, "", A_RST);
    box_row(s);
    for (int sec = 0; sec < nsec; sec++) {
        const char *ka = keyA[sec][0] ? keyA[sec] : NULL, *kb = keyB[sec][0] ? keyB[sec] : NULL;
        if (ka && kb)
            snprintf(s, sizeof s, "%s%sSECTOR %02d%s   %skey A %.12s · key B %.12s%s", t->alert, A_BOLD, sec, A_RST,
                     A_DIM, ka, kb, A_RST);
        else if (ka || kb)
            snprintf(s, sizeof s, "%s%sSECTOR %02d%s   %skey %c %.12s%s", t->alert, A_BOLD, sec, A_RST,
                     A_DIM, ka ? 'A' : 'B', ka ? ka : kb, A_RST);
        else
            snprintf(s, sizeof s, "%s%sSECTOR %02d%s   %sno key found%s", t->alert, A_BOLD, sec, A_RST, A_DIM, A_RST);
        box_row(s);
        for (int b = 0; b < 4; b++) {
            int gb = sec * 4 + b;
            uint8_t row[16]; memcpy(row, blk[gb], 16);
            /* The trailer's Key A always reads back masked (000000) and Key B often does too; substitute the
             * keys we actually recovered so the dump shows real keys (as mfc_output.py does). */
            if (b == 3 && ok[gb]) {
                if (ka) for (int i = 0; i < 6; i++) { unsigned v = 0; sscanf(ka + i * 2, "%2x", &v); row[i] = (uint8_t)v; }
                if (kb) for (int i = 0; i < 6; i++) { unsigned v = 0; sscanf(kb + i * 2, "%2x", &v); row[10 + i] = (uint8_t)v; }
            }
            mfc_block_row(gb, row, ok[gb]);
        }
    }
    box_sep();
    snprintf(s, sizeof s, "%s%s✓ read complete%s   %s%d/%d blocks%s   %s▸ manufacturer / trailer%s",
             total == nblk ? t->accent : t->alert, A_BOLD, A_RST, A_DIM, total, nblk, A_RST, A_DIM, A_RST);
    box_row(s);
    box_bottom();
    printf("\n");
    box_inner = BOX_INNER;
}

static void render_mfc_block(const uint8_t uid[4], uint8_t sak, const uint8_t atqa[2],
                             int block, const uint8_t data[16], char keytype,
                             const char *key, const char *prng)
{
    const theme_t *t = g_theme;
    char s[512]; uint8_t row[16]; memcpy(row, data, sizeof row);
    if ((block & 3) == 3 && key && fixed_hex(key, 12)) {
        int off = keytype == 'B' ? 10 : 0;
        for (int i = 0; i < 6; i++) {
            unsigned v = 0; sscanf(key + i * 2, "%2x", &v); row[off + i] = (uint8_t)v;
        }
    }

    box_inner = 74;
    printf("\n");
    box_top();
    snprintf(s, sizeof s, "%s%sMIFARE Classic%s  %s13.56 MHz · ISO14443-A · 1K%s",
             t->accent, A_BOLD, A_RST, A_DIM, A_RST);
    box_row(s);
    box_sep();
    snprintf(s, sizeof s, "%-5s %s%s%02X:%02X:%02X:%02X%s   %sSAK %02X · ATQA %02X%02X · PRNG %s%s",
             "UID", t->alert, A_BOLD, uid[0], uid[1], uid[2], uid[3], A_RST,
             A_DIM, sak, atqa[1], atqa[0], (prng && prng[0]) ? prng : "?", A_RST);
    box_row(s);
    snprintf(s, sizeof s, "%sSECTOR %02d%s   key %c %.12s", A_DIM, block / 4, A_RST,
             keytype, key ? key : "????????????");
    box_row(s);
    box_sep();
    snprintf(s, sizeof s, "%sblk   hex bytes%*sascii%s", A_DIM, 41, "", A_RST);
    box_row(s);
    mfc_block_row(block, row, 1);
    box_sep();
    snprintf(s, sizeof s, "%s%s✓ read complete%s   %sblock %d%s",
             t->accent, A_BOLD, A_RST, A_DIM, block, A_RST);
    box_row(s);
    box_bottom();
    printf("\n");
    box_inner = BOX_INNER;
}

/* `collect <proto> <card|reader|sniff> [-u UID] [-k key]`: gather key material to files. `mfc card` streams
 * the on-device Hardnested nonce cloud into host-side mfc.log; sniff runs host mfkey64 and writes the recovered
 * keys to /nfc/mfc.dict. Reader-mode collection is not implemented yet. */
/* Returns 1 if it launched a streaming device command (the caller must not re-arm the prompt - the REPL
 * streams the sniff), or 0 if fully handled synchronously. */
static int rfid_collect(uint32_t sid, const char *args)
{
    char a[160]; snprintf(a, sizeof a, "%s", args ? args : "");
    char *save = NULL;
    const char *proto = strtok_r(a, " ", &save);
    const char *mode  = strtok_r(NULL, " ", &save);
    char uidhex[16] = "", keyhex[16] = "";
    for (char *w = strtok_r(NULL, " ", &save); w; w = strtok_r(NULL, " ", &save)) {
        if (!strcmp(w, "-u")) { char *v = strtok_r(NULL, " ", &save); if (v) snprintf(uidhex, sizeof uidhex, "%s", v); }
        else if (!strcmp(w, "-k")) { char *v = strtok_r(NULL, " ", &save); if (v) snprintf(keyhex, sizeof keyhex, "%s", v); }
    }
    if (!proto || strcmp(proto, "mfc")) { printf("collect: only 'mfc' supported\nusage: collect mfc <card|reader|sniff> [-u UID] [-k key]\n"); return 0; }
    if (!mode) { printf("usage: collect mfc <card|reader|sniff> [-u UID] [-k key]\n"); return 0; }
    if (!strcmp(mode, "reader")) { printf("collect: mfc reader (tag emulation) is not implemented yet\n"); return 0; }
    if (!strcmp(mode, "sniff")) {
        /* Sniff reader<->card auths; the frame parser (mfc_sniff_frame) mfkey64s each and, on stop, writes
         * /nfc/mfc.dict (mfc_sniff_finalize). Reuses the normal sniff streaming path. Press a key to stop. */
        g_mfc_sniff = 1; g_ms_state = 0; g_ms_nkeys = 0; g_ms_uid = 0; g_ms_have_uid = 0;
        printf("collect: sniffing auths - press a key to stop and recover keys via mfkey64\n");
        send_dev_line("sniff nfca");
        return 1;
    }
    if (strcmp(mode, "card")) { printf("usage: collect mfc <card|reader|sniff> [-u UID] [-k key]\n"); return 0; }

    uint8_t cfg[15]; memset(cfg, 0, sizeof cfg);
    if (keyhex[0]) { uint64_t k = strtoull(keyhex, NULL, 16); cfg[0] |= 1; for (int i = 0; i < 6; i++) cfg[1 + i] = (uint8_t)(k >> (40 - i * 8)); }
    if (uidhex[0]) {
        int ul = (int)strlen(uidhex) / 2;
        if (ul == 4 || ul == 7) { cfg[0] |= 2; for (int i = 0; i < ul; i++) { unsigned v = 0; sscanf(uidhex + i * 2, "%2x", &v); cfg[7 + i] = (uint8_t)v; } cfg[14] = (uint8_t)ul; }
        else printf("collect: ignoring -u (need 4 or 7 byte hex UID)\n");
    }
    char cfgtmp[] = "/tmp/fantasi-mfccfg-XXXXXX";
    int cfd = mkstemp(cfgtmp);
    if (cfd < 0) { printf("collect: mkstemp failed\n"); return 0; }
    FILE *cf = fdopen(cfd, "wb");
    if (cf) { fwrite(cfg, 1, sizeof cfg, cf); fclose(cf); } else close(cfd);
    if (upload_ram(MFC_CFG_RAM, cfgtmp, 1) < 0) {
        unlink(cfgtmp);
        printf("collect: config upload failed\n");
        return 0;
    }
    unlink(cfgtmp);
    /* Stream the device output line-by-line (the nonce cloud is ~1500 lines/target, far past any fixed
     * buffer). The module frames each target's nonces with NONCE-BEGIN/END; route those into a local nonce
     * file for the offline solver (hardnested_main) and print only the human `collect:` status lines. */
    const char *noncepath = "mfc.log";
    FILE *nf = NULL; long nonce_count = 0; int in_nonce = 0, link_ok = 1, done = 0, cancelled = 0;
    char line[256]; size_t ll = 0;
    send_dev_line("collect mfc card");
    while (!done) {
        if (g_rfid_sigstop) { cancelled = 1; break; }
        CliResponse resp;
        int r = recv_resp(&resp);
        if (r < 0) { link_ok = 0; break; }
        if (g_rfid_sigstop) { cancelled = 1; break; }
        if (r == 0) continue;
        if (resp.id != sid) continue;
        if (resp.which_payload == CliResponse_module_request_tag) { serve_module(resp.payload.module_request); continue; }
        if (resp.which_payload == CliResponse_error_tag) { link_ok = 0; break; }
        if (resp.which_payload != CliResponse_output_tag) continue;
        for (const char *o = resp.payload.output; *o; o++) {
            if (*o == '\n' || *o == '\r') {
                if (ll == 0) continue;                     /* swallow the \n of a \r\n (and blank lines) */
                line[ll] = 0; ll = 0;
                if (!strncmp(line, "NONCE-BEGIN", 11)) { if (!nf) nf = fopen(noncepath, "w"); in_nonce = 1; }
                else if (!strncmp(line, "NONCE-END", 9)) { in_nonce = 0; }
                else if (in_nonce && !strncmp(line, "Sec ", 4)) { if (nf) { fputs(line, nf); fputc('\n', nf); nonce_count++; } }
                else { char *c = strstr(line, "collect:"); if (c) { printf("%s\n", c); fflush(stdout); } }
            } else if (ll < sizeof line - 1) {
                line[ll++] = *o;
            }
        }
        if (ll >= 6 && !memcmp(line + ll - 6, "rfid> ", 6)) done = 1;   /* trailing prompt (no newline) */
    }
    if (nf) fclose(nf);
    if (cancelled) { printf("collect: cancelled\n"); return 0; }
    if (!link_ok) { printf("collect: link lost\n"); return 0; }
    if (nonce_count) printf("collect: captured %ld nonces -> %s  (crack: hardnested_main %s)\n",
                            nonce_count, noncepath, noncepath);
    return 0;
}

/* `emulate <dump.json>`: emulate a card from a Fantasi/pm3 dump JSON. Parses the "blocks" object into a 1 KB
 * MIFARE Classic image, stages it at /ramfs/mfc_emu.bin, and runs the emulation module (streams until the
 * user presses a key). Returns 1 (streaming) so the REPL forwards keystrokes to stop it, else 0. */
static int rfid_emulate(uint32_t sid, const char *args)
{
    (void)sid;
    char path[256]; snprintf(path, sizeof path, "%s", args ? args : "");
    char *p = path; while (*p == ' ') p++;
    size_t pl = strlen(p); while (pl && (p[pl-1] == ' ' || p[pl-1] == '\n')) p[--pl] = 0;
    if (!*p) { printf("usage: emulate <dump.json>   (MIFARE Classic 1K)\n"); return 0; }

    FILE *f = fopen(p, "rb");
    if (!f) { printf("emulate: cannot open '%s': %s\n", p, strerror(errno)); return 0; }
    static char js[70000];
    size_t jn = fread(js, 1, sizeof js - 1, f); fclose(f); js[jn] = 0;

    char *bl = strstr(js, "\"blocks\"");
    if (!bl) { printf("emulate: no \"blocks\" in '%s'\n", p); return 0; }

    static uint8_t img[1024];
    for (int i = 0; i < 1024; i++) img[i] = 0;
    int got = 0;
    for (char *q = bl; (q = strchr(q, '"')) != NULL; ) {           /* scan "<block>": "<32 hex>" pairs */
        char *end; long bn = strtol(q + 1, &end, 10);
        if (end == q + 1 || *end != '"') { q++; continue; }
        char *colon = strchr(end, ':'); if (!colon || colon - end > 4) { q = end + 1; continue; }
        char *v1 = strchr(colon, '"'); char *v2 = v1 ? strchr(v1 + 1, '"') : NULL;
        if (v1 && v2 && bn >= 0 && bn < 64 && (v2 - (v1 + 1)) == 32) {
            for (int i = 0; i < 16; i++) { unsigned x = 0; sscanf(v1 + 1 + i * 2, "%2x", &x); img[bn * 16 + i] = (uint8_t)x; }
            got++;
        }
        q = v2 ? v2 + 1 : end + 1;
    }
    if (got == 0) { printf("emulate: parsed 0 blocks from '%s'\n", p); return 0; }

    /* meta "prng" (recorded by `read mfc`): shown for reference only - the emulation always answers with a
     * valid weak-LFSR nonce, which is what the real weak/static test card does. */
    char prng[8] = "weak";
    char *pr = strstr(js, "\"prng\"");
    if (pr) {
        char *c = strchr(pr, ':'), *v1 = c ? strchr(c, '"') : NULL, *v2 = v1 ? strchr(v1 + 1, '"') : NULL;
        if (v1 && v2 && (v2 - v1 - 1) > 0 && (v2 - v1 - 1) < (int)sizeof prng) {
            int L = (int)(v2 - v1 - 1); memcpy(prng, v1 + 1, L); prng[L] = 0;
        }
    }

    char tmp[] = "/tmp/fantasi-emu-XXXXXX";
    int fd = mkstemp(tmp);
    if (fd < 0) { printf("emulate: mkstemp failed\n"); return 0; }
    FILE *tf = fdopen(fd, "wb");
    if (tf) { fwrite(img, 1, sizeof img, tf); fclose(tf); } else close(fd);
    if (upload_ram(MFC_EMU_RAM, tmp, 1) < 0) { unlink(tmp); printf("emulate: image upload failed\n"); return 0; }
    unlink(tmp);
    printf("emulate: MIFARE Classic 1K, uid=%02X%02X%02X%02X, %d/64 blocks, prng=%s - press a key to stop\n",
           img[0], img[1], img[2], img[3], got, prng);
    send_dev_line("emulate mfc");
    return 1;
}

/* `read mfc`: dictionary read. Stage the general dictionary, run the read module (which tries it plus the
 * card-specific /nfc/mfc.dict - both streamed - and dumps each block under whichever key reads it), then
 * render. Key RECOVERY is the job of `collect`; read just uses known/dictionary keys. */
static void rfid_read_mfc(uint32_t sid, const rw_args_t *a)
{
    static char cap[16384];
    /* The dictionaries already live on the device: /nfc/mfc_dict.dic (general, flashed by `make flash`) and
     * /nfc/mfc.dict (card-specific, written by `collect mfc sniff`). The module reads them directly. The RFID
     * driver requests mfc_read only when this command reaches it; serve_module() streams the ELF to /ramfs,
     * and fantasi_run_module(..., true) deletes that source as soon as it has been loaded. */
    char devcmd[96];
    if (a->block >= 0)
        snprintf(devcmd, sizeof devcmd, "read mfc %d%s%s", a->block,
                 a->have_key ? " " : "", a->have_key ? a->key : "");
    else if (a->have_key)
        snprintf(devcmd, sizeof devcmd, "read mfc all %s", a->key);
    else
        snprintf(devcmd, sizeof devcmd, "read mfc");

    read_progress_begin(a->block >= 0 ? "preparing MIFARE Classic block read" :
                                        "preparing MIFARE Classic reader");
    /* Buffer raw MFC records for the themed table while showing only the
     * module's reading-status records on the in-place progress line. */
    int cr = app_capture(sid, devcmd, cap, sizeof cap, CAPTURE_READ_PROGRESS);
    if (cr != 0) {
        read_progress_end();
        /* The resident driver self-cleans on every normal return. A forced
         * app_stop can bypass that epilogue, so remove only here rather than
         * charging every RFID launch two remote filesystem round trips. */
        if (cr == -2) { delete_ram(MFC_REQ_RAM); delete_ram(MFC_KEY_RAM); }
        printf("\nread: %s\n", cr == -2 ? "cancelled" : "link lost");
        return;
    }
    read_progress_end();

    uint8_t uid[4] = {0}, atqa[2] = {0}, sak = 0, blk[MFC_BLOCKS][16];
    int ok[MFC_BLOCKS] = {0}, seen_blk[MFC_BLOCKS] = {0};
    char keyA[MFC_SECTORS][16], keyB[MFC_SECTORS][16]; int got_uid = 0;
    char prng[8] = "weak";                                 /* PRNG class profiled by the module (mfc: prng=...) */
    char failure[256] = "";
    int fatal = 0, done_sectors = -1, done_block = -1;      /* retain diagnostics while cap is tokenized */
    for (int i = 0; i < MFC_SECTORS; i++) { keyA[i][0] = 0; keyB[i][0] = 0; }
    memset(blk, 0, sizeof blk);
    for (char *line = cap; line && *line; ) {
        char *nl = strchr(line, '\n'); if (nl) *nl = 0;
        char *p;
        if ((p = strstr(line, "uid=")) != NULL) {
            unsigned u0, u1, u2, u3, sk, aq;
            if (sscanf(p, "uid=%2x%2x%2x%2x sak=%2x atqa=%4x", &u0, &u1, &u2, &u3, &sk, &aq) == 6) {
                uid[0] = u0; uid[1] = u1; uid[2] = u2; uid[3] = u3; sak = sk;
                atqa[0] = aq & 0xFF; atqa[1] = (aq >> 8) & 0xFF; got_uid = 1;
            }
        } else if ((p = strstr(line, "sec ")) != NULL) {
            unsigned sc; char kc; char kh[16];
            if (sscanf(p, "sec %u key%c=%12s", &sc, &kc, kh) == 3 &&
                sc < MFC_SECTORS && (kc == 'A' || kc == 'B') && fixed_hex(kh, 12))
                snprintf(kc == 'B' ? keyB[sc] : keyA[sc], 16, "%s", kh);
        } else if ((p = strstr(line, "blk ")) != NULL) {
            unsigned bn; char hx[40];
            if (sscanf(p, "blk %u = %32s", &bn, hx) == 2 && bn < MFC_BLOCKS) {
                if (!strcmp(hx, "locked")) {
                    seen_blk[bn] = 1;
                } else if (fixed_hex(hx, 32)) {
                    seen_blk[bn] = 1;
                    for (int i = 0; i < 16; i++) { unsigned v; sscanf(hx + i * 2, "%2x", &v); blk[bn][i] = (uint8_t)v; }
                    ok[bn] = 1;
                }
            }
        } else if ((p = strstr(line, "prng=")) != NULL) {
            sscanf(p, "prng=%7s", prng);                   /* "weak" | "hard" from the device profiler */
        } else if (sscanf(line, "mfc: done %d sectors", &done_sectors) == 1) {
            /* The module emits this only after all key searches and all sector
             * status records. Validate the record counts below before rendering. */
        } else if (sscanf(line, "mfc: done block %d", &done_block) == 1) {
            /* Single-block completion marker; validated against -b below. */
        } else {
            size_t ll = strlen(line);
            while (ll && (line[ll - 1] == '\r' || line[ll - 1] == '\n')) line[--ll] = 0;
            if (ll && strncmp(line, "reading:", 8) != 0) {
                snprintf(failure, sizeof failure, "%s", line);
                /* UID, progress, and data records were consumed above. Any
                 * other MFC diagnostic except the final count is fatal. */
                if (!strncmp(line, "mfc:", 4) && strncmp(line, "mfc: done ", 10)) fatal = 1;
            }
        }
        line = nl ? nl + 1 : NULL;
    }
    int nkeys = 0, nblocks = 0;
    for (int s = 0; s < MFC_SECTORS; s++)
        nkeys += !!keyA[s][0] + !!keyB[s][0];
    for (int b = 0; b < MFC_BLOCKS; b++) nblocks += !!seen_blk[b];
    if (!got_uid || fatal) { printf("\n%s\n", failure[0] ? failure : "read: no card"); return; }

    if (a->block >= 0) {
        int sector = a->block / 4;
        int have_a = keyA[sector][0] != 0, have_b = keyB[sector][0] != 0;
        if (done_block != a->block || nblocks != 1 || !seen_blk[a->block] ||
            !ok[a->block] || have_a + have_b != 1) {
            printf("\nread: incomplete MIFARE Classic block result\n");
            return;
        }
        char keytype = have_b ? 'B' : 'A';
        const char *used_key = have_b ? keyB[sector] : keyA[sector];
        render_mfc_block(uid, sak, atqa, a->block, blk[a->block],
                         keytype, used_key, prng);

        if (a->have_save) {
            char bh[MFC_BLOCKS][40];
            for (int b = 0; b < MFC_BLOCKS; b++) bh[b][0] = 0;
            uint8_t row[16]; memcpy(row, blk[a->block], sizeof row);
            if ((a->block & 3) == 3) {
                int off = keytype == 'B' ? 10 : 0;
                for (int i = 0; i < 6; i++) {
                    unsigned v = 0; sscanf(used_key + i * 2, "%2x", &v); row[off + i] = (uint8_t)v;
                }
            }
            for (int i = 0; i < 16; i++) sprintf(bh[a->block] + i * 2, "%02X", row[i]);
            char path[300];
            if (a->save[0]) snprintf(path, sizeof path, "%s", a->save);
            else snprintf(path, sizeof path, "hf-mfc-%02X%02X%02X%02X-block-%d.json",
                          uid[0], uid[1], uid[2], uid[3], a->block);
            write_dump_json(RFID_REG[a->ri].token, bh, MFC_BLOCKS, path, prng);
        }
        return;
    }

    if (done_sectors != MFC_SECTORS || nkeys != MFC_SECTORS * 2 || nblocks != MFC_BLOCKS) {
        printf("\nread: incomplete MIFARE Classic result (%d/32 keys, %d/64 block statuses)\n",
               nkeys, nblocks);
        return;
    }
    render_mfc(uid, sak, atqa, blk, ok, keyA, keyB, prng);

    if (a->have_save) {                                    /* JSON: 16-byte blocks; splice recovered keys into
                                                            * each sector trailer (KeyA reads back masked). */
        char bh[MFC_BLOCKS][40];
        for (int b = 0; b < MFC_BLOCKS; b++) {
            if (!ok[b]) { bh[b][0] = 0; continue; }
            uint8_t row[16]; memcpy(row, blk[b], 16);
            if ((b & 3) == 3) {                            /* sector trailer (4-block sectors): restore keys */
                int s = b / 4;
                if (s < MFC_SECTORS && keyA[s][0]) for (int i = 0; i < 6; i++) { unsigned v; sscanf(keyA[s] + i * 2, "%2x", &v); row[i]      = (uint8_t)v; }
                if (s < MFC_SECTORS && keyB[s][0]) for (int i = 0; i < 6; i++) { unsigned v; sscanf(keyB[s] + i * 2, "%2x", &v); row[10 + i] = (uint8_t)v; }
            }
            for (int i = 0; i < 16; i++) sprintf(bh[b] + i * 2, "%02X", row[i]);
        }
        char path[300];
        if (a->save[0]) snprintf(path, sizeof path, "%s", a->save);
        else snprintf(path, sizeof path, "hf-mfc-%02X%02X%02X%02X-dump.json", uid[0], uid[1], uid[2], uid[3]);
        write_dump_json(RFID_REG[a->ri].token, bh, MFC_BLOCKS, path, prng);  /* PRNG class profiled during the read */
    }
}

/* Dispatch one rfid> command line: host abstractions (read/write/emulate/collect/list) -> device raw, or
 * host-rendered (help/list). Shared by the interactive prompt AND the non-interactive `-c "rfid <cmd>"`
 * one-shot. Sets g_rfid_ready when the command was answered host-side (no device prompt is coming). Trims
 * `line` in place; does not free it. */
static void rfid_dispatch_line(uint32_t sid, char *line)
{
    char *t = line; while (*t == ' ') t++;                 /* trim for exact match */
    size_t tl = strlen(t); while (tl && t[tl-1] == ' ') t[--tl] = '\0';
    /* `<verb> -h` / `<verb> --help` -> per-command help (pm3 muscle memory). Only when the flag is the
     * sole argument right after the verb; anything more is a real command and falls through. */
    char *sp = strchr(t, ' ');
    if (sp) {
        char *a = sp; while (*a == ' ') a++;
        if (!strcmp(a, "-h") || !strcmp(a, "--help")) {
            char vb[32]; size_t vn = (size_t)(sp - t); if (vn >= sizeof vb) vn = sizeof vb - 1;
            memcpy(vb, t, vn); vb[vn] = 0;
            rfid_help_cmd(vb);
            g_rfid_ready = 1;
            return;
        }
    }
    if (!strcmp(t, "help") || !strcmp(t, "?")) {
        rfid_help();                                       /* host-rendered; not sent to the device */
        g_rfid_ready = 1;
    } else if (!strncmp(t, "help ", 5)) {
        rfid_help_cmd(t + 5);                              /* `help <command>` - per-command detail */
        g_rfid_ready = 1;
    } else if (!strcmp(t, "list") || !strncmp(t, "list ", 5)) {
        rfid_list(t[4] ? t + 5 : NULL);
        g_rfid_ready = 1;
    } else if (!strcmp(t, "write") || !strncmp(t, "write ", 6)) {
        char rew[256];
        if (rfid_write(t[5] == ' ' ? t + 6 : t + 5, rew, sizeof rew)) send_dev_line(rew);
        else g_rfid_ready = 1;
    } else if (!strcmp(t, "read") || !strncmp(t, "read ", 5)) {
        char rew[256]; rw_args_t ra;
        int rr = rfid_read(t[4] == ' ' ? t + 5 : t + 4, rew, sizeof rew, &ra);
        if (rr == 1) send_dev_line(rew);
        else if (rr == 2) { rfid_read_all(sid, &ra); g_rfid_ready = 1; }
        else if (rr == 3) { rfid_read_mfc(sid, &ra); g_rfid_ready = 1; }
        else g_rfid_ready = 1;
    } else if (!strcmp(t, "collect") || !strncmp(t, "collect ", 8)) {
        if (rfid_collect(sid, t[7] == ' ' ? t + 8 : t + 7) != 1) g_rfid_ready = 1;
    } else if (!strcmp(t, "emulate") || !strncmp(t, "emulate ", 8)) {
        if (rfid_emulate(sid, t[7] == ' ' ? t + 8 : t + 7) != 1) g_rfid_ready = 1;
    } else if (bare_verb_needs_arg(t)) {
        printf("usage: %s %s\n", t, bare_verb_needs_arg(t));
        g_rfid_ready = 1;
    } else {
        char rew[256];                                     /* resolve protocol -> annotator + device cmd */
        int rr = rfid_resolve(line, rew, sizeof rew);
        if (rr == 2) {
            g_rfid_ready = 1;
        } else {
            const char *snd = (rr == 1) ? rew : line;
            size_t ll = strlen(snd);
            char *sn = malloc(ll + 2);
            if (sn) { memcpy(sn, snd, ll); sn[ll] = '\r'; sn[ll + 1] = 0;
                      send_input((const uint8_t *)sn, ll + 1); free(sn); }
        }
    }
}

void proto_cmd_rfid(const char *arg)
{
    /* Non-interactive one-shot: `fantasi --usb -c "rfid <cmd...>"` runs <cmd> (e.g. `emulate <json>`) without a
     * pty - dispatched host-side exactly like a typed line, then output streams until the app ends (a streaming
     * cmd runs until stopped via a second channel's `kill`/^C; a host-answered cmd exits right after). */
    char *oneshot = (arg && *arg) ? strdup(arg) : NULL;
    int oneshot_sent = 0;
    g_rfid_sigstop = 0;
    void (*old_sigint)(int) = signal(SIGINT, rfid_on_sigstop);
    void (*old_sigterm)(int) = signal(SIGTERM, rfid_on_sigstop);
    rx_len = 0;
    detect_arch();
    if (upload_ram(DRIVER_RAM, elf_path("rfid"), 1) < 0) {
        fprintf(stderr, "rfid: driver upload failed\n");
        delete_ram(MFC_CFG_RAM); delete_ram(MFC_EMU_RAM);
        signal(SIGINT, old_sigint); signal(SIGTERM, old_sigterm); free(oneshot);
        return;
    }

    CliRequest req = CliRequest_init_zero;
    req.id = ++rq_id; req.which_payload = CliRequest_app_launch_tag;
    strncpy(req.payload.app_launch, DRIVER_RAM, sizeof req.payload.app_launch - 1);
    uint32_t sid = req.id;
    if (send_req(&req) < 0) {
        fprintf(stderr, "rfid: launch failed\n");
        delete_ram(DRIVER_RAM);
        delete_ram(MFC_CFG_RAM); delete_ram(MFC_EMU_RAM);
        signal(SIGINT, old_sigint); signal(SIGTERM, old_sigterm); free(oneshot);
        return;
    }

    /* The device app does NO line editing or echo - the HOST owns the rfid> line with readline (left/
     * right, history, TAB completion, ^D), just like the main prompt. The app prints "rfid> " when it
     * wants a line; sniff_emit catches that into g_rfid_ready and hides it, then readline draws its own
     * prompt and reads the line, which we forward. Between the line and the next prompt a command may be
     * running (e.g. sniff streaming frames), so there we drop to a raw terminal and forward keystrokes -
     * any key still stops sniff, ^C aborts. Non-interactive (piped/serial) skips readline and just
     * forwards stdin, as before. */
    int interactive = !oneshot && isatty(0);   /* a `-c "rfid <cmd>"` one-shot is never interactive */
    struct termios cook_tio, raw_tio; int have_tio = 0;
    if (interactive && tcgetattr(0, &cook_tio) == 0) {
        raw_tio = cook_tio; cfmakeraw(&raw_tio);
        raw_tio.c_cc[VMIN] = 0; raw_tio.c_cc[VTIME] = 0;
        have_tio = 1;
    }

    rl_completion_func_t *save_comp = NULL;
    rl_hook_func_t       *save_hook = NULL;
    HISTORY_STATE        *save_hist = NULL;
    char rfid_hist[512] = "";                          /* the rfid> app's own persistent history file */
    if (interactive) {
        save_comp = rl_attempted_completion_function; rl_attempted_completion_function = rfid_completion;
        save_hook = rl_event_hook;                    /* The main CLI's transport hook must not run against
                                                       * this app loop, but the session still needs a lease
                                                       * heartbeat while readline waits indefinitely. */
        rl_event_hook = rfid_readline_heartbeat;
        save_hist = history_get_history_state();       /* the rfid> history is its own, restored on exit */
        HISTORY_STATE empty = {0};
        history_set_history_state(&empty);
        if (!g_no_history) {                           /* load the rfid> history so up/down recalls it */
            fantasi_state_path("fantasi.rfid.log", rfid_hist, sizeof rfid_hist);
            if (rfid_hist[0]) { read_history(rfid_hist); stifle_history(1000); }
        }
    }

    g_rfid_ready = 0;
    int done = 0, stdin_done = 0, stop_sent = 0;
    int cmd_running = 0;                                /* a readline-dispatched command is streaming */
    time_t stop_at = 0;                                 /* when app_stop was sent (bounded-drain deadline) */
    while (!done) {
        for (;;) {                                     /* drain everything the app emitted (frames, results, prompt) */
            CliResponse resp;
            int r = recv_resp(&resp);
            if (r < 0) { done = 1; break; }
            if (r == 0) break;
            if (r == 1 && resp.id == sid) {
                if (resp.which_payload == CliResponse_output_tag && resp.payload.output[0] && !g_rfid_sigstop)
                    sniff_emit(resp.payload.output);
                else if (resp.which_payload == CliResponse_module_request_tag) serve_module(resp.payload.module_request);
                else if (resp.which_payload == CliResponse_error_tag) { fprintf(stderr, "\nrfid: %s\n", resp.payload.error.message); done = 1; }
                if (!resp.has_next) done = 1;
            }
            if (done) break;
        }
        if (done) break;

        // Bounded command timeout
        if (stop_sent) {
            if (!stop_at) stop_at = time(NULL);
            else if (time(NULL) - stop_at >= 3) break;
        }

        if (oneshot && !oneshot_sent && g_rfid_ready) {   /* -c one-shot: run the command once the app is ready */
            g_rfid_ready = 0;
            rfid_dispatch_line(sid, oneshot);
            oneshot_sent = 1;
            if (g_rfid_ready) {
                /* A normal host-answered command leaves the driver idle at its
                 * prompt, so `exit` lets it unwind cooperatively. A signal can
                 * instead break app_capture() while its device module is still
                 * running; an input line would sit unread behind that module.
                 * Send the asynchronous stop request in that case so RFID and
                 * module memory are released deterministically. */
                if (g_rfid_sigstop) {
                    CliRequest s = CliRequest_init_zero;
                    s.id = ++rq_id; s.which_payload = CliRequest_app_stop_tag;
                    s.payload.app_stop = true;
                    send_req(&s);
                } else {
                    send_input((const uint8_t *)"exit\r", 5);
                }
                stop_sent = 1; stdin_done = 1;             /* don't let the check below app_stop it again */
            }
            continue;                                      /* streaming cmd (emulate/sniff): drain until it ends */
        }
        // The app's next prompt, not output silence, completes a forwarded one-shot.
        if (oneshot && oneshot_sent && !stop_sent && g_rfid_ready) {
            g_rfid_ready = 0;
            CliRequest s = CliRequest_init_zero;
            s.id = ++rq_id; s.which_payload = CliRequest_app_stop_tag; s.payload.app_stop = true;
            send_req(&s);
            stop_sent = 1; stdin_done = 1;
            continue;
        }
        if (g_rfid_sigstop && !stop_sent) {               /* SIGINT/SIGTERM: stop the running app cleanly. Gate on
                                                           * stop_sent, NOT stdin_done: a `-c` one-shot has stdin at
                                                           * /dev/null, so stdin_done goes 1 immediately and would
                                                           * suppress the stop send entirely (the stop-key bug). */
            CliRequest s = CliRequest_init_zero;
            s.id = ++rq_id; s.which_payload = CliRequest_app_stop_tag; s.payload.app_stop = true;
            send_req(&s);
            stop_sent = 1; stdin_done = 1;                 /* then drain until the device confirms the app ended */
        }

        if (g_rfid_ready && interactive && !stdin_done) {
            g_rfid_ready = 0;
            cmd_running = 0;                                   /* back at the prompt: no command is streaming */
            if (have_tio) tcsetattr(0, TCSANOW, &cook_tio);   /* readline needs a cooked base (OPOST on) */
            char *line = readline("rfid> ");
            if (!line) {                                       /* ^D / EOF -> exit the app cleanly */
                send_input((const uint8_t *)"exit\r", 5);
                stdin_done = 1;
            } else {
                if (*line) { add_history(line); if (rfid_hist[0]) write_history(rfid_hist); }
                rfid_dispatch_line(sid, line);
                cmd_running = !g_rfid_ready;               /* host-answered -> ready, loop to readline; else streaming */
                free(line);
            }
            if (have_tio) tcsetattr(0, TCSANOW, &raw_tio);     /* raw while the command runs (forward keys) */
        } else if (stdin_done) {
            usleep(10000);                                      /* input at EOF: just wait for the app to finish */
        } else if (interactive && !cmd_running) {
            usleep(10000);                                      /* awaiting a prompt (not running): don't read stdin,
                                                                 * let the typed line reach readline */
        } else {
            /* a command is running (interactive) or we're piped: forward stdin so a key stops sniff, ^C aborts.
             * poll blocks up to 10 ms, so it also paces the drain loop without spinning. */
            struct pollfd p = { .fd = 0, .events = POLLIN };
            if (poll(&p, 1, 10) > 0 && (p.revents & POLLIN)) {
                uint8_t in[64];
                ssize_t n = read(0, in, sizeof in);
                if (n == 0) stdin_done = 1;
                else for (ssize_t i = 0; i < n; i++) {
                    if (in[i] == 0x03) {                        /* ^C -> abort the app */
                        CliRequest s = CliRequest_init_zero;
                        s.id = ++rq_id; s.which_payload = CliRequest_app_stop_tag; s.payload.app_stop = true;
                        send_req(&s);
                    } else send_input(&in[i], 1);
                }
            }
        }
    }

    if (interactive) {
        rl_attempted_completion_function = save_comp;
        rl_event_hook = save_hook;
        if (save_hist) { clear_history(); history_set_history_state(save_hist); free(save_hist); }
    }
    if (have_tio) tcsetattr(0, TCSANOW, &cook_tio);
    signal(SIGINT, old_sigint);
    signal(SIGTERM, old_sigterm);
    free(oneshot);
    delete_ram(DRIVER_RAM);                                  /* nothing persists */
    for (int i = 0; i < NMODS; i++) delete_ram(RFID_MODS[i].ram);
    delete_ram(MFC_CFG_RAM);
    delete_ram(MFC_EMU_RAM);
    printf("\n");
}
#endif /* HAS_PROTO */

LOCAL_COMMAND_BLE("rfid", "launch RFID app", cmd_rfid, proto_cmd_rfid);
