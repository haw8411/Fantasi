/* Fantasi CLI - host-side shell wrapping the device's serial CLI + its USB
 * storage. The device exposes a "Fantasi" FAT drive over MSC; local file
 * commands (ls, cd, cat, upload, rm, mkdir) operate on it via the OS mount
 * (udisksctl), and over BLE they use the protobuf transport. Serial commands
 * are passed through to the device unchanged. */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/fs.h>   /* BLKFLSBUF */
#include <scsi/sg.h>    /* SG_IO: read the device generation sector, cache-bypassing */
#include <termios.h>
#include <errno.h>
#include <glob.h>
#include <sys/stat.h>
#include <dirent.h>
#include <signal.h>
#include <poll.h>
#include <time.h>
#include <readline/readline.h>
#include <readline/history.h>

#ifdef HAS_PROTO
#include "ble_transport.h"
#include "fantasi.pb.h"
#include <pb_encode.h>
#include <pb_decode.h>
#endif

#ifdef HAS_USB_VENDOR
#include "usb_transport.h"
#endif

#include "cli_internal.h"
#include "theme.h"

/* When non-NULL, device command output is appended here instead of printed, so the
 * client can run a command silently and read its result (used for the saved theme
 * at startup). Both output paths (serial framed + BLE/USB protobuf) honor it. */
bool g_no_history;                           /* -c one-shot mode: skip reading/persisting history */

void fantasi_state_path(const char *name, char *out, size_t len)
{
    const char *home = getenv("HOME");
    if (!home || !*home) { if (len) out[0] = '\0'; return; }
    char dir[400];
    snprintf(dir, sizeof dir, "%s/.fantasi", home);
    mkdir(dir, 0700);                         /* ignore EEXIST */
    snprintf(out, len, "%s/%s", dir, name);
}

static char  *g_cap;
static size_t g_cap_sz, g_cap_len;
static void cap_putc(char c) { if (g_cap && g_cap_len + 1 < g_cap_sz) g_cap[g_cap_len++] = c; }
static void cap_puts(const char *s) { while (*s) cap_putc(*s++); }

/* Set when the protobuf transport is the USB vendor (WebUSB) pipe rather than
 * BLE. Declared unconditionally so the shared proto-path checks compile even in
 * builds without USB/BLE. */
bool use_usb;
bool g_switch_mode;   /* PM3 (switch-mode): pace uploads, no pipelining (SAM7S dual-bank OUT) */

/* ---- FAT mount state (the device's "Fantasi" MSC drive, mounted by the OS) ---- */

char cwd[256] = "/";   /* device-side path, e.g. "/", "/apps", "/ramfs" */
static char g_mnt[256];       /* OS mountpoint of the Fantasi FAT, "" if unmounted */
static char g_blk[64];        /* block device currently mounted */
static bool g_switched;       /* sent `msc` to enter MSC mode (switch-mode PM3) */

/* fat_mount / fat_unmount / fat_path are declared in cli_internal.h and defined
 * below (after the serial/block discovery helpers). */

/* ---- Persistent serial path (for reconnect after MSC) ---- */

static char g_ser_path[64];

/* Restrict device selection to this device name (hal_device_name()),
 * set by --name. The CDC scanners below match it against the USB device's iSerial
 * (sysfs `serial`); the USB-vendor and BLE transports filter on it too. Empty = any. */
static char g_device_name[64];

/* True if a sysfs USB device dir (/sys/bus/usb/devices/X) matches the requested
 * --name filter: no filter set, or its `serial` (iSerialNumber, = hal_device_name())
 * equals g_device_name. Used to disambiguate multiple connected Fantasi devices. */
static bool usb_dev_name_matches(const char *usb_dev_path)
{
    if (!g_device_name[0]) return true;
    char spath[512];
    snprintf(spath, sizeof(spath), "%s/serial", usb_dev_path);
    FILE *fs = fopen(spath, "r");
    if (!fs) return false;
    char serial[64] = "";
    if (!fgets(serial, sizeof(serial), fs)) { fclose(fs); return false; }
    fclose(fs);
    serial[strcspn(serial, "\n")] = '\0';
    return strcmp(serial, g_device_name) == 0;
}

/* ---- Serial port ---- */

int ser_fd = -1;

static bool ser_open(const char *path)
{
    ser_fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (ser_fd < 0) {
        fprintf(stderr, "cannot open %s: %s\n", path, strerror(errno));
        return false;
    }
    /* Clear O_NONBLOCK - we only needed it for open() to avoid blocking
     * on carrier detect. Reads use poll(); writes must not silently drop. */
    int flags = fcntl(ser_fd, F_GETFL, 0);
    fcntl(ser_fd, F_SETFL, flags & ~O_NONBLOCK);

    struct termios t;
    tcgetattr(ser_fd, &t);
    cfmakeraw(&t);
    cfsetspeed(&t, B115200);
    t.c_cflag |= CLOCAL | CREAD;
    tcsetattr(ser_fd, TCSANOW, &t);
    tcflush(ser_fd, TCIOFLUSH);
    return true;
}

static bool ser_connected;
bool msc_active;
/* True when the last byte written to stdout was not a newline, i.e. command output
 * left the cursor mid-line. -c mode adds a single trailing newline only then, so
 * output that already ends in '\n' doesn't get a spurious blank line. */
static bool g_out_pending_nl;
static void ser_drain(void);
static bool wait_for_fantasi_serial(char *out, size_t len, int timeout_s);

static void ser_close(void)
{
    if (ser_fd >= 0) {
        close(ser_fd);
        ser_fd = -1;
    }
}

static void ser_mark_disconnected(void)
{
    ser_close();
    ser_connected = false;
}

static bool ser_try_reconnect(void)
{
    char path[64];
    if (!wait_for_fantasi_serial(path, sizeof(path), 0))
        return false;
    usleep(500000);
    if (!ser_open(path))
        return false;
    snprintf(g_ser_path, sizeof(g_ser_path), "%s", path);
    ser_connected = true;
    usleep(300000);
    ser_drain();
    return true;
}

static void ser_drain(void)
{
    struct pollfd pfd = { .fd = ser_fd, .events = POLLIN };
    char d[512];
    while (poll(&pfd, 1, 50) > 0 && (pfd.revents & POLLIN))
        if (read(ser_fd, d, sizeof(d)) <= 0) break;
}

static void ser_write(const char *s, size_t len)
{
    while (len > 0) {
        ssize_t n = write(ser_fd, s, len);
        if (n <= 0) {
            if (!msc_active) ser_mark_disconnected();
            return;
        }
        s += n; len -= n;
    }
}

void ser_send_cmd(const char *cmd)
{
    ser_drain();
    ser_write(cmd, strlen(cmd));
    ser_write("\r\n", 2);
}

/* Framing bytes (FRAME_SENTINEL / FRAME_START / FRAME_END) are defined in
 * cli_internal.h: firmware sends \x06\x01 before command output and \x06\x02
 * when the command is done. */

static void ser_emit_framed(const uint8_t *buf, ssize_t n, uint8_t *prev,
                            bool *done)
{
    for (ssize_t i = 0; i < n; i++) {
        uint8_t c = buf[i];
        if (*prev == FRAME_SENTINEL && c == FRAME_END) { *done = true; return; }
        if (*prev == FRAME_SENTINEL && c != FRAME_END)
            ; /* drop stray sentinel */
        else if (c == FRAME_SENTINEL)
            ; /* hold - check next byte */
        else if (c != '\r') {
            if (g_cap) cap_putc((char)c);
            else     { putchar(c); g_out_pending_nl = (c != '\n'); }
        }
        *prev = c;
    }
    fflush(stdout);
}

static void ser_read_response(void)
{
    /* Phase 1: eat echo + wait for \x06\x01 (command started). */
    uint8_t prev = 0;
    uint8_t carry[256];
    ssize_t carry_len = 0;

    for (;;) {
        struct pollfd pfd = { .fd = ser_fd, .events = POLLIN };
        int r = poll(&pfd, 1, 5000);
        if (r <= 0) return;
        if (pfd.revents & (POLLERR | POLLHUP)) {
            if (!msc_active) ser_mark_disconnected();
            return;
        }

        uint8_t chunk[256];
        ssize_t n = read(ser_fd, (char *)chunk, sizeof(chunk));
        if (n <= 0) {
            if (!msc_active) ser_mark_disconnected();
            return;
        }

        bool found = false;
        for (ssize_t i = 0; i < n; i++) {
            uint8_t c = chunk[i];
            if (prev == FRAME_SENTINEL && c == FRAME_START) {
                carry_len = n - i - 1;
                if (carry_len > 0) memcpy(carry, &chunk[i + 1], carry_len);
                found = true;
                break;
            }
            prev = c;
        }
        if (found) break;
    }

    /* Phase 2: passthrough until \x06\x02 (command done) or Ctrl-C. */
    struct termios old_tio, raw_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    raw_tio = old_tio;
    raw_tio.c_lflag &= ~(ICANON | ECHO | ISIG);
    raw_tio.c_cc[VMIN] = 0;
    raw_tio.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw_tio);

    /* Process leftover bytes from phase 1 */
    prev = 0;
    bool finished = false;
    if (carry_len > 0)
        ser_emit_framed(carry, carry_len, &prev, &finished);

    while (!finished) {
        struct pollfd pfds[2] = {
            { .fd = STDIN_FILENO, .events = POLLIN },
            { .fd = ser_fd,       .events = POLLIN },
        };
        int r = poll(pfds, 2, 500);

        if (ser_fd < 0) break;
        if (r < 0 && errno == EINTR) continue;

        if (pfds[1].revents & (POLLERR | POLLHUP)) {
            /* While MSC is mounted (composite), CDC can briefly hiccup as the
             * USB task services storage; don't tear down the link on it (matches
             * phase 1) - only a genuine disconnect (MSC inactive) is fatal. */
            if (!msc_active) ser_mark_disconnected();
            break;
        }

        if (pfds[0].revents & POLLIN) {
            char in[64];
            ssize_t n = read(STDIN_FILENO, in, sizeof(in));
            if (n > 0) {
                ser_write(in, n);
                for (ssize_t i = 0; i < n; i++)
                    if (in[i] == 0x03) goto done;
            }
        }

        if (pfds[1].revents & POLLIN) {
            uint8_t chunk[256];
            ssize_t n = read(ser_fd, (char *)chunk, sizeof(chunk));
            if (n <= 0) { if (!msc_active) ser_mark_disconnected(); break; }
            ser_emit_framed(chunk, n, &prev, &finished);
        }
    }

done:
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
}

/* ---- Resolve path against cwd ---- */

/* Collapse `.`, `..`, and duplicate/trailing slashes in an absolute path, in
 * place. Without this, `cd ..` in /ramfs produced "/ramfs/.." (the device has no
 * notion of "..") and validation failed. Every command routes through
 * resolve_path, so normalizing here fixes navigation everywhere. */
static void normalize_path(char *path)
{
    char *segs[64];
    int   nseg = 0;
    for (char *p = path; *p; ) {
        while (*p == '/') p++;                 /* skip run of slashes */
        if (!*p) break;
        char *s = p;
        while (*p && *p != '/') p++;
        if (*p) { *p = '\0'; p++; }
        if (strcmp(s, ".") == 0) continue;
        if (strcmp(s, "..") == 0) { if (nseg > 0) nseg--; continue; }
        if (nseg < 64) segs[nseg++] = s;       /* segs point into `path` */
    }
    char tmp[256];
    size_t w = 0;
    for (int i = 0; i < nseg; i++) {
        size_t l = strlen(segs[i]);
        if (w + 1 + l >= sizeof tmp) break;
        tmp[w++] = '/';
        memcpy(tmp + w, segs[i], l);
        w += l;
    }
    if (w == 0) tmp[w++] = '/';                /* root */
    tmp[w] = '\0';
    memcpy(path, tmp, w + 1);
}

void resolve_path(const char *arg, char *out, size_t len)
{
    if (!arg || !arg[0]) {
        strncpy(out, cwd, len);
    } else if (arg[0] == '/') {
        strncpy(out, arg, len);
    } else {
        size_t cwdlen = strlen(cwd);
        if (cwdlen == 1)
            snprintf(out, len, "/%s", arg);
        else
            snprintf(out, len, "%s/%s", cwd, arg);
    }
    out[len - 1] = '\0';
    normalize_path(out);
}

/* If the destination argument denotes a directory, append basename(src) to the
 * already-resolved destination so `cp/mv SRC DIR` lands at DIR/<basename> - matching
 * shell semantics. A directory is indicated by a trailing '/', or a '.'/'..' final
 * component: ".", "..", ".../", ".../.", or ".../..". (resolve_path has already collapsed
 * these in dst_resolved, so we read the intent from the raw argument.) `dst_resolved`
 * is normalized: "/" for root, otherwise no trailing slash. No-op for a plain filename. */
void dir_target(const char *src_resolved, const char *dst_raw,
                char *dst_resolved, size_t cap)
{
    size_t rl = strlen(dst_raw);
    bool is_dir = rl > 0 && dst_raw[rl - 1] == '/';
    if (!is_dir && (strcmp(dst_raw, ".") == 0 || strcmp(dst_raw, "..") == 0))
        is_dir = true;
    if (!is_dir && rl >= 2 && dst_raw[rl - 1] == '.' && dst_raw[rl - 2] == '/')
        is_dir = true;                                        /* ".../." */
    if (!is_dir && rl >= 3 && dst_raw[rl - 1] == '.' && dst_raw[rl - 2] == '.'
                           && dst_raw[rl - 3] == '/')
        is_dir = true;                                        /* ".../.." */
    if (!is_dir) return;

    const char *base = strrchr(src_resolved, '/');
    base = base ? base + 1 : src_resolved;
    size_t l = strlen(dst_resolved);
    if (l == 1 && dst_resolved[0] == '/')                     /* root: "/" + x -> "/x" */
        snprintf(dst_resolved + l, cap - l, "%s", base);
    else                                                      /* "/dir" + x -> "/dir/x" */
        snprintf(dst_resolved + l, cap - l, "/%s", base);
}

/* ---- Local commands (operate on the OS-mounted "Fantasi" FAT) ----
 *
 * fat_mount() puts the device in MSC mode (switch-mode on PM3), waits for the
 * kernel to expose the Fantasi block device, and mounts it via udisksctl.
 * fat_path() maps a device path ("/apps/x") to the host mountpoint
 * ("<g_mnt>/apps/x"); the file ops are then plain stdio against that path.
 * Synthetic-FAT data is captured in bounded RAM staging and committed at SCSI
 * SYNCHRONIZE CACHE boundaries. Local mutators therefore flush the host mount
 * and issue that command explicitly before reporting completion or ejecting. */

const char *fat_path(const char *vpath)
{
    static char host[512];
    /* vpath is already absolute ("/...") or empty (root). */
    if (!vpath || !vpath[0] || strcmp(vpath, "/") == 0)
        snprintf(host, sizeof(host), "%s", g_mnt);
    else
        snprintf(host, sizeof(host), "%s%s", g_mnt, vpath);
    return host;
}

static bool fat_scsi_sync(const char *blk)
{
    int fd = open(blk, O_RDWR | O_NONBLOCK);
    if (fd < 0) return false;

    unsigned char cdb[10] = { 0x35 };       /* SYNCHRONIZE CACHE(10) */
    unsigned char sense[32] = { 0 };
    sg_io_hdr_t io;
    memset(&io, 0, sizeof io);
    io.interface_id = 'S';
    io.dxfer_direction = SG_DXFER_NONE;
    io.cmdp = cdb;
    io.cmd_len = sizeof cdb;
    io.sbp = sense;
    io.mx_sb_len = sizeof sense;
    io.timeout = 5000;

    int rc = ioctl(fd, SG_IO, &io);
    bool ok = rc == 0 && (io.info & SG_INFO_OK_MASK) == SG_INFO_OK;
    int sync_errno = rc < 0 ? errno : EIO;
    close(fd);
    if (!ok) errno = sync_errno;
    return ok;
}

bool fat_sync(void)
{
    if (!g_mnt[0]) return true;
    /* Flush only this mounted filesystem. `sync()` dirties latency and error
     * reporting with every unrelated filesystem on the host, and Linux may
     * legitimately omit SYNCHRONIZE CACHE for this write-through MSC LUN. */
    int mfd = open(g_mnt, O_RDONLY | O_DIRECTORY);
    if (mfd < 0) {
        fprintf(stderr, "cannot open filesystem for sync: %s\n", strerror(errno));
        return false;
    }
    int src = syncfs(mfd);
    int sync_errno = errno;
    close(mfd);
    if (src < 0) {
        errno = sync_errno;
        fprintf(stderr, "host filesystem sync failed: %s\n", strerror(errno));
        return false;
    }

    /* Issue SYNCHRONIZE CACHE so the device commits staged MSC writes. */
    if (!fat_scsi_sync(g_blk)) {
        fprintf(stderr, "device filesystem sync failed: %s\n", strerror(errno));
        return false;
    }
    return true;
}

/* ---- Command dispatch ---- */

static const char *skip_word(const char *s)
{
    while (*s && *s != ' ' && *s != '\t') s++;
    while (*s == ' ' || *s == '\t') s++;
    return s;
}

/* Match the first word of `line` to a command registered in the local_cmd
 * section (see cli_internal.h / cli/commands/). */
const local_cmd_t *cli_local_match(const char *line)
{
    for (const local_cmd_t *c = __start_local_cmd; c < __stop_local_cmd; c++) {
        size_t len = strlen(c->name);
        if (strncmp(line, c->name, len) == 0 &&
            (line[len] == 0 || line[len] == ' '))
            return c;
    }
    return NULL;
}

#ifdef HAS_PROTO
bool     use_ble;       /* talking to the device over BLE rather than USB/MSC */
uint32_t proto_req_id;    /* monotonic protobuf request id */
uint32_t proto_session_id; /* device-owned session (zero on legacy firmware) */
static uint32_t proto_active_request;
static int proto_session_ensure(void);
static void proto_session_ping(void);
static void proto_session_close_client(void);
#endif

static bool handle_local(const char *line)
{
    const local_cmd_t *c = cli_local_match(line);
    if (!c) return true;
    if (c->fn == cmd_exit) return false;
    const char *arg = skip_word(line);
    if (!*arg) arg = NULL;

#ifdef HAS_PROTO
    if ((use_ble || use_usb) && c->proto_fn) {
        if (proto_session_ensure() < 0) return true;
        c->proto_fn(arg);
        return true;
    }
#endif

    c->fn(arg);
    return true;
}

/* ---- Auto-detect ---- */

static bool find_fantasi_device(char *ser_path, size_t ser_len,
                                char *blk_path, size_t blk_len)
{
    glob_t g;
    ser_path[0] = blk_path[0] = '\0';

    if (glob("/sys/bus/usb/devices/[0-9]*", 0, NULL, &g) != 0)
        return false;

    for (size_t i = 0; i < g.gl_pathc; i++) {
        char vpath[512], ppath[512];
        snprintf(vpath, sizeof(vpath), "%s/idVendor", g.gl_pathv[i]);
        snprintf(ppath, sizeof(ppath), "%s/idProduct", g.gl_pathv[i]);

        FILE *fv = fopen(vpath, "r"), *fp = fopen(ppath, "r");
        if (!fv || !fp) { if (fv) fclose(fv); if (fp) fclose(fp); continue; }

        char vid[8], pid[8];
        if (!fgets(vid, sizeof(vid), fv) || !fgets(pid, sizeof(pid), fp)) {
            fclose(fv); fclose(fp); continue;
        }
        fclose(fv); fclose(fp);
        vid[strcspn(vid, "\n")] = '\0';
        pid[strcspn(pid, "\n")] = '\0';

        if (strcmp(vid, "1209") != 0 || strcmp(pid, "0001") != 0) continue;
        if (!usb_dev_name_matches(g.gl_pathv[i])) continue;   /* --name filter */

        /* Found Fantasi device - scan for tty and block device */
        char pattern[512];
        snprintf(pattern, sizeof(pattern), "%s/*/tty/ttyACM*", g.gl_pathv[i]);
        glob_t tg;
        if (glob(pattern, 0, NULL, &tg) == 0 && tg.gl_pathc > 0) {
            const char *base = strrchr(tg.gl_pathv[0], '/');
            if (base) snprintf(ser_path, ser_len, "/dev%s", base);
            globfree(&tg);
        }

        snprintf(pattern, sizeof(pattern), "%s/*/host*/target*/*:*:*:*/block/sd*",
                 g.gl_pathv[i]);
        glob_t bg;
        if (glob(pattern, 0, NULL, &bg) == 0 && bg.gl_pathc > 0) {
            const char *base = strrchr(bg.gl_pathv[0], '/');
            if (base) snprintf(blk_path, blk_len, "/dev%s", base);
            globfree(&bg);
        }
        break;
    }
    globfree(&g);
    return ser_path[0] != '\0';
}

/* ---- MSC mode switching (for platforms like PM3 that lack composite CDC+MSC) ---- */

static bool wait_for_fantasi_block(char *out, size_t len, int timeout_s)
{
    /* timeout_s == 0: a single scan with no waiting (a pure presence probe). */
    int iterations = timeout_s * 4;
    if (iterations < 1) iterations = 1;
    for (int i = 0; i < iterations; i++) {
        glob_t g;
        if (glob("/sys/bus/usb/devices/[0-9]*", 0, NULL, &g) != 0) {
            usleep(250000);
            continue;
        }
        for (size_t j = 0; j < g.gl_pathc; j++) {
            char vpath[512], ppath[512];
            snprintf(vpath, sizeof(vpath), "%s/idVendor", g.gl_pathv[j]);
            snprintf(ppath, sizeof(ppath), "%s/idProduct", g.gl_pathv[j]);
            FILE *fv = fopen(vpath, "r"), *fp = fopen(ppath, "r");
            if (!fv || !fp) { if (fv) fclose(fv); if (fp) fclose(fp); continue; }
            char vid[8], pid[8];
            if (!fgets(vid, sizeof(vid), fv) || !fgets(pid, sizeof(pid), fp)) {
                fclose(fv); fclose(fp); continue;
            }
            fclose(fv); fclose(fp);
            vid[strcspn(vid, "\n")] = '\0';
            pid[strcspn(pid, "\n")] = '\0';
            if (strcmp(vid, "1209") != 0 || strcmp(pid, "0001") != 0) continue;
            if (!usb_dev_name_matches(g.gl_pathv[j])) continue;   /* --name filter */

            char pattern[512];
            snprintf(pattern, sizeof(pattern),
                     "%s/*/host*/target*/*:*:*:*/block/sd*", g.gl_pathv[j]);
            glob_t bg;
            if (glob(pattern, 0, NULL, &bg) == 0 && bg.gl_pathc > 0) {
                const char *base = strrchr(bg.gl_pathv[0], '/');
                if (base) {
                    snprintf(out, len, "/dev%s", base);
                    globfree(&bg);
                    globfree(&g);
                    return true;
                }
                globfree(&bg);
            }
        }
        globfree(&g);
        if (i + 1 < iterations) usleep(250000);
    }
    return false;
}

static bool wait_for_fantasi_serial(char *out, size_t len, int timeout_s)
{
    int iterations = timeout_s * 4;
    if (iterations < 1) iterations = 1;
    for (int i = 0; i < iterations; i++) {
        glob_t g;
        if (glob("/sys/bus/usb/devices/[0-9]*", 0, NULL, &g) != 0) {
            usleep(250000);
            continue;
        }
        for (size_t j = 0; j < g.gl_pathc; j++) {
            char vpath[512], ppath[512];
            snprintf(vpath, sizeof(vpath), "%s/idVendor", g.gl_pathv[j]);
            snprintf(ppath, sizeof(ppath), "%s/idProduct", g.gl_pathv[j]);
            FILE *fv = fopen(vpath, "r"), *fp = fopen(ppath, "r");
            if (!fv || !fp) { if (fv) fclose(fv); if (fp) fclose(fp); continue; }
            char vid[8], pid[8];
            if (!fgets(vid, sizeof(vid), fv) || !fgets(pid, sizeof(pid), fp)) {
                fclose(fv); fclose(fp); continue;
            }
            fclose(fv); fclose(fp);
            vid[strcspn(vid, "\n")] = '\0';
            pid[strcspn(pid, "\n")] = '\0';
            if (strcmp(vid, "1209") != 0 || strcmp(pid, "0001") != 0) continue;
            if (!usb_dev_name_matches(g.gl_pathv[j])) continue;   /* --name filter */

            char pattern[512];
            snprintf(pattern, sizeof(pattern), "%s/*/tty/ttyACM*", g.gl_pathv[j]);
            glob_t tg;
            if (glob(pattern, 0, NULL, &tg) == 0 && tg.gl_pathc > 0) {
                const char *base = strrchr(tg.gl_pathv[0], '/');
                if (base) {
                    snprintf(out, len, "/dev%s", base);
                    globfree(&tg);
                    globfree(&g);
                    return true;
                }
                globfree(&tg);
            }
        }
        globfree(&g);
        if (i + 1 < iterations) usleep(250000);
    }
    return false;
}

/* The device's external-write generation, read straight off the block device with
 * an SG_IO READ(10) of the reserved generation sector (see GEN_LBA in
 * fat_ramdisk.c). SG_IO bypasses the page cache, so this always reflects the
 * device's current state - the race-free way to tell whether the filesystem changed
 * under a mounted view since we last looked. Returns 0 if unavailable. */
#define GEN_LBA 3
static uint32_t fat_gen(const char *blk)
{
    int fd = open(blk, O_RDONLY | O_NONBLOCK);
    if (fd < 0) return 0;
    unsigned char cdb[10] = { 0x28, 0, 0, 0, 0, GEN_LBA, 0, 0, 1, 0 };  /* READ(10) lba=3 len=1 */
    unsigned char buf[512], sense[32];
    sg_io_hdr_t io;
    memset(&io, 0, sizeof io);
    io.interface_id = 'S'; io.dxfer_direction = SG_DXFER_FROM_DEV;
    io.cmdp = cdb; io.cmd_len = sizeof cdb; io.dxferp = buf; io.dxfer_len = sizeof buf;
    io.sbp = sense; io.mx_sb_len = sizeof sense; io.timeout = 5000;
    uint32_t gen = 0;
    if (ioctl(fd, SG_IO, &io) == 0 && (io.info & SG_INFO_OK_MASK) == SG_INFO_OK &&
        memcmp(buf, "FSgen", 5) == 0)
        gen = buf[8] | (buf[9] << 8) | (buf[10] << 16) | ((uint32_t)buf[11] << 24);
    close(fd);
    return gen;
}

/* The generation the currently-established OS mount reflects, persisted across our
 * per-command (`-c`) invocations since the mount itself persists. */
static uint32_t mount_gen_load(void)
{
    char path[512]; fantasi_state_path("mntgen", path, sizeof path);
    if (!path[0]) return 0;
    FILE *f = fopen(path, "r"); if (!f) return 0;
    unsigned long g = 0; if (fscanf(f, "%lu", &g) != 1) g = 0;
    fclose(f); return (uint32_t)g;
}
static void mount_gen_store(uint32_t gen)
{
    char path[512]; fantasi_state_path("mntgen", path, sizeof path);
    if (!path[0]) return;
    FILE *f = fopen(path, "w"); if (!f) return;
    fprintf(f, "%lu\n", (unsigned long)gen); fclose(f);
}

/* Current mountpoint of `blk` (empty if not mounted). */
static bool findmnt_target(const char *blk, char *out, size_t len)
{
    out[0] = '\0';
    char cmd[256]; snprintf(cmd, sizeof cmd, "findmnt -n -o TARGET %s 2>/dev/null", blk);
    FILE *p = popen(cmd, "r");
    if (p) { if (fgets(out, len, p)) out[strcspn(out, "\n")] = '\0'; pclose(p); }
    return out[0] != '\0';
}

/* Mount `blk` via udisksctl. `usefree` trusts the FAT's FSInfo free-cluster count
 * instead of scanning the whole FAT on statfs - on a card-sized drive that scan is
 * thousands of slow synthetic reads (~20s). Returns the mountpoint in `out`. */
static bool udisks_do_mount(const char *blk, char *out, size_t len)
{
    out[0] = '\0';
    char cmd[256]; snprintf(cmd, sizeof cmd, "udisksctl mount -b %s -o usefree 2>&1", blk);
    FILE *p = popen(cmd, "r");
    if (p) {
        char line[512];
        while (fgets(line, sizeof(line), p)) {
            char *at = strstr(line, " at ");   /* "Mounted <dev> at <path>" / "already mounted at ..." */
            if (!at) continue;
            char *m = at + 4;
            while (*m == '`' || *m == '\'' || *m == '"' || *m == ' ') m++;
            size_t k = 0;
            while (m[k] && m[k] != '`' && m[k] != '\'' && m[k] != '"' && m[k] != '\n') k++;
            if (k > 0 && m[k - 1] == '.') k--;   /* the "already mounted" form ends with '.' */
            m[k] = '\0';
            snprintf(out, len, "%s", m);
        }
        pclose(p);
    }
    return out[0] != '\0';
}

bool fat_mount(void)
{
    if (g_mnt[0]) return true;   /* already mounted */

    char blk[64];
    /* Composite devices (FZ/CU) expose the MSC block device alongside CDC, so
     * it is already present. Switch-mode devices (PM3) reuse the CDC endpoints
     * for MSC and must be told to switch with the `msc` command first. */
    if (!wait_for_fantasi_block(blk, sizeof(blk), 1)) {
        if (!ser_connected) { fprintf(stderr, "  not connected\n"); return false; }

        printf("  switching to MSC mode...\n");
        /* Send `msc` with retries. A CDC ACM enumeration quirk can inject a
         * stray byte that corrupts the first command; if the port doesn't go
         * away (device didn't switch), flush and retry. */
        for (int attempt = 0; attempt < 3; attempt++) {
            ser_write("\r\n", 2);
            usleep(150000);
            ser_drain();
            ser_write("msc\r\n", 5);

            struct pollfd pfd = { .fd = ser_fd, .events = POLLIN | POLLERR | POLLHUP };
            bool switched = false;
            for (int i = 0; i < 20; i++) {
                int r = poll(&pfd, 1, 200);
                if (r > 0 && (pfd.revents & (POLLERR | POLLHUP))) { switched = true; break; }
                if (r > 0 && (pfd.revents & POLLIN)) {
                    char d[256];
                    ssize_t n = read(ser_fd, d, sizeof(d));
                    if (n <= 0) { switched = true; break; }
                }
            }
            if (switched) break;
        }
        ser_close();
        ser_connected = false;
        g_switched = true;

        if (!wait_for_fantasi_block(blk, sizeof(blk), 10)) {
            fprintf(stderr, "  block device did not appear\n");
            if (wait_for_fantasi_serial(g_ser_path, sizeof(g_ser_path), 5))
                ser_open(g_ser_path);
            return false;
        }
    }

    snprintf(g_blk, sizeof(g_blk), "%s", blk);
    /* A switch-mode device's block node has just appeared - give udev a moment. A
     * composite device's node is always present, so no settle is needed there. */
    if (g_switched) usleep(500000);

    /* Freshness without churn: read the device's write-generation (cache-bypassing
     * SG_IO). If the drive is already mounted (our persistent mount) and its
     * generation still matches what that mount reflects, reuse it as-is - a mounted
     * FS only serves stale metadata if the FS changed under it, and the generation
     * says it hasn't (our own writes over MSC don't bump it). Otherwise (unmounted,
     * or another transport wrote) mount fresh, which is the only reliable way to
     * drop the OS's cached view without root. */
    uint32_t dev_gen = fat_gen(g_blk);
    char existing[256];
    if (findmnt_target(g_blk, existing, sizeof existing)) {
        if (dev_gen && dev_gen == mount_gen_load()) {
            snprintf(g_mnt, sizeof g_mnt, "%s", existing);   /* unchanged: reuse (fast) */
            msc_active = true;
            return true;
        }
        char cmd[256];                                       /* changed: remount fresh */
        snprintf(cmd, sizeof cmd, "udisksctl unmount -b %s >/dev/null 2>&1", g_blk);
        (void)system(cmd);
    }

    /* udev/udisks needs a moment to register a freshly-enumerated device, so the
     * first mount attempt often races and fails - retry for a few seconds. */
    for (int i = 0; i < 12; i++) {
        if (udisks_do_mount(g_blk, g_mnt, sizeof(g_mnt))) {
            mount_gen_store(dev_gen);   /* this mount now reflects dev_gen */
            msc_active = true;
            return true;
        }
        usleep(500000);
    }
    fprintf(stderr, "  mount failed: %s\n", blk);
    g_mnt[0] = '\0';
    return false;
}

/* Linux can return from a FAT unmount while asynchronous readahead submitted by
 * the final metadata invalidation is still draining through usb-storage. On a
 * switch-mode PM3, sending START STOP immediately would ask the firmware to drop
 * MSC underneath that READ10. Watch the block queue until both completions and
 * in-flight I/O have been quiet, with a finite fallback for unusual kernels. */
static void fat_wait_block_quiet(const char *blk)
{
    const char *name = strrchr(blk, '/');
    name = name ? name + 1 : blk;
    if (!name[0]) { usleep(300000); return; }

    char stat_path[160];
    snprintf(stat_path, sizeof stat_path, "/sys/class/block/%s/stat", name);
    uint64_t previous = UINT64_MAX;
    unsigned quiet_ms = 0;

    for (unsigned elapsed = 0; elapsed < 8000; elapsed += 50) {
        FILE *f = fopen(stat_path, "r");
        unsigned long long reads, read_merges, read_sectors, read_ms;
        unsigned long long writes, write_merges, write_sectors, write_ms, inflight;
        int fields = f ? fscanf(f, "%llu %llu %llu %llu %llu %llu %llu %llu %llu",
                                &reads, &read_merges, &read_sectors, &read_ms,
                                &writes, &write_merges, &write_sectors, &write_ms,
                                &inflight) : 0;
        if (f) fclose(f);
        if (fields != 9) { usleep(300000); return; }

        uint64_t completed = (uint64_t)reads + (uint64_t)writes;
        if (inflight == 0 && completed == previous) quiet_ms += 50;
        else                                        quiet_ms = 0;
        previous = completed;
        if (quiet_ms >= 300) return;
        usleep(50000);
    }
}

void fat_unmount(void)
{
    if (!g_mnt[0]) return;

    /* Commit any last metadata/data the command left dirty before the switch-mode
     * eject tears down MSC and releases the device-side staging model. Continue
     * with the eject even on failure so a constrained device can recover to CDC. */
    (void)fat_sync();

    /* Composite devices (FZ/CU) expose MSC and CDC at once, so there is nothing to
     * switch back to - leave the drive mounted (udisks manages it) instead of
     * churning an unmount/remount around every file command. Cross-transport
     * freshness is still guaranteed: the next fat_mount drops the block cache
     * (BLKFLSBUF) before reading, so a write made over another transport is seen.
     * Only switch-mode devices (PM3), which reuse the CDC endpoints for MSC, must
     * actually unmount + SCSI-eject to re-enumerate as CDC. */
    if (!g_switched) { g_mnt[0] = '\0'; return; }

    char cmd[256];
    snprintf(cmd, sizeof(cmd), "udisksctl unmount -b %s >/dev/null 2>&1", g_blk);
    (void)system(cmd);
    fat_wait_block_quiet(g_blk);
    g_mnt[0] = '\0';
    msc_active = false;

    {
        snprintf(cmd, sizeof(cmd), "eject %s 2>/dev/null", g_blk);
        (void)system(cmd);
        g_switched = false;

        fprintf(stderr, "  ejected, waiting for serial...\n");   /* progress -> stderr, keep stdout clean for piping */
        char new_ser[64];
        if (wait_for_fantasi_serial(new_ser, sizeof(new_ser), 10)) {
            usleep(500000);
            snprintf(g_ser_path, sizeof(g_ser_path), "%s", new_ser);
            if (ser_open(g_ser_path)) {
                usleep(200000);
                ser_drain();
                ser_connected = true;
                fprintf(stderr, "  reconnected: %s\n", g_ser_path);
            } else {
                fprintf(stderr, "  cannot reopen %s\n", g_ser_path);
            }
        } else {
            fprintf(stderr, "  serial port did not reappear\n");
        }
    }
    g_blk[0] = '\0';
}

#ifdef HAS_USB_VENDOR
/* Re-establish the WebUSB session after the device dropped. Returns true once
 * reconnected. A device that came back already in vendor mode (forced-composite
 * --usb, or a warm re-enumeration) is picked up directly; a switch-mode device
 * (PM3) boots to CDC after a reset, so we reopen serial and switch it back to
 * the vendor pipe with `webusb`, mirroring the initial upgrade. */
static bool usb_reconnect(void)
{
    if (usb_transport_open() == 0) return true;   /* back already in vendor mode */

    if (g_switch_mode) {
        char path[64];
        if (!wait_for_fantasi_serial(path, sizeof(path), 0)) return false;
        if (!ser_open(path)) return false;
        snprintf(g_ser_path, sizeof(g_ser_path), "%s", path);
        ser_write("\r\n", 2);          /* clear the CDC enumeration-quirk stray byte */
        usleep(100000);
        ser_drain();
        ser_write("webusb\r\n", 8);
        usleep(300000);                /* let the reply drain before CDC tears down */
        ser_close();
        if (usb_transport_open() == 0) return true;
    }
    return false;
}
#endif

/* ---- Readline event hook: detect disconnect/reconnect while idle ---- */

static int rl_poll_serial(void)
{
    /* Switch-mode MSC (PM3): CDC is gone (ser_fd < 0) and the MSC transfer path
     * does its own disconnect handling - skip. Composite devices (FZ/CU) keep a
     * persistent MSC mount but leave CDC open (ser_fd >= 0), so we must still
     * poll it here or a real disconnect (POLLHUP) is never noticed and the
     * prompt never goes red. */
    if (msc_active && ser_fd < 0) return 0;

#ifdef HAS_PROTO
    if (use_ble) {
        static int session_heartbeat;
        if (ser_connected && ++session_heartbeat >= 100) {
            session_heartbeat = 0;
            proto_session_ping();
        }
        if (ser_connected) {
            ble_transport_process();
            if (!ble_transport_connected()) {
                ser_connected = false;
                rl_set_prompt(
                    "\001" C_RED "\002" "fantasi" "\001" C_RESET "\002" "> ");
                rl_redisplay();
            }
        } else {
            /* Throttle reconnect attempts to ~1/s (the hook fires ~10/s). */
            static int throttle;
            if (++throttle >= 10) {
                throttle = 0;
                if (ble_transport_reconnect()) {
                    proto_session_id = 0;
                    proto_rx_len = 0;
                    ble_transport_set_response_session(0);
                    /* OPEN is deliberately lazy: doing protocol negotiation
                     * inside readline's idle hook would make `exit`/Ctrl-D
                     * wait behind a missing response.  The next remote command
                     * calls proto_session_ensure() before it writes. */
                    ser_connected = true;
                    printf("\n  reconnected over BLE\n");
                    rl_set_prompt("fantasi> ");
                    rl_redisplay();
                }
            }
        }
        return 0;
    }
#endif

#ifdef HAS_USB_VENDOR
    if (use_usb) {
        if (ser_connected) {
            /* The presence check is cheap and frequent; usb_transport_alive()
             * internally rate-limits and session-jitters actual EP0 lease
             * heartbeats so many idle CLI processes do not form a burst. */
            static int probe;
            if (++probe >= 5) {
                probe = 0;
                if (!usb_transport_alive()) {
                    ser_connected = false;
                    rl_set_prompt(
                        "\001" C_RED "\002" "fantasi" "\001" C_RESET "\002" "> ");
                    rl_redisplay();
                }
            }
        } else {
            /* Throttle reconnect attempts to ~1/s. */
            static int uthrottle;
            if (++uthrottle >= 10) {
                uthrottle = 0;
                if (usb_reconnect()) {
                    proto_session_id = usb_transport_session_id();
                    proto_rx_len = 0;
                    ser_connected = true;
                    printf("\n  reconnected over USB\n");
                    rl_set_prompt("fantasi> ");
                    rl_redisplay();
                }
            }
        }
        return 0;
    }
#endif

    if (ser_connected && ser_fd >= 0) {
        struct pollfd pfd = { .fd = ser_fd, .events = 0 };
        if (poll(&pfd, 1, 0) > 0 && (pfd.revents & (POLLERR | POLLHUP))) {
            ser_mark_disconnected();
            /* The MSC mount (if any) died with the device. Drop it so the
             * reconnect below isn't blocked by the stale msc_active + ser_fd<0
             * early-return, and so a later file command re-mounts the fresh
             * block device (fat_mount re-detects it). */
            if (msc_active) fat_unmount();
            rl_set_prompt(
                "\001" C_RED "\002" "fantasi" "\001" C_RESET "\002" "> ");
            rl_redisplay();
        }
    } else if (!ser_connected) {
        if (ser_try_reconnect()) {
            printf("\n  reconnected: %s\n", g_ser_path);
            rl_set_prompt("fantasi> ");
            rl_redisplay();
        }
    }
    return 0;
}

/* ---- Main ---- */

static volatile bool running = true;

static void sigint_handler(int sig) { (void)sig; running = false; }

#ifdef HAS_PROTO

static uint8_t proto_rx_accum[65536];
size_t  proto_rx_len;

static volatile sig_atomic_t proto_stream_interrupted;

/* A resynchronizing BLE parser examines arbitrary byte offsets after joining a
 * broadcast mid-frame or dropping an overflowing backlog. Requiring the normal
 * response invariants makes a protobuf-looking substring inside an incomplete
 * frame extraordinarily unlikely to be accepted as a new boundary. */
static bool proto_response_sane(const CliResponse *response)
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

#ifdef HAS_USB_VENDOR
/* Same framed protobuf over the USB vendor bulk pipe (libusb). The device runs
 * the identical engine, so requests/responses are byte-for-byte the BLE ones. */
static uint8_t usb_rx_accum[4096];
static size_t  usb_rx_len;
static unsigned usb_fast_polls;

static int usb_write_req(CliRequest *req)
{
    uint8_t buf[2 + CliRequest_size];
    pb_ostream_t stream = pb_ostream_from_buffer(buf + 2, sizeof(buf) - 2);
    if (!pb_encode(&stream, CliRequest_fields, req)) return -1;
    uint16_t len = (uint16_t)stream.bytes_written;
    buf[0] = (uint8_t)(len & 0xFF);
    buf[1] = (uint8_t)(len >> 8);
    return (usb_transport_write(buf, 2 + len) == (ssize_t)(2 + len)) ? 0 : -1;
}

static int usb_send_proto(CliRequest *req)
{
    uint8_t d[512];
    while (usb_transport_read(d, sizeof(d)) > 0) {}   /* drain stale (advances+releases) */
    usb_transport_read_reset();                        /* rewind read offset for the new response */
    usb_rx_len = 0;
    int rc = usb_write_req(req);
    /* Small command replies are latency-sensitive and normally ready within a
     * scheduler turn. Do not add speculative EP0 reads to every upload ACK:
     * two independent writers already contend for that single control pipe,
     * and extra READs can starve the other process's WRITE retry budget. */
    usb_fast_polls = (rc == 0 &&
                      req->which_payload == CliRequest_command_tag) ? 3u : 0u;
    return rc;
}

static int usb_recv_proto(CliResponse *resp)
{
    /* Reset the idle budget only when the accumulated frame grows. Bytes that
     * cannot fit are not progress and must not keep the loop alive. */
    unsigned resync = 0;
    for (int idle = 0; idle < 200; idle++) {
        if (proto_stream_interrupted) return -1;         /* ^C during a stream */
        if (!usb_transport_connected()) return -1;
        uint8_t chunk[512];
        ssize_t n = usb_transport_read(chunk, sizeof(chunk));
        if (n < 0) return -1;
        if (n > 0) {
            size_t room = sizeof(usb_rx_accum) - usb_rx_len;
            size_t copy = (size_t)n < room ? (size_t)n : room;
            if (copy > 0) {
                memcpy(usb_rx_accum + usb_rx_len, chunk, copy);
                usb_rx_len += copy;
                idle = 0;                 /* progress: the frame grew */
            } else {
                usleep(50000);            /* bytes we can't accumulate: no progress */
            }
        } else {
            /* A multiplexed EP0 READ returns an empty packet immediately when
             * its mailbox is empty. Preserve the legacy bulk path's roughly
             * ten-second response deadline without spinning on control I/O.
             * Immediately after submitting a request or acknowledging a
             * has_next frame, however, the lower-priority device worker is
             * normally only one scheduler turn behind us. A few short polls
             * avoid turning that benign race into a fixed 50 ms delay. */
            if (usb_fast_polls) {
                usb_fast_polls--;
                usleep(2000);
            } else {
                usleep(50000);
            }
        }
        if (usb_rx_len >= 2) {
            uint16_t msg_len = (uint16_t)usb_rx_accum[0] | ((uint16_t)usb_rx_accum[1] << 8);
            if (2u + (unsigned)msg_len > sizeof(usb_rx_accum)) {
                /* The declared frame cannot fit our accumulator, so the framing
                 * is desynchronized from the device mailbox (e.g. a stale prefix
                 * left in front of a fresh frame). The offset READ is idempotent
                 * and stateless, so discard the accumulator and re-read this
                 * frame from the top; that heals a host-side desync with no data
                 * loss. Bound the attempts so a genuinely oversized frame fails
                 * the command (caller re-issues) instead of hanging. */
                if (resync++ >= 4) return -1;
                usb_rx_len = 0;
                usb_transport_read_reset();
                idle = 0;
                continue;
            }
            if (usb_rx_len >= 2u + msg_len) {
                pb_istream_t stream = pb_istream_from_buffer(usb_rx_accum + 2, msg_len);
                *resp = (CliResponse){0};
                bool ok = pb_decode(&stream, CliResponse_fields, resp);
                size_t consumed = 2 + msg_len;
                usb_rx_len -= consumed;
                if (usb_rx_len > 0) memmove(usb_rx_accum, usb_rx_accum + consumed, usb_rx_len);
                /* Release this frame's device mailbox so a streamed next frame
                 * (has_next) can be emitted, and rewind the read offset. */
                usb_transport_frame_consumed();
                usb_fast_polls = (ok && resp->has_next) ? 3u : 0u;
                return ok ? 0 : -1;
            }
        }
    }
    return PROTO_RECV_TIMEOUT;
}
#endif /* HAS_USB_VENDOR */

static int ble_recv_proto_raw(CliResponse *resp);

static int ble_write_req_raw(CliRequest *req)
{
    uint8_t buf[2 + CliRequest_size];
    pb_ostream_t stream = pb_ostream_from_buffer(buf + 2, sizeof(buf) - 2);
    if (!pb_encode(&stream, CliRequest_fields, req)) return -1;
    uint16_t len = (uint16_t)stream.bytes_written;
    buf[0] = (uint8_t)(len & 0xFF);
    buf[1] = (uint8_t)(len >> 8);
    if (req->has_session && req->session) {
        ssize_t written = req->which_payload == CliRequest_file_write_tag
            ? ble_transport_write_session_command(req->session, buf + 2, len)
            : ble_transport_write_session(req->session, buf + 2, len);
        return written == len
             ? 0 : -1;
    }
    return (ble_transport_write(buf, 2 + len) == (ssize_t)(2 + len)) ? 0 : -1;
}

/* Establish a firmware-owned logical session. BLE performs this in-band (all
 * host processes see notifications and select the OPEN reply by randomized
 * request id); WebUSB receives its session directly from the EP0 OPEN request.
 * A legacy firmware response has no session field, which selects the
 * legacy single-stream behavior. */
static int proto_session_ensure(void)
{
#ifdef HAS_USB_VENDOR
    if (use_usb) {
        uint32_t current = usb_transport_session_id();
        if (current != proto_session_id) {
            proto_session_id = current;
            usb_rx_len = 0;
        }
        return usb_transport_connected() ? 0 : -1;
    }
#endif
    if (!use_ble || proto_session_id) return 0;

    ble_transport_process();
    uint8_t discard[512];
    while (ble_transport_read(discard, sizeof(discard)) > 0) {}
    proto_rx_len = 0;

    CliRequest request = CliRequest_init_zero;
    request.id = ++proto_req_id;
    request.which_payload = CliRequest_session_open_tag;
    request.payload.session_open = true;
    if (ble_write_req_raw(&request) < 0) return -1;

    for (;;) {
        CliResponse response;
        int rc = ble_recv_proto_raw(&response);
        if (rc < 0) return rc;
        if (response.id != request.id) continue;
        if (response.has_session && response.session) {
            proto_session_id = response.session;
            ble_transport_set_response_session(proto_session_id);
            return 0;
        }
        /* Legacy firmware decoded the new oneof as unknown and answered on its
         * implicit stream. Continue in compatibility mode. */
        return 0;
    }
}

static void proto_session_ping(void)
{
    if (!proto_session_id || !use_ble) return;
    CliRequest ping = CliRequest_init_zero;
    ping.id = ++proto_req_id;
    ping.which_payload = CliRequest_session_ping_tag;
    ping.payload.session_ping = true;
    (void)proto_write_req(&ping);
}

/* BLE sessions are leased by host ingress. Device output deliberately does not
 * renew a lease, otherwise a dead `log` process could survive forever while a
 * different process kept the shared physical link connected. This heartbeat is
 * used both while readline is idle and while a command is waiting for output. */
static void proto_stream_heartbeat(void)
{
    static struct timespec last;
    struct timespec now;
    if (!use_ble || !proto_session_id ||
        clock_gettime(CLOCK_MONOTONIC, &now) != 0)
        return;
    int64_t elapsed_ms = (int64_t)(now.tv_sec - last.tv_sec) * 1000 +
                         (now.tv_nsec - last.tv_nsec) / 1000000;
    if (last.tv_sec == 0 || elapsed_ms >= 10000) {
        last = now;
        proto_session_ping();
    }
}

static void proto_session_close_client(void)
{
    if (!proto_session_id || !use_ble || !ble_transport_connected()) return;
    CliRequest close = CliRequest_init_zero;
    close.id = ++proto_req_id;
    close.which_payload = CliRequest_session_close_tag;
    close.payload.session_close = true;
    (void)proto_write_req(&close);
    proto_session_id = 0;
    ble_transport_set_response_session(0);
}

/* Encode + write a request without touching the receive side. The pipelined
 * upload uses this so it doesn't drain pending acks between sends. */
int proto_write_req(CliRequest *req)
{
    if (proto_session_ensure() < 0) return -1;
    if (proto_session_id) {
        req->has_session = true;
        req->session = proto_session_id;
    }
#ifdef HAS_USB_VENDOR
    if (use_usb) return usb_write_req(req);
#endif
    return ble_write_req_raw(req);
}

int proto_send(CliRequest *req)
{
    if (proto_session_ensure() < 0) return -1;
    proto_active_request = req->id;
#ifdef HAS_USB_VENDOR
    if (use_usb) {
        if (proto_session_id) { req->has_session = true; req->session = proto_session_id; }
        return usb_send_proto(req);
    }
#endif
    /* Drain stale data from both transport and accumulator */
    ble_transport_process();
    char d[256]; while (ble_transport_read(d, sizeof(d)) > 0) {}
    proto_rx_len = 0;

    return proto_write_req(req);
}

static int ble_recv_proto_raw(CliResponse *resp)
{
    for (int idle = 0; idle < 200; idle++) {
        if (proto_stream_interrupted) return -1;
        proto_stream_heartbeat();
        /* If the device dropped (e.g. a reboot), stop waiting for a response
         * that will never come - checked periodically (the query isn't free). */
        if ((idle % 10) == 9 && !ble_transport_connected()) return -1;
        int bfd = ble_transport_fd();
        if (bfd >= 0) {
            struct pollfd pfd = { .fd = bfd, .events = POLLIN };
            poll(&pfd, 1, proto_rx_len > 0 ? 1 : 50);
        }
        ble_transport_process();
        /* Drain the transport ring completely so a notification burst cannot
         * overflow the length-prefixed protobuf stream. Regression risk: one
         * fixed-size read per iteration can let the ring grow between drains. */
        uint8_t chunk[1024];
        ssize_t n;
        while ((n = ble_transport_read(chunk, sizeof(chunk))) > 0) {
            idle = 0;
            size_t copy = (size_t)n;
            if (proto_rx_len + copy > sizeof(proto_rx_accum))
                copy = sizeof(proto_rx_accum) - proto_rx_len;
            memcpy(&proto_rx_accum[proto_rx_len], chunk, copy);
            proto_rx_len += copy;
            if (proto_rx_len == sizeof(proto_rx_accum)) break;
        }

        /* Notifications are broadcast to every subscribed process. An idle
         * process can discard an overflowing backlog or subscribe while some
         * other session is mid-frame, so do not assume byte zero is a frame
         * boundary. Scan for the first complete response that nanopb validates.
         * A complete frame is at most CliResponse_size + 2, which also bounds
         * how much unmatched suffix we retain. */
        for (;;) {
            bool discarded_foreign = false;
            for (size_t pos = 0; pos + 2 <= proto_rx_len; pos++) {
                uint16_t msg_len = (uint16_t)proto_rx_accum[pos] |
                                   ((uint16_t)proto_rx_accum[pos + 1] << 8);
                if (!msg_len || msg_len > CliResponse_size ||
                    pos + 2u + msg_len > proto_rx_len)
                    continue;
                CliResponse candidate = CliResponse_init_zero;
                pb_istream_t stream = pb_istream_from_buffer(
                    &proto_rx_accum[pos + 2], msg_len);
                if (!pb_decode(&stream, CliResponse_fields, &candidate) ||
                    !proto_response_sane(&candidate))
                    continue;
                size_t consumed = pos + 2u + msg_len;
                proto_rx_len -= consumed;
                if (proto_rx_len > 0)
                    memmove(proto_rx_accum, &proto_rx_accum[consumed],
                            proto_rx_len);

                /* Every BlueZ subscriber sees every notification. Once OPEN
                 * gives this process a SID, consume another process's complete
                 * response immediately. Leaving it at the head of the buffer
                 * makes the short buffered-data poll spin on the same frame and
                 * exhaust the timeout before our own response arrives. */
                if (proto_session_id &&
                    (!candidate.has_session ||
                     candidate.session != proto_session_id)) {
                    discarded_foreign = true;
                    break;
                }
                *resp = candidate;
                return 0;
            }
            if (!discarded_foreign) break;
        }
        size_t keep = 2u + CliResponse_size;
        if (proto_rx_len > keep) {
            memmove(proto_rx_accum, proto_rx_accum + proto_rx_len - keep, keep);
            proto_rx_len = keep;
        }
    }
    return PROTO_RECV_TIMEOUT;
}

int proto_recv(CliResponse *resp)
{
    if (proto_session_ensure() < 0) return -1;
    for (;;) {
#ifdef HAS_USB_VENDOR
        int rc = use_usb ? usb_recv_proto(resp) : ble_recv_proto_raw(resp);
#else
        int rc = ble_recv_proto_raw(resp);
#endif
        if (rc != 0) return rc;
        if (proto_session_id) {
            if (resp->has_session && resp->session == proto_session_id) return 0;
        } else if (!resp->has_session) {
            return 0;
        }
        /* BLE notifications are broadcast to every subscribed host process.
         * A response for a different device session is simply not ours. */
    }
}

void proto_send_cmd(const char *cmd)
{
    CliRequest req = CliRequest_init_zero;
    req.id = ++proto_req_id;
    req.which_payload = CliRequest_command_tag;
    size_t cl = strlen(cmd);                             /* the field is pre-zeroed by init_zero, so a */
    if (cl >= sizeof(req.payload.command)) cl = sizeof(req.payload.command) - 1;   /* bounded copy leaves */
    memcpy(req.payload.command, cmd, cl);               /* it NUL-terminated (an over-long command truncates) */
    proto_send(&req);
}

static void proto_stream_sigint(int sig) { (void)sig; proto_stream_interrupted = 1; }

static void proto_read_response(bool forward_stdin)
{
    proto_stream_interrupted = 0;
    struct sigaction sa = { .sa_handler = proto_stream_sigint };
    sigaction(SIGINT, &sa, NULL);

    CliResponse resp;
    bool has_more = false;

    for (;;) {
        if (proto_stream_interrupted) break;

        int rc = proto_recv(&resp);
        /* Once has_next establishes a stream, inactivity is not completion.
         * Keep waiting until the device ends it, the link fails, or the user
         * interrupts it. Ordinary requests still retain the receive deadline. */
        if (rc == PROTO_RECV_TIMEOUT && has_more) continue;
        if (rc < 0) break;
        /* A late/duplicated terminal response from the preceding request must
         * not complete this one. This matters particularly on BlueZ versions
         * that fan one notification out once per StartNotify owner. */
        if (resp.id != proto_active_request) continue;
        if (resp.which_payload == CliResponse_output_tag && resp.payload.output[0]) {
            if (g_cap) cap_puts(resp.payload.output);
            else {
                printf("%s", resp.payload.output);
                size_t L = strlen(resp.payload.output);
                if (L) g_out_pending_nl = (resp.payload.output[L - 1] != '\n');
            }
        }
        else if (resp.which_payload == CliResponse_error_tag)
            fprintf(stderr, "error: %s\n", resp.payload.error.message);
        fflush(stdout);
        has_more = resp.has_next;
        if (!has_more) goto done;

        /* Only a launch treats stdin as app I/O: a piped/non-tty ^C arrives as a
         * literal 0x03 byte (an interactive tty delivers SIGINT, handled above),
         * and forwarding it stops the app. Other streaming commands (ps, log)
         * must NOT read stdin - it holds the next scripted command. */
        if (forward_stdin) {
            struct pollfd sin = { .fd = STDIN_FILENO, .events = POLLIN };
            if (poll(&sin, 1, 0) > 0 && (sin.revents & POLLIN)) {
                char c;
                if (read(STDIN_FILENO, &c, 1) > 0 && c == 0x03) break;
            }
        }
    }

    /* Streaming exit: cancellation is an out-of-band protobuf request in a
     * multiplexed session, so it reaches the blocked worker without injecting
     * bytes into another client's stream. Raw ^C remains the legacy fallback. */
    if (proto_active_request) {
        if (proto_session_id) {
            CliRequest cancel = CliRequest_init_zero;
            cancel.id = ++proto_req_id;
            cancel.which_payload = CliRequest_cancel_tag;
            cancel.payload.cancel.request_id = proto_active_request;
            (void)proto_write_req(&cancel);
        } else {
            uint8_t ctrl_c = 0x03;
#ifdef HAS_USB_VENDOR
            if (use_usb) usb_transport_write(&ctrl_c, 1);
            else
#endif
            ble_transport_write(&ctrl_c, 1);
        }
        proto_stream_interrupted = 0;
        CliResponse drain;
        for (int i = 0; i < 20; i++) {
            if (proto_recv(&drain) < 0) break;
            if (drain.id == proto_active_request && !drain.has_next) break;
        }
    }

done:
    proto_active_request = 0;
    signal(SIGINT, sigint_handler);
}


/* Read & discard any pending BLE notifications until the link goes quiet, then
 * reset the protobuf framing accumulator. Used before re-requesting a download
 * range so stale bytes from the aborted stream can't desync the new one. */
void proto_drain_quiet(void)
{
    uint8_t tmp[1024];
    int quiet = 0;
    while (quiet < 6) {
#ifdef HAS_USB_VENDOR
        if (use_usb) {
            if (usb_transport_read(tmp, sizeof(tmp)) > 0) { quiet = 0; continue; }
            usleep(3000);
            quiet++;
            continue;
        }
#endif
        ble_transport_process();
        if (ble_transport_read(tmp, sizeof(tmp)) > 0) { quiet = 0; continue; }
        struct pollfd pfd = { .fd = ble_transport_fd(), .events = POLLIN };
        if (ble_transport_fd() >= 0) poll(&pfd, 1, 3);
        quiet++;
    }
    proto_rx_len = 0;   /* discard partial frame */
#ifdef HAS_USB_VENDOR
    usb_rx_len = 0;
#endif
}

#endif /* HAS_PROTO */

#ifdef HAS_USB_VENDOR
/* Read the device id ("PM3"/"FZ"/"CU") over the open serial CLI. */
static void query_device_id(char *out, size_t len)
{
    out[0] = '\0';
    ser_drain();
    ser_write("device\r\n", 8);
    char buf[256]; int pos = 0;
    for (int i = 0; i < 20 && pos < (int)sizeof(buf) - 1; i++) {
        struct pollfd p = { .fd = ser_fd, .events = POLLIN };
        if (poll(&p, 1, 100) > 0) {
            ssize_t n = read(ser_fd, buf + pos, sizeof(buf) - 1 - pos);
            if (n > 0) pos += n;
        }
    }
    buf[pos] = '\0';
    if      (strstr(buf, "PM3")) snprintf(out, len, "PM3");
    else if (strstr(buf, "CU"))  snprintf(out, len, "CU");
    else if (strstr(buf, "FZ"))  snprintf(out, len, "FZ");
}

/* Connect straight to the WebUSB vendor pipe without opening CDC. Composite
 * devices expose both interfaces, and going through CDC first would make
 * otherwise-independent WebUSB processes race for the serial CLI's replies.
 * A switch-mode device that has already dropped CDC is passed as such so the
 * host retains its conservative upload pacing. */
static bool connect_webusb_direct(bool switch_mode)
{
    if (usb_transport_open() != 0) return false;
    use_usb = true;
    ser_connected = true;
    /* The caller's guess (from the connection route) can be wrong: a PM3 reached
     * directly over the already-open vendor pipe arrives here with switch_mode
     * false, yet it still needs conservative upload pacing. The device's EP0 size,
     * now known from the descriptor, is the authoritative signal - a tiny EP0 is
     * the constrained SAM7S. Without this a directly-connected PM3 pipelines
     * uploads and overruns its dual-bank OUT, truncating the transfer. */
    g_switch_mode = switch_mode || usb_transport_constrained_ep0();
    printf("transport: WebUSB\n");
    return true;
}

/* Move file/CLI traffic onto the USB vendor (WebUSB) protobuf pipe - the default
 * for every USB device. The vendor interface carries the whole CLI + files
 * independent of the MSC drive, so file ops keep working even with mass-storage
 * disabled, and there's no MSC attach/detach churn.
 *
 * Switch-mode devices (PM3) send `webusb` and re-open over libusb; composite
 * devices (FZ/Kiisu/CU) already expose the vendor interface. On failure we fall
 * back to the serial + MSC path: `required` (from --usb) makes that a hard error,
 * otherwise it's a quiet fallback. A switch-mode device that already tore down
 * CDC can't fall back, so its failure is always surfaced. */
bool try_webusb_upgrade(bool required)
{
    char id[8];
    query_device_id(id, sizeof(id));
    bool switch_mode = (strcmp(id, "PM3") == 0);

    if (switch_mode) {
        ser_write("webusb\r\n", 8);
        usleep(300000);          /* let the reply drain before CDC tears down */
        ser_close();
    }
    for (int i = 0; i < 40; i++) {
        if (usb_transport_open() == 0) { use_usb = true; break; }
        usleep(150000);
    }
    if (!use_usb) {
        if (required || switch_mode)
            fprintf(stderr, "WebUSB upgrade failed (libusb permission, or no vendor interface)\n");
        else
            printf("transport: Serial\n");
        return false;
    }
    g_switch_mode = switch_mode;   /* PM3: pace uploads (SAM7S dual-bank OUT) */
    printf("transport: WebUSB\n");
    return true;
}
#endif /* HAS_USB_VENDOR */

/* ---- readline TAB completion for the fantasi> prompt ----
 * The first word completes against the command set: the host-local commands (the local_cmd registry)
 * PLUS the device's own commands (ps, log, version, ...), fetched once by querying its `help` and
 * cached - completing only local names would wrongly turn `p` into `pwd` when the device also has `ps`.
 * Arguments of the file commands complete against the device filesystem (see devfile_generator);
 * `upload`'s first argument is a host file (readline's default). */
static char g_dev_cmd[64][24];
static int  g_dev_ncmd = -1;   /* -1 = not yet fetched */

static void fetch_dev_cmds(void)
{
    if (!ser_connected) return;                 /* retry once the device is up */
    g_dev_ncmd = 0;
#ifdef HAS_PROTO
    if (use_ble || use_usb) {
        char out[2048]; int olen = 0;
        proto_send_cmd("help");
        CliResponse resp;
        do {
            if (proto_recv(&resp) < 0) break;
            if (resp.which_payload == CliResponse_output_tag) {
                int sl = (int)strlen(resp.payload.output);
                if (olen + sl < (int)sizeof out) { memcpy(out + olen, resp.payload.output, sl); olen += sl; }
            }
        } while (resp.has_next);
        out[olen] = '\0';
        for (char *lp = out; *lp && g_dev_ncmd < 64; ) {   /* first word of each help line = a command */
            char *eol = strchr(lp, '\n'); if (eol) *eol = '\0';
            char *p = lp; while (*p == ' ') p++;
            char *ns = p; while (*p && *p != ' ') p++;
            int nl = (int)(p - ns);
            if (nl > 0 && nl < 24) { memcpy(g_dev_cmd[g_dev_ncmd], ns, nl); g_dev_cmd[g_dev_ncmd][nl] = '\0'; g_dev_ncmd++; }
            lp = eol ? eol + 1 : lp + strlen(lp);
        }
    }
#endif
}

static char *cmd_generator(const char *text, int state)
{
    static const local_cmd_t *lc; static int di; static size_t tlen;
    if (state == 0) {
        if (g_dev_ncmd < 0) fetch_dev_cmds();
        lc = __start_local_cmd; di = 0; tlen = strlen(text);
    }
    for (; lc < __stop_local_cmd; ) {           /* host-local commands */
        const char *nm = lc->name; lc++;
        if (!strncmp(nm, text, tlen)) return strdup(nm);
    }
    for (; di < g_dev_ncmd; ) {                  /* device commands (skip any that duplicate a local one) */
        const char *nm = g_dev_cmd[di]; di++;
        if (strncmp(nm, text, tlen)) continue;
        int dup = 0;
        for (const local_cmd_t *c = __start_local_cmd; c < __stop_local_cmd; c++)
            if (!strcmp(c->name, nm)) { dup = 1; break; }
        if (!dup) return strdup(nm);
    }
    return NULL;
}

/* ---- Device filename completion ----
 * Path arguments complete against the device filesystem, listed the same way the
 * file commands read it: a dir_list protobuf request over WebUSB/BLE, or readdir
 * on the OS-mounted FAT over serial. On plain serial with no block device exposed
 * (a switch-mode PM3 still in CDC mode) completion stays silent - switching the
 * device into MSC mode on a TAB press would be far too disruptive. */
static char **g_fc_names;         /* entries of the directory being completed; dirs end in '/' */
static int    g_fc_count, g_fc_cap;
static bool   g_fc_dirs_only;     /* cd/mkdir/rmdir: only directories make sense */

static void fc_reset(void)
{
    for (int i = 0; i < g_fc_count; i++) free(g_fc_names[i]);
    g_fc_count = 0;
}

static void fc_add(const char *name, bool is_dir)
{
    if (!name[0] || strcmp(name, ".") == 0 || strcmp(name, "..") == 0) return;
    if (g_fc_count == g_fc_cap) {
        int ncap = g_fc_cap ? g_fc_cap * 2 : 32;
        char **nn = realloc(g_fc_names, ncap * sizeof *nn);
        if (!nn) return;
        g_fc_names = nn; g_fc_cap = ncap;
    }
    size_t l = strlen(name);
    char *s = malloc(l + 2);
    if (!s) return;
    memcpy(s, name, l);
    if (is_dir) s[l++] = '/';
    s[l] = '\0';
    g_fc_names[g_fc_count++] = s;
}

/* Fill g_fc_names with the entries of the resolved device directory `dirpath`. */
static void fc_list(const char *dirpath)
{
#ifdef HAS_PROTO
    if (use_ble || use_usb) {
        if (!ser_connected) return;   /* a dead link would just sit in proto_recv timeouts */
        CliRequest req = CliRequest_init_zero;
        req.id = ++proto_req_id;
        req.which_payload = CliRequest_dir_list_tag;
        strncpy(req.payload.dir_list.path, dirpath,
                sizeof(req.payload.dir_list.path) - 1);
        if (proto_send(&req) < 0) return;
        CliResponse resp;
        do {
            if (proto_recv(&resp) < 0) break;
            if (resp.which_payload == CliResponse_dir_entry_tag)
                fc_add(resp.payload.dir_entry.name, resp.payload.dir_entry.is_dir);
        } while (resp.has_next);
        return;
    }
#endif
    /* Serial + MSC path. Only when the FAT is reachable without disturbing the
     * device: already mounted, or the block device already exposed (composite
     * FZ/CU, where fat_mount just mounts it). */
    if (!g_mnt[0]) {
        char blk[64];
        if (!wait_for_fantasi_block(blk, sizeof blk, 0)) return;
        if (!fat_mount()) return;
    }
    char host[512];
    snprintf(host, sizeof host, "%s", fat_path(dirpath));   /* fat_path's static buffer is reused below */
    DIR *d = opendir(host);
    if (!d) return;
    struct dirent *e;
    while ((e = readdir(d))) {
        char full[800];
        snprintf(full, sizeof full, "%s/%s", host, e->d_name);
        struct stat st;
        fc_add(e->d_name, stat(full, &st) == 0 && S_ISDIR(st.st_mode));
    }
    closedir(d);
}

static char *devfile_generator(const char *text, int state)
{
    static int idx; static size_t blen;
    static char dirpfx[192], base[192];   /* "apps/fo" -> dirpfx "apps/", base "fo" */
    if (state == 0) {
        idx = 0;
        const char *slash = strrchr(text, '/');
        const char *b = slash ? slash + 1 : text;
        size_t dl = slash ? (size_t)(slash - text) + 1 : 0;   /* keep the '/' */
        if (dl >= sizeof dirpfx || strlen(b) >= sizeof base) return NULL;
        memcpy(dirpfx, text, dl); dirpfx[dl] = '\0';
        snprintf(base, sizeof base, "%s", b);
        blen = strlen(base);

        char resolved[256];
        resolve_path(dirpfx, resolved, sizeof resolved);   /* "" -> cwd; relative vs cwd; "/x/" -> "/x" */
        fc_reset();
        fc_list(resolved);
    }
    while (idx < g_fc_count) {
        const char *nm = g_fc_names[idx++];
        if (g_fc_dirs_only && nm[strlen(nm) - 1] != '/') continue;
        if (strncmp(nm, base, blen) != 0) continue;
        char *m = malloc(strlen(dirpfx) + strlen(nm) + 1);
        if (!m) return NULL;
        strcpy(m, dirpfx);
        strcat(m, nm);
        return m;
    }
    return NULL;
}

/* Commands whose arguments are device paths. `upload` is special-cased in
 * cli_completion: its first argument is a host file, device from the second on. */
static bool takes_device_paths(const char *cmd)
{
    static const char *const names[] = { "ls", "cd", "cat", "rm", "mkdir", "rmdir",
                                         "crc32", "edit", "cp", "mv", "launch" };
    for (size_t i = 0; i < sizeof names / sizeof names[0]; i++)
        if (strcmp(cmd, names[i]) == 0) return true;
    return false;
}

static char **cli_completion(const char *text, int start, int end)
{
    (void)end;

    /* Complete within the current ';'-separated segment (see exec_commands). */
    int seg = 0;
    for (int i = 0; i < start; i++)
        if (rl_line_buffer[i] == ';') seg = i + 1;
    while (rl_line_buffer[seg] == ' ' || rl_line_buffer[seg] == '\t') seg++;

    if (start <= seg) {                          /* the command word */
        rl_attempted_completion_over = 1;        /* command list only - no host-file fallback */
        return rl_completion_matches(text, cmd_generator);
    }

    /* An argument: find the segment's command word and which argument this is. */
    char cmd[24] = "";
    int argn = 0;                                /* 1 = completing the first argument */
    for (int i = seg; i < start; ) {
        while (i < start && (rl_line_buffer[i] == ' ' || rl_line_buffer[i] == '\t')) i++;
        if (i >= start) break;
        int ws = i;
        while (i < start && rl_line_buffer[i] != ' ' && rl_line_buffer[i] != '\t') i++;
        if (argn == 0) {
            int cl = i - ws;
            if (cl >= (int)sizeof cmd) cl = (int)sizeof cmd - 1;
            memcpy(cmd, rl_line_buffer + ws, cl);
            cmd[cl] = '\0';
        }
        argn++;
    }

    if (strcmp(cmd, "upload") == 0 && argn == 1)
        return NULL;                             /* upload's source is a host file: readline's default */

    rl_attempted_completion_over = 1;            /* device args never fall back to host files */
    if (!takes_device_paths(cmd) && !(strcmp(cmd, "upload") == 0 && argn >= 2))
        return NULL;

    g_fc_dirs_only = strcmp(cmd, "cd") == 0 || strcmp(cmd, "mkdir") == 0 ||
                     strcmp(cmd, "rmdir") == 0;
    char **m = rl_completion_matches(text, devfile_generator);
    /* A directory match ends in '/': suppress the trailing space so the next TAB
     * descends into it. */
    if (m && m[0][0] && m[0][strlen(m[0]) - 1] == '/')
        rl_completion_append_character = '\0';
    return m;
}

/* Refresh the client-side settings cached from the device. Currently that's just the active theme (read
 * from the device's saved `theme`, default synthwave when unset/unknown/disconnected). Runs once at
 * startup and again on the `reload` command; extend it as more client-side settings are added. */
void load_client_settings(void)
{
    char out[80] = {0};
    g_cap = out; g_cap_sz = sizeof out; g_cap_len = 0;
#ifdef HAS_PROTO
    if (use_ble || use_usb) { proto_send_cmd("settings get theme"); proto_read_response(false); }
    else
#endif
    if (ser_fd >= 0 && ser_connected) { ser_send_cmd("settings get theme"); ser_read_response(); }
    g_cap = NULL;

    char *nm = out;                                       /* trim surrounding whitespace/newlines */
    while (*nm == ' ' || *nm == '\r' || *nm == '\n' || *nm == '\t') nm++;
    char *e = nm + strlen(nm);
    while (e > nm && (e[-1] == ' ' || e[-1] == '\r' || e[-1] == '\n' || e[-1] == '\t')) *--e = 0;

    theme_set((*nm && !strstr(nm, "not set")) ? nm : NULL);   /* unset/empty -> synthwave default */
}

/* Execute one command line: a host-local command, or pass it through to the device over the active
 * transport. Returns false if the CLI should exit (the `exit`/`quit` command). */
static bool exec_line(const char *line)
{
    if (cli_local_match(line))
        return handle_local(line);
#ifdef HAS_PROTO
    if (use_ble || use_usb) {
        /* Only a launch turns stdin into app I/O (so a piped ^C reaches the app). */
        bool is_launch = strncmp(line, "launch", 6) == 0 && (line[6] == '\0' || line[6] == ' ');
        proto_send_cmd(line);
        proto_read_response(is_launch);
        if ((use_ble && !ble_transport_connected()) || (use_usb && !usb_transport_connected()))
            ser_connected = false;
        return true;
    }
#endif
    /* Switch-mode devices (PM3) share USB endpoints between MSC and CDC: leave MSC before a serial cmd. */
    if (ser_fd < 0 && msc_active) fat_unmount();
    if (ser_fd >= 0) { ser_send_cmd(line); ser_read_response(); }
    else if (!ser_connected) fprintf(stderr, "device disconnected\n");
    return true;
}

/* Run a line that may hold several ';'-separated commands, left to right, stopping
 * early (returning false, so the caller exits) if one is `exit`. strtok_r collapses
 * empty segments (";;", trailing/leading ';'); whitespace-only segments are trimmed
 * away too. The split is literal - there is no quoting/escaping, matching the simple
 * whitespace-delimited argument model, and no command takes a ';' in an argument. */
static bool exec_commands(const char *line)
{
    if (!strchr(line, ';')) return exec_line(line);   /* common case: no copy needed */

    char *dup = strdup(line);
    if (!dup) return exec_line(line);                 /* OOM: fall back to the whole line */

    bool cont = true;
    char *save = NULL;
    for (char *seg = strtok_r(dup, ";", &save); seg && cont;
         seg = strtok_r(NULL, ";", &save)) {
        while (*seg == ' ' || *seg == '\t') seg++;                       /* ltrim */
        char *e = seg + strlen(seg);
        while (e > seg && (e[-1] == ' ' || e[-1] == '\t')) *--e = '\0';  /* rtrim */
        if (*seg) cont = exec_line(seg);
    }
    free(dup);
    return cont;
}

int main(int argc, char **argv)
{
    char blk_path[64] = "";

#ifdef HAS_PROTO
    /* Request ids are visible on the shared BLE notification stream before a
     * logical session has been opened. Start each process in a different range
     * so simultaneous OPEN replies cannot be mistaken for one another. */
    struct timespec request_seed;
    clock_gettime(CLOCK_MONOTONIC, &request_seed);
    proto_req_id = (uint32_t)request_seed.tv_nsec ^
                   ((uint32_t)getpid() * 2654435761u);
    if (proto_req_id == UINT32_MAX) proto_req_id = 1;
#endif

    bool force_ble = false;
    bool force_usb = false;
    bool force_serial = false;
    const char *oneshot = NULL;              /* -c: run this one command, then exit (no history) */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) { oneshot = argv[++i]; continue; }
        if (strncmp(argv[i], "/dev/tty", 8) == 0)
            strncpy(g_ser_path, argv[i], sizeof(g_ser_path) - 1);
        else if (strncmp(argv[i], "/dev/sd", 7) == 0)
            strncpy(blk_path, argv[i], sizeof(blk_path) - 1);
        else if (strcmp(argv[i], "--ble") == 0)
            force_ble = true;
        else if (strcmp(argv[i], "--usb") == 0)
            force_usb = true;
        else if (strcmp(argv[i], "--serial") == 0)
            force_serial = true;
        else if (strcmp(argv[i], "--name") == 0 && i + 1 < argc) {
            /* Pick a specific device by name (hal_device_name()) when
             * several Fantasi devices are connected. Applies across every
             * transport - the name is the USB iSerial (USB-vendor + CDC serial)
             * and the BLE advertised name ("Fantasi <name>"). */
            snprintf(g_device_name, sizeof(g_device_name), "%s", argv[++i]);
            usb_transport_set_name(g_device_name);
#ifdef HAS_PROTO
            ble_transport_set_name(g_device_name);
#endif
        }
#ifdef HAS_PROTO
        else if (strncmp(argv[i], "--ble-addr=", 11) == 0) {
            force_ble = true;
            ble_transport_set_addr(argv[i] + 11);
        }
#endif
        else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("usage: fantasi [--ble|--ble-addr=ADDR|--usb|--serial] [--name NAME] [-c <command>] [/dev/ttyACMx] [/dev/sdX]\n");
            printf("  --name NAME   select a specific device by name (see `whoami`) when several are connected;\n");
            printf("                works over USB and BLE\n");
            return 0;
        }
    }

    if ((int)force_serial + (int)force_usb + (int)force_ble > 1) {
        fprintf(stderr, "--serial, --usb, and --ble are mutually exclusive\n");
        return 1;
    }

#ifdef HAS_PROTO
    if (force_ble) {
        if (ble_transport_open() == 0) {
            use_ble = true;
            ser_connected = true;
            printf("transport: BLE\n");
        } else {
            fprintf(stderr, "BLE connection failed\n");
            return 1;
        }
    }
    if (!use_ble)
#endif
    if (!g_ser_path[0]) {
        if (!find_fantasi_device(g_ser_path, sizeof(g_ser_path),
                                 blk_path[0] ? NULL : blk_path,
                                 sizeof(blk_path))) {
            bool connected = false;
#ifdef HAS_USB_VENDOR
            /* No CDC serial found - a switch-mode device (PM3) already in WebUSB mode
               exposes only the vendor interface; connect to it directly. --serial stays
               on the CDC path and never touches the vendor pipe. */
            if (!force_serial) connected = connect_webusb_direct(true);
#endif
#ifdef HAS_PROTO
            /* BLE is an auto-detect fallback only: --usb (WebUSB only) and --serial
               (serial only) never reach it. */
            if (!connected && !force_usb && !force_serial && ble_transport_open() == 0) {
                use_ble = true;
                ser_connected = true;
                printf("transport: BLE\n");
                connected = true;
            }
#endif
            if (!connected) {
                fprintf(stderr, "no Fantasi device found (%s)\n",
                        force_usb ? "USB" : force_serial ? "serial" : "USB or BLE");
                return 1;
            }
        }
    }

#ifdef HAS_PROTO
    if (!use_ble)
#endif
      if (!use_usb) {   /* skip if case B already connected over the vendor pipe */
#ifdef HAS_USB_VENDOR
        /* Prefer an already-present vendor interface before opening CDC. This
         * is essential for multiplexing: each host process can OPEN its own
         * WebUSB session, whereas CDC is one shared byte stream whose device-id
         * probe replies can be consumed by a different process. PM3 in serial
         * mode has no vendor interface yet, so it falls through to the legacy
         * serial `webusb` switch below. */
        if (!force_serial && connect_webusb_direct(false)) {
            /* connected over the vendor pipe - banner printed there */
        } else
        /* A switch-mode device (PM3) already in WebUSB mode has no CDC port
           (e.g. a previous fantasi invocation upgraded it and didn't switch
           back). If the serial port is absent, talk to the vendor pipe directly
           rather than failing to open it. */
        if (access(g_ser_path, F_OK) != 0 && connect_webusb_direct(true)) {
            /* connected over the vendor pipe - banner printed there */
        } else
#endif
        {
        if (!ser_open(g_ser_path)) return 1;

        usleep(500000);
        ser_write("\r\n", 2);
        ser_drain();
        ser_connected = true;

#ifdef HAS_USB_VENDOR
        /* Default to the WebUSB protobuf pipe for every USB device: file/CLI
         * traffic then works regardless of the MSC drive's state and needs no
         * mount. Falls back to this serial + MSC path if the vendor interface
         * can't be opened. --usb makes a failure a hard error; --serial opts out
         * of the upgrade entirely and stays on this CDC + MSC path (e.g. to leave
         * the vendor interface free for a second, independent channel). */
        if (!force_serial && !try_webusb_upgrade(force_usb) && force_usb)
            return 1;                    /* --usb is WebUSB only: no silent serial fallback */
#endif

        /* Legacy MSC file path (serial transport only). The block device is mounted
         * lazily on the first file command (fat_mount()), not eagerly here: a mount
         * costs a couple of seconds on a card-sized drive, and non-file commands
         * (whoami, version, ...) forwarded to the device's serial CLI never need it.
         * The WebUSB pipe carries files over protobuf and never mounts at all. */
        }
      }

    signal(SIGINT, sigint_handler);
    load_client_settings();                              /* apply the device's saved theme etc. (silent) */

    if (oneshot) {
        g_no_history = true;                             /* -c: run one command, save nothing */
        exec_commands(oneshot);
    } else {
        rl_attempted_completion_function = cli_completion;   /* TAB: command word + device filenames */
        /* The idle-reconnect hook is only useful for an interactive session. With
         * piped (non-TTY) input a non-NULL rl_event_hook keeps readline's read loop
         * alive at EOF instead of returning NULL, so a script that doesn't end in
         * `exit` (e.g. one ending in a streaming `launch`, which swallows trailing
         * lines as app input) would hang. Only install it on a real terminal. */
        if (isatty(STDIN_FILENO))
            rl_event_hook = rl_poll_serial;

        /* Persistent command history: load past commands so up/down recall them, and append each new
         * command so it survives the session. Only for an interactive terminal (piped input has no use
         * for it). The rfid app keeps its own history file (see cli/commands/rfid.c). */
        char hist[512] = "";
        if (isatty(STDIN_FILENO)) {
            fantasi_state_path("fantasi.log", hist, sizeof hist);
            if (hist[0]) { using_history(); read_history(hist); stifle_history(1000); }
        }

        #define PROMPT_LIVE    "fantasi> "
        #define PROMPT_DEAD    "\001" C_RED "\002" "fantasi" "\001" C_RESET "\002" "> "

        while (running) {
            const char *prompt = ser_connected ? PROMPT_LIVE : PROMPT_DEAD;
            char *rl = readline(prompt);
            if (!rl) break;
            if (!rl[0]) { free(rl); continue; }
            add_history(rl);
            if (hist[0]) write_history(hist);            /* persist this command immediately */

            char line[256];
            strncpy(line, rl, sizeof(line) - 1);
            line[sizeof(line) - 1] = '\0';
            free(rl);

            /* rtrim: a tab-completed filename gets a trailing space appended, which
             * would otherwise end up inside the path argument ("open failed"). */
            size_t ll = strlen(line);
            while (ll > 0 && (line[ll - 1] == ' ' || line[ll - 1] == '\t'))
                line[--ll] = '\0';
            if (!ll) continue;

            if (!exec_commands(line)) break;
        }
    }

    if (msc_active) fat_unmount();
    ser_close();
#ifdef HAS_PROTO
    if (use_ble) {
        proto_session_close_client();
        ble_transport_close();
    }
#endif
#ifdef HAS_USB_VENDOR
    if (use_usb) usb_transport_close();
#endif
    /* Terminate the line only if command output left the cursor mid-line, so a
     * response that already ends in '\n' doesn't get an extra blank line. */
    if (g_out_pending_nl) printf("\n");
    return 0;
}
