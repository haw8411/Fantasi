#include "cli_internal.h"

#include <stdio.h>
#include <string.h>
#include <poll.h>
#include <unistd.h>

/* help merges the host-local commands (this registry, shown in yellow) with the
 * device's own command set, queried by sending "help" and parsing its output -
 * over BLE (protobuf) or USB serial (framed text). */
static void cmd_help(void)
{
    typedef struct { const char *name; const char *help; bool local; } entry_t;
    entry_t entries[64];
    int count = 0;

    for (const local_cmd_t *c = __start_local_cmd;
         c < __stop_local_cmd && count < 64; c++) {
        entries[count].name = c->name;
        entries[count].help = c->help;
        entries[count].local = true;
        count++;
    }

    char all_output[2048];
    int all_len = 0;

#ifdef HAS_BLE
    if (use_ble || use_usb) {
        ble_send_cmd("help");
        CliResponse resp;
        do {
            if (ble_recv_proto(&resp) < 0) break;
            if (resp.which_payload == CliResponse_output_tag) {
                int slen = strlen(resp.payload.output);
                if (all_len + slen < (int)sizeof(all_output))
                    { memcpy(&all_output[all_len], resp.payload.output, slen); all_len += slen; }
            }
        } while (resp.has_next);
    } else
#endif
    {
        /* Composite (FZ/CU) serves CDC alongside MSC; only switch-mode (PM3,
         * ser_fd < 0 while mounted) must leave MSC to talk over serial. */
        if (ser_fd < 0 && msc_active) fat_unmount();
        ser_send_cmd("help");
        uint8_t fprev = 0;
        bool in_frame = false;
        for (;;) {
            struct pollfd pfd = { .fd = ser_fd, .events = POLLIN };
            if (poll(&pfd, 1, 5000) <= 0) break;
            char chunk[256];
            ssize_t n = read(ser_fd, chunk, sizeof(chunk));
            if (n <= 0) break;
            for (ssize_t i = 0; i < n; i++) {
                uint8_t c = (uint8_t)chunk[i];
                if (fprev == FRAME_SENTINEL && c == FRAME_START)
                    { in_frame = true; fprev = 0; continue; }
                if (fprev == FRAME_SENTINEL && c == FRAME_END)
                    { goto done; }
                if (fprev == FRAME_SENTINEL)
                    { fprev = c; continue; }
                if (c == FRAME_SENTINEL)
                    { fprev = c; continue; }
                fprev = c;
                if (!in_frame) continue;
                if (c == '\r') continue;
                if (all_len < (int)sizeof(all_output) - 1)
                    all_output[all_len++] = (char)c;
            }
        }
    }
done:
    all_output[all_len] = '\0';

    /* Parse lines from device help output */
    static char names[64][20], helps_buf[64][80];
    char *lp = all_output;
    while (*lp && count < 64) {
        char *eol = strchr(lp, '\n');
        if (eol) *eol = '\0';
        char *p = lp;
        while (*p == ' ') p++;
        char *ns = p;
        while (*p && *p != ' ') p++;
        int nlen = (int)(p - ns);
        while (*p == ' ') p++;
        if (nlen > 0 && nlen < 20 && *p) {
            memcpy(names[count], ns, nlen);
            names[count][nlen] = '\0';
            bool dup = false;
            for (int k = 0; k < count; k++)
                if (strcmp(entries[k].name, names[count]) == 0)
                    { dup = true; break; }
            if (!dup) {
                strncpy(helps_buf[count], p, 79);
                helps_buf[count][79] = '\0';
                entries[count].name = names[count];
                entries[count].help = helps_buf[count];
                entries[count].local = false;
                count++;
            }
        }
        lp = eol ? eol + 1 : lp + strlen(lp);
    }
    for (int i = 0; i < count - 1; i++)
        for (int j = i + 1; j < count; j++)
            if (strcmp(entries[i].name, entries[j].name) > 0) {
                entry_t tmp = entries[i];
                entries[i] = entries[j];
                entries[j] = tmp;
            }

    for (int i = 0; i < count; i++) {
        if (entries[i].local)
            printf("  " C_YELLOW "%-10s" C_RESET "  %s\n",
                   entries[i].name, entries[i].help);
        else
            printf("  %-10s  %s\n", entries[i].name, entries[i].help);
    }
}

LOCAL_COMMAND("help", "list commands", (local_fn)cmd_help);
