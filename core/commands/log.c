#include "../cli.h"
#include "../log.h"

#include <string.h>
#include <stdio.h>

static int cmd_log(int argc, char **argv)
{
    if (argc >= 3 && strcmp(argv[1], "write") == 0) {
        char msg[128];
        int pos = 0;
        for (int i = 2; i < argc && pos < (int)sizeof(msg) - 2; i++) {
            if (i > 2) msg[pos++] = ' ';
            int n = snprintf(msg + pos, sizeof(msg) - pos, "%s", argv[i]);
            if (n > 0) pos += n;
        }
        fantasi_log(LOG_INFO, "%s", msg);
        return 0;
    }
    fantasi_log_stream();
    return 0;
}

CLI_COMMAND("log", "stream or write log messages", cmd_log);
