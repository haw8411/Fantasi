#include "../cli.h"
#include "../../hal/hal.h"
#include "../version.h"

#include <stddef.h>

static int cmd_version(int argc, char **argv)
{
    (void)argc; (void)argv;
    /* Device id ("FZ"/"CU"/"PM3") lowercased for the version line. */
    char dev[8];
    const char *id = hal_device_id();
    size_t i = 0;
    for (; id[i] && i < sizeof(dev) - 1; i++)
        dev[i] = (id[i] >= 'A' && id[i] <= 'Z') ? (char)(id[i] + 32) : id[i];
    dev[i] = '\0';
    cli_printf("fantasi %s \"%s\" (git %s, device %s)\r\n",
               FANTASI_VERSION, FANTASI_CODENAME, FANTASI_GIT_HASH, dev);
    return 0;
}

CLI_COMMAND("version", "show firmware version", cmd_version);
