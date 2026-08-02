/* `reload` - re-read the client-side settings the CLI caches from the device (currently the active theme),
 * so a `settings set theme <name>` takes effect without restarting the client. */
#include "cli_internal.h"

#include <stdio.h>

static void cmd_reload(const char *arg)
{
    (void)arg;
    load_client_settings();
    printf("client settings reloaded\n");
}

LOCAL_COMMAND("reload", "refresh client settings from the device", cmd_reload);
