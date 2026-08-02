/* `kill` - stop the running app. Gated with the app loader (like `launch`).
 *
 * A launched app streams on the CLI/proto task that started it, so that channel
 * is busy until the app exits or gets ^C. `kill` is meant to be issued from a
 * DIFFERENT channel (e.g. WebUSB while an app runs over serial): it flags the
 * app for termination and the launching session performs the single-owner
 * teardown. Only the app task is killable - never a system task. */
#ifdef FANTASI_ENABLE_APPS

#include "../cli.h"
#include "../app_run.h"

#include <stdlib.h>

static int cmd_kill(int argc, char **argv)
{
    int app_pid = app_running_pid();
    if (app_pid < 0) {
        cli_write("kill: no app running\r\n");
        return 1;
    }
    if (argc >= 2) {
        int pid = atoi(argv[1]);
        if (pid != app_pid) {
            cli_printf("kill: pid %d is not the running app (pid %d); "
                       "only the app is killable (see ps)\r\n", pid, app_pid);
            return 1;
        }
    }
    app_kill_running();
    cli_printf("kill: stopping app (pid %d)\r\n", app_pid);
    return 0;
}

CLI_COMMAND("kill", "stop a pid or the running app", cmd_kill);

#endif
