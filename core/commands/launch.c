/* The launch command only exists where the app loader is compiled in. The whole
 * translation unit is gated, so on platforms without FANTASI_ENABLE_APPS this
 * file registers nothing (and the wildcard build simply yields an empty TU). */
#ifdef FANTASI_ENABLE_APPS

#include "../cli.h"
#include "../app_run.h"

static int cmd_launch(int argc, char **argv)
{
    if (argc < 2) { cli_printf("usage: launch <path>\r\n"); return 1; }
    return app_run(argv[1]);
}

CLI_COMMAND("launch", "load and run an app (/ramfs or /apps path)", cmd_launch);

#endif
