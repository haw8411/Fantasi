/* The launch command only exists where the app loader is compiled in. The whole
 * translation unit is gated, so on platforms without FANTASI_ENABLE_APPS this
 * file registers nothing (and the wildcard build simply yields an empty TU). */
#ifdef FANTASI_ENABLE_APPS

#include "../cli.h"
#include "../app_run.h"
#include <string.h>

#ifdef FANTASI_ENABLE_BERRY
#include "../berry_host.h"
static int ends_with(const char *s, const char *sfx)
{
    size_t n = strlen(s), m = strlen(sfx);
    return n >= m && strcmp(s + n - m, sfx) == 0;
}
#endif

/* Dispatch by extension: a Berry script (.be) runs on the Berry VM, anything
 * else is loaded as a relocatable ELF app. */
static int cmd_launch(int argc, char **argv)
{
    if (argc < 2) { cli_printf("usage: launch <path>\r\n"); return 1; }
#ifdef FANTASI_ENABLE_BERRY
    if (ends_with(argv[1], ".be")) return be_exec(argv[1]);
#endif
    return app_run(argv[1]);
}

CLI_COMMAND("launch", "load and run an app (.elf) or Berry script (.be)", cmd_launch);

#endif
