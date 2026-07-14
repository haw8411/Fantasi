/* berryvfs - integration-test app: drives the embedded Berry VM through the app
 * API (api->be_exec) to read a file from the VFS and write a derived result,
 * then reads that result back and prints it. Proves the whole
 * app -> API -> Berry -> VFS (read + write) path end to end.
 *
 * The app - not the Berry script - prints the result: be_exec runs the VM in the
 * app task, which has no bound CLI context, so Berry's own print() would be
 * dropped. api->print routes through the app's output pipe instead, so the
 * script hands its answer back via a file the app reads with api->read_file. */
#include "app_api.h"

#define DATA_PATH   "/ramfs/berryvfs_data.txt"   /* written by the app  */
#define SCRIPT_PATH "/ramfs/berryvfs_run.be"     /* written by the app  */
#define RESULT_PATH "/ramfs/berryvfs_out.txt"    /* written by Berry     */

static const char DATA[] = "payload-2468";

/* Berry: read the data file the app wrote (VFS file port), append a marker, and
 * write the result to a second file for the app to read back. */
static const char SCRIPT[] =
    "var f = open('" DATA_PATH "', 'r')\n"
    "var s = f.readline()\n"
    "f.close()\n"
    "var o = open('" RESULT_PATH "', 'w')\n"
    "o.write(s + '-seen')\n"
    "o.close()\n";

int app_main(const fantasi_api_t *api)
{
    api->print("berryvfs: start\n");

    if (api->abi_version < 2 || !api->be_exec) {
        api->print("berryvfs: needs firmware ABI >= 2 with be_exec\n");
        return 1;
    }

    if (api->write_file(DATA_PATH, DATA, sizeof(DATA) - 1) != 0) {
        api->print("berryvfs: FAIL writing data file\n");
        return 1;
    }
    if (api->write_file(SCRIPT_PATH, SCRIPT, sizeof(SCRIPT) - 1) != 0) {
        api->print("berryvfs: FAIL writing script file\n");
        return 1;
    }

    int rc = api->be_exec(SCRIPT_PATH);
    if (rc != 0) {
        api->printf("berryvfs: FAIL be_exec rc=%d\n", rc);
        return 1;
    }

    char buf[64];
    int32_t n = api->read_file(RESULT_PATH, buf, sizeof(buf) - 1);
    if (n < 0) {
        api->print("berryvfs: FAIL reading result file\n");
        return 1;
    }
    buf[n] = '\0';
    api->printf("berryvfs: result=%s\n", buf);   /* expect payload-2468-seen */
    api->print("berryvfs: done\n");
    return 0;
}
