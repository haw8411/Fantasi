#include "../cli.h"
#include "../proto.h"

static int cmd_w(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    fantasi_proto_write_sessions();
    return 0;
}

CLI_COMMAND("w", "list active BLE/WebUSB protobuf sessions", cmd_w);
