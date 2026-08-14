# Firmware CLI commands

Each device command lives in its own file here and **self-registers** - there is
no central command table. At build time every `core/commands/*.c` is picked up by
the platform Makefiles (`$(wildcard $(ROOT)/core/commands/*.c)`), and each command
emits a descriptor into the `cli_cmd` linker section. The dispatcher
(`core/cli.c`, `cli_lookup`) and `help` walk that section, so adding a command is
just dropping in a file and rebuilding.

These commands run on the device and are reachable over **both** transports (USB
serial and BLE) - `core/proto.c` dispatches through the same `cli_lookup`.

## Add a command

Create `core/commands/mycmd.c`:

```c
#include "../cli.h"
#include "../../hal/hal.h"   /* only if you call hal_* */

static int cmd_mycmd(int argc, char **argv)
{
    cli_printf("hello from mycmd, argc=%d\r\n", argc);
    return 0;   /* exit status; nonzero signals an error */
}

CLI_COMMAND("mycmd", "one-line help text", cmd_mycmd);
```

Then `make PLATFORM=flipper` (or `chameleon` / `proxmark3`). `mycmd` now appears
in `help` and is dispatchable. Nothing else to edit.

### Rules & conventions

- **`CLI_COMMAND(name, help, fn)`** (from `core/cli.h`) registers the command.
  `fn` must be `static int fn(int argc, char **argv)` and unique within the file.
- Output goes through `cli_printf` / `cli_write` (routed to the calling task's
  transport). Use `\r\n` line endings. Never write to a raw FD.
- Emit no `\x06` bytes - that is the framing sentinel; `cli_write` strips it.
- Keep line output under the print-buffer limit (see `CLI_PRINT_BUF` in cli.c);
  call `cli_printf` repeatedly for longer output.
- For raw transport access (e.g. reading a passkey), use `cli_current_ctx()`.
- Includes are relative: `"../cli.h"`, `"../log.h"`, `"../../hal/hal.h"`.

### Shared helpers (not commands)

Files here that don't call `CLI_COMMAND` are shared helpers, compiled by the same
wildcard:

- `ble_common.{c,h}` - `parse_mac()` (used by pair/connect/unpair).
- `cmd_util.{c,h}` - `flush_before_reset()` (used by reboot/dfu/msc).

### Conditional commands

A command that depends on a build feature guards its whole file with the feature
macro so the wildcard can always compile it. Example - `launch.c` only registers
when the app loader is present:

```c
#ifdef FANTASI_ENABLE_APPS
#include "../cli.h"
#include "../app_run.h"
static int cmd_launch(int argc, char **argv) { ... }
CLI_COMMAND("launch", "...", cmd_launch);
#endif
```

On a platform without the macro the file compiles to an empty translation unit and
registers nothing.

## How it works

`CLI_COMMAND` places a `const cli_command_t { name, help, fn }` into the input
section `cli_cmd` with `__attribute__((used))`. The GNU linker concatenates these
across all objects and provides `__start_cli_cmd` / `__stop_cli_cmd` bounding the
array; referencing them from the (live) dispatcher keeps every entry through
`--gc-sections`. Order in the section is link order, so `help` sorts by name at
runtime. No linker-script changes are required.
