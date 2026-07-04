# Host CLI commands

Each **host-side** command lives in its own file here and self-registers - no
central table. `cli/Makefile` picks up every `cli/commands/*.c` via a wildcard,
and each command emits a descriptor into the `local_cmd` linker section. The
dispatcher in `cli/main.c` (`cli_local_match`) and `help` walk that section.

Host commands run on your computer, not the device. They operate on the device's
files through the OS-mounted "Fantasi" FAT (MSC) or, when connected over BLE,
through the protobuf transport. Anything that is *not* a host command is forwarded
to the device's own CLI (see `../../core/commands/`).

`help` shows host commands in **yellow** and device commands plain, merged and
sorted.

## Add a command

Create `cli/commands/mycmd.c`:

```c
#include "cli_internal.h"
#include <stdio.h>

static void cmd_mycmd(const char *arg)
{
    if (!fat_mount()) { fprintf(stderr, "no filesystem\n"); return; }
    char path[256];
    resolve_path(arg, path, sizeof(path));
    printf("mycmd on %s (host mount: %s)\n", path, fat_path(path));
}

LOCAL_COMMAND("mycmd", "one-line help text", cmd_mycmd);
```

Then `make -C cli`. `mycmd` now appears in `help` and dispatches. Nothing else to
edit.

### Commands with a BLE variant

Over BLE there is no OS mount, so a command that touches files needs a separate
handler using the protobuf transport. Provide both and register with
`LOCAL_COMMAND_BLE`:

```c
#include "cli_internal.h"

static void cmd_mycmd(const char *arg) { /* MSC / stdio path */ }

#ifdef HAS_BLE
static void ble_cmd_mycmd(const char *arg)
{
    CliRequest req = CliRequest_init_zero;
    req.id = ++ble_req_id;
    /* ... fill req, ble_send_proto(&req), loop on ble_recv_proto(&resp) ... */
}
#endif

LOCAL_COMMAND_BLE("mycmd", "help text", cmd_mycmd, ble_cmd_mycmd);
```

When the host is talking over BLE (`use_ble`), the dispatcher calls `ble_fn`;
otherwise it calls `fn`. Wrap the BLE handler in `#ifdef HAS_BLE` - the CLI builds
without BLE when `libsystemd` is absent, and `LOCAL_COMMAND_BLE` drops the BLE arg
in that build.

### Registration macros (from `cli_internal.h`)

- `LOCAL_COMMAND(name, help, fn)` - command with no BLE-specific handler.
- `LOCAL_COMMAND_BLE(name, help, fn, ble_fn)` - separate handler for the BLE path.
- `fn`/`ble_fn` are `void (*)(const char *arg)`; `arg` is NULL when no argument.
  For a no-argument command, cast: `LOCAL_COMMAND("pwd", "...", (local_fn)cmd_pwd)`.
- The descriptor name is derived from `__LINE__`, so one file may register several
  names (e.g. `exit.c` registers both `exit` and `quit`).

### Shared internals (declared in `cli_internal.h`, defined in `main.c`)

`cwd`, `resolve_path`, `fat_mount`/`fat_unmount`/`fat_path`/`fat_sync`, `ser_fd`,
`msc_active`, `ser_send_cmd`, and (under `HAS_BLE`) `use_ble`, `ble_req_id`,
`ble_send_proto`, `ble_write_req`, `ble_recv_proto`, `ble_send_cmd`,
`ble_drain_quiet`, `ble_rx_len`, `CAT_WINDOW`. Reuse these rather than
reimplementing transport or mount logic.

## How it works

`LOCAL_COMMAND*` places a `const local_cmd_t` into the `local_cmd` section with
`__attribute__((used))`. The linker concatenates them and provides
`__start_local_cmd` / `__stop_local_cmd`; `cli_local_match` iterates between them.
The host build does not use `--gc-sections`, so `used` alone keeps the entries.
