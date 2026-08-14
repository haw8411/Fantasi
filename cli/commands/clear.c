#include "cli_internal.h"

#include <stdio.h>
#include <unistd.h>

/* Clear the terminal screen + scrollback, like the shell `clear`. Host-only: writes
 * the ANSI erase sequences to stdout and touches no device. Skipped when stdout is not
 * a TTY (e.g. a piped `-c clear`) so it can't pollute redirected output. */
static void cmd_clear(void)
{
    if (isatty(STDOUT_FILENO)) {
        fputs("\033[H\033[2J\033[3J", stdout);   /* home, clear screen, clear scrollback */
        fflush(stdout);
    }
}

/* No argument; the registry slot is local_fn, so cast like pwd/the other niladic ones. */
LOCAL_COMMAND("clear", "clear the terminal screen", (local_fn)cmd_clear);
