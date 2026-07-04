#include "cli_internal.h"

#include <stdio.h>

static void cmd_pwd(void)
{
    printf("%s\n", cwd);
}

/* Takes no argument; the registry slot is local_fn, so cast like the original. */
LOCAL_COMMAND("pwd", "print working directory", (local_fn)cmd_pwd);
