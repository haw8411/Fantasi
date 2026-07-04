#include "cli_internal.h"

/* No-op body: the REPL ends the session when handle_local() sees a matched
 * command whose fn is cmd_exit. Registered under both names. */
void cmd_exit(const char *arg) { (void)arg; }

LOCAL_COMMAND("exit", "exit the CLI", cmd_exit);
LOCAL_COMMAND("quit", "exit the CLI", cmd_exit);
