/* app_run - load, run, and fully unload a Fantasi app. */
#ifndef CORE_APP_RUN_H
#define CORE_APP_RUN_H

/* Load the app at `path` (/ramfs/.. or /apps/..), run it on a dedicated,
 * interruptible task, and free everything afterward. Streams the app's output to
 * the current CLI session; returns when the app exits or the user presses ^C.
 * Returns the app's exit code, or a negative value on load/spawn failure or
 * abort. Must be called from a CLI/proto task (it owns that session). */
int app_run(const char *path);

/* Weak hardware hooks an app reaches through the API. Default to no-ops / "no
 * device"; a platform overrides the ones it supports. */
#include <stdint.h>
#include <stdbool.h>
void     hal_app_led(uint8_t r, uint8_t g, uint8_t b);
uint32_t hal_app_buttons(void);
bool     hal_app_has_display(void);
void     hal_app_display_clear(void);
void     hal_app_display_print(int col, int row, const char *s);
void     hal_app_display_flush(void);
/* Bracket an app's run so the platform pauses its own screen redraws. */
void     hal_app_display_acquire(void);
void     hal_app_display_release(void);

/* Request the currently-running app be killed - used by the `kill` command from a
 * DIFFERENT channel than the one running the app (the launching session sees the
 * request and does the single-owner teardown). Returns false if none running. */
bool     app_kill_running(void);

/* FreeRTOS task number of the running app, or -1 if none, so `kill <pid>` can
 * verify a pid refers to the app before acting (system tasks aren't killable). */
int      app_running_pid(void);

#endif /* CORE_APP_RUN_H */
