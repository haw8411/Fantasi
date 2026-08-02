/* app_run - load, run, and fully unload a Fantasi app. */
#ifndef CORE_APP_RUN_H
#define CORE_APP_RUN_H

#include <stdbool.h>

/* Load the app at `path` (/ramfs/.. or /apps/..), run it on a dedicated,
 * interruptible task, and free everything afterward. Streams the app's output to
 * the current CLI session; returns when the app exits or the user presses ^C.
 * Returns the app's exit code, or a negative value on load/spawn failure or
 * abort. Must be called from a CLI/proto task (it owns that session). */
int app_run(const char *path);

/* Load the module ELF at `path`, run it synchronously on the calling task with
 * the given API table, then fully unload it. Unlike app_run() this spawns no
 * task and does not touch the CLI-streaming/launch machinery: it is meant to be
 * called from *inside* a running app (a resident "shell") so it can hot-load a
 * feature module into RAM, run it, and reclaim it - one module resident at a
 * time. Returns the module's exit code, or -1 on load failure. Exposed to apps
 * as the resolved symbol `fantasi_run_module`.
 *
 * `delete_source`: delete `path` once the image is resident, before running it.
 * The load reads the file completely (every SHF_ALLOC section is streamed into
 * its own RAM and all relocations applied) before returning, so the file is dead
 * weight afterwards - and on a RAM-backed VFS it is heap the module itself could
 * be using. Pass true for a just-in-time module that will be re-provisioned next
 * run; pass false to keep the file (e.g. a module run repeatedly from flash).
 * Only applied on a successful load, so a failed load leaves the file to inspect
 * or retry. */
struct fantasi_api;
int app_run_module(const char *path, const struct fantasi_api *api, bool delete_source);

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ---- Async app session (protobuf channels) ----
 * Unlike app_run() (which streams synchronously and owns the calling task for the
 * app's whole life), an async launch spawns the app plus a pump task and returns
 * immediately, leaving the proto RX loop free to service the app: deliver a
 * feature module (FileWriteChunk -> /ramfs) it asks for mid-run, feed it
 * keystrokes, or stop it. The proto engine supplies these emit callbacks; the
 * pump task calls them to stream the app's output back, forward its module
 * requests, and signal exit. All carry the app_launch request id (`session`). */
typedef struct app_session_cb {
    void (*output)(uint32_t session, const char *data, size_t len);   /* app stdout chunk */
    void (*module_request)(uint32_t session, const char *name);       /* app wants module <name> */
    void (*done)(uint32_t session, int code);                         /* app exited (last message) */
} app_session_cb_t;

/* Start an async launch of the app at `path`. Returns 0 (spawned; the pump will
 * drive cb), or negative on busy (-1) / not-found (-2) / load error (-3). One
 * launch at a time (shared with app_run's launch_busy). */
int  app_launch_async(const char *path, uint32_t session, const app_session_cb_t *cb);
/* Feed console input to the running async app (api->read_input sees it). */
void app_session_feed_input(const uint8_t *data, size_t n);
/* Ask the running async app to stop (the ^C equivalent for proto channels). */
void app_session_stop(void);
/* True while an async app session is live. */
bool app_session_active(void);

/* Weak hardware hooks an app reaches through the API. Default to no-ops / "no
 * device"; a platform overrides the ones it supports. */
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

/* App launch shortcuts (slots 0-7, persisted as scN=<path> in settings.cfg).
 * Used by the Flipper Shortcuts menu, the `shortcut` command, and the screenless
 * LED+button launchers. shortcut_get fills buf and returns its length (0 if the
 * slot is empty); shortcut_run launches the slot's app (returns -1 if empty). */
int      shortcut_get(int slot, char *buf, int len);
int      shortcut_run(int slot);

#endif /* CORE_APP_RUN_H */
