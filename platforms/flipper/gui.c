/* gui - on-device menu for the Flipper Zero (see gui.h).
 *
 * One task owns the whole UI state machine:
 *
 *   SPLASH --UP--> MAIN ("Main Menu") --OK--> APPS / SETTINGS
 *      ^            |  ^                        |
 *      +---BACK-----+  +----------BACK----------+
 *
 * While a menu is open the task holds the display (hal_app_display_acquire),
 * which stops the periodic splash/status repaint in hal.c. Launching an app
 * reuses app_run() unchanged: this task binds its own CLI session whose
 * transport discards output and turns a BACK press into ^C (0x03), so the
 * back button ends the app through the exact same teardown as ^C/kill. */

#include "gui.h"
#include "display.h"
#include "app_api.h"
#include "app_run.h"
#include "cli.h"
#include "vfs.h"
#include "hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdio.h>
#include <string.h>

#define GUI_TASK_STACK    (configMINIMAL_STACK_SIZE * 8)  /* runs lfs + app_run paths, like cli */
#define GUI_BTN_QUEUE_LEN 8
#define GUI_DEBOUNCE_MS   30

/* List geometry, in pixels, sized around the stock-Flipper fonts in
 * font_data.c: the title (font_bold, 9+2 band) owns y0-10 with a separator
 * at y12; items (font_item, 8+2 band) run at a 13px pitch - a 12px rounded
 * highlight bar (1px pad + 10px band + 1px pad) plus a 1px gap - giving 4
 * rows that end flush at the panel bottom (y14 + 3*13 + 12 = 64). The
 * rightmost 7px column is reserved for the scroll arrows. */
#define GUI_TITLE_LINE_Y  12
#define GUI_LIST_Y        14
#define GUI_ITEM_BAND     (8 + 2)               /* font_item ascent+descent */
#define GUI_ROW_PITCH     (GUI_ITEM_BAND + 3)
#define GUI_VISIBLE       4
#define GUI_SEL_W         (DISPLAY_WIDTH - 8)
#define GUI_MAX_ITEMS     24
#define GUI_LABEL_MAX     (DISPLAY_COLS + 1)
#define GUI_PATH_MAX      48

static QueueHandle_t btn_q;

typedef enum { ST_SPLASH, ST_MAIN, ST_APPS, ST_SHORTCUTS, ST_SETTINGS } gui_state_t;
static gui_state_t state;

/* One shared item table; rebuilt on every menu entry. item_value is drawn
 * right-aligned in the row when non-empty (settings state: [on]/[off]). */
static char item_label[GUI_MAX_ITEMS][GUI_LABEL_MAX];
static char item_value[GUI_MAX_ITEMS][12];
static char item_path[GUI_MAX_ITEMS][GUI_PATH_MAX];   /* apps menu only */
static int  item_count;

/* Per-menu cursor, so Apps keeps its selection across launches and re-entry. */
static int sel_main, top_main;
static int sel_apps, top_apps;
static int sel_sc,   top_sc;
static int sel_set,  top_set;

/* ---- ISR side ---- */

void gui_buttons_from_isr(uint32_t mask, BaseType_t *woken)
{
    static TickType_t last_press[6];
    if (!btn_q) return;
    TickType_t now = xTaskGetTickCountFromISR();
    for (int i = 0; i < 6; i++) {
        if (!(mask & (1U << i))) continue;
        if (last_press[i] && (now - last_press[i]) < pdMS_TO_TICKS(GUI_DEBOUNCE_MS))
            continue;
        last_press[i] = now;
        uint8_t b = (uint8_t)(1U << i);
        xQueueSendFromISR(btn_q, &b, woken);
    }
}

/* ---- CLI session (so cli_printf and app_run work from this task) ----
 *
 * write: discard - menu-launched apps have no console to stream to.
 * read:  only pumped by app_run's session loop while an app is running;
 *        translating BACK into ^C makes the back button abort the app through
 *        app_run's existing ^C path. Other buttons are dropped here - a
 *        running app reads them live via api->buttons(). */

static size_t gui_tp_write(const uint8_t *buf, size_t len, void *ctx)
{
    (void)buf; (void)ctx;
    return len;
}

static size_t gui_tp_read(uint8_t *buf, size_t len, void *ctx)
{
    (void)ctx;
    uint8_t b;
    while (len && xQueueReceive(btn_q, &b, 0) == pdTRUE) {
        if (b == FANTASI_BTN_BACK) {
            buf[0] = 0x03;
            return 1;
        }
    }
    return 0;
}

static bool gui_tp_connected(void *ctx) { (void)ctx; return true; }

static cli_ctx_t gui_cli_ctx;

/* ---- rendering ---- */

static void draw_menu(const char *title, int sel, int top)
{
    display_lock();
    display_clear();

    display_text(&font_bold, 1, 0, title, false);
    display_hline(GUI_TITLE_LINE_Y);

    if (item_count == 0)
        display_text(&font_item, 4, GUI_LIST_Y + 1, "(empty)", false);

    for (int i = 0; i < GUI_VISIBLE && top + i < item_count; i++) {
        int y = GUI_LIST_Y + i * GUI_ROW_PITCH;
        bool inv = (top + i == sel);
        if (inv) {
            /* Rounded highlight bar: 1px pad above/below the glyph band. */
            display_fill_rect(0, y - 1, GUI_SEL_W, GUI_ITEM_BAND + 2, true);
            display_fill_rect(0, y - 1, 1, 1, false);
            display_fill_rect(0, y + GUI_ITEM_BAND, 1, 1, false);
            display_fill_rect(GUI_SEL_W - 1, y - 1, 1, 1, false);
            display_fill_rect(GUI_SEL_W - 1, y + GUI_ITEM_BAND, 1, 1, false);
        }
        display_text(&font_item, 4, y, item_label[top + i], inv);
        if (item_value[top + i][0]) {
            int vx = GUI_SEL_W - 4 - display_text_width(&font_item, item_value[top + i]);
            display_text(&font_item, vx, y, item_value[top + i], inv);
        }
    }

    /* Scroll indicators on the right edge when the list continues off-screen. */
    if (top > 0)
        display_text(&font_item, DISPLAY_WIDTH - 6, GUI_LIST_Y,
                     DISPLAY_CHAR_UP, false);
    if (top + GUI_VISIBLE < item_count)
        display_text(&font_item, DISPLAY_WIDTH - 6,
                     GUI_LIST_Y + (GUI_VISIBLE - 1) * GUI_ROW_PITCH,
                     DISPLAY_CHAR_DOWN, false);

    display_flush();
    display_unlock();
}

/* Move the cursor for an UP/DOWN press and keep it visible. */
static void nav(uint8_t b, int *sel, int *top)
{
    if (b == FANTASI_BTN_UP   && *sel > 0)              (*sel)--;
    if (b == FANTASI_BTN_DOWN && *sel < item_count - 1) (*sel)++;
    if (*sel < *top)                *top = *sel;
    if (*sel >= *top + GUI_VISIBLE) *top = *sel - GUI_VISIBLE + 1;
}

static void clamp(int *sel, int *top)
{
    if (*sel >= item_count) *sel = item_count ? item_count - 1 : 0;
    if (*sel < 0) *sel = 0;
    if (*top > *sel) *top = *sel;
    if (*top < 0) *top = 0;
}

/* ---- menu content ---- */

static void main_scan(void)
{
    memset(item_value, 0, sizeof(item_value));
    strcpy(item_label[0], "Apps");
    strcpy(item_label[1], "Shortcuts");
    strcpy(item_label[2], "Settings");
    item_count = 3;
}

/* Shortcut slots 0-7: each is an app path saved in settings.cfg as "scN=<path>"
 * (also settable with the `shortcut` CLI command / used by the LED+button
 * launcher on the screenless targets). Lists all eight slots; OK runs the
 * assigned app, empty slots do nothing. */
static void shortcuts_scan(void)
{
    memset(item_value, 0, sizeof(item_value));
    for (int i = 0; i < 8; i++) {
        char key[4] = { 's', 'c', (char)('0' + i), 0 };
        char path[GUI_PATH_MAX];
        int r = hal_settings_get(key, path, sizeof(path));
        if (r > 0) {
            snprintf(item_path[i], sizeof(item_path[0]), "%s", path);
            const char *base = strrchr(path, '/');
            base = base ? base + 1 : path;
            snprintf(item_label[i], sizeof(item_label[0]), "%d %s", i, base);
        } else {
            item_path[i][0] = '\0';
            snprintf(item_label[i], sizeof(item_label[0]), "%d -", i);
        }
    }
    item_count = 8;
}

static void apps_add(const char *name, uint32_t size, bool is_dir, void *vctx)
{
    (void)size;
    const char *prefix = vctx;
    if (is_dir || item_count >= GUI_MAX_ITEMS) return;
    snprintf(item_label[item_count], sizeof(item_label[0]), "%s", name);
    snprintf(item_path[item_count], sizeof(item_path[0]), "%s/%s", prefix, name);
    item_count++;
}

static void apps_scan(void)
{
    memset(item_value, 0, sizeof(item_value));
    item_count = 0;
    vfs_list("/ramfs", apps_add, (void *)"/ramfs");
    vfs_list("/apps",  apps_add, (void *)"/apps");
}

/* Settings shown as toggles. Known settings appear with their default even
 * before settings.cfg exists; any other key=value line in the file is shown
 * too and toggled generically between 0 and 1. */
#define SET_MAX     8
#define SET_KEY_MAX 14

static char set_key[SET_MAX][SET_KEY_MAX];
static bool set_on[SET_MAX];
static int  set_count;

/* Fold one "key=value" line into the toggle list (hal_settings_foreach cb). */
static void settings_scan_line(const char *line, void *ctx)
{
    (void)ctx;
    const char *eq = strchr(line, '=');
    if (!eq || eq == line) return;
    int klen = (int)(eq - line);
    if (klen >= SET_KEY_MAX) return;              /* key too long to track */
    char key[SET_KEY_MAX];
    memcpy(key, line, klen);
    key[klen] = '\0';
    const char *val = eq + 1;
    /* "hid" carries a string value (persistent/switch); everything else is 0/1. */
    bool on = (strcmp(key, "hid") == 0) ? (strcmp(val, "switch") == 0) : (val[0] == '1');
    for (int i = 0; i < set_count; i++)
        if (strcmp(set_key[i], key) == 0) { set_on[i] = on; return; }
    if (set_count < SET_MAX) {
        strcpy(set_key[set_count], key);
        set_on[set_count] = on;
        set_count++;
    }
}

static void settings_scan(void)
{
    /* Defaults first (must mirror what the firmware assumes when the key is
     * absent - hal_post_init treats missing "ble" as on; HID defaults to the
     * persistent keyboard). For "hid", set_on == "switch mode". */
    strcpy(set_key[0], "ble");
    set_on[0] = true;
    strcpy(set_key[1], "hid");
    set_on[1] = false;   /* persistent */
    strcpy(set_key[2], "msc");
    set_on[2] = true;    /* drive present */
    set_count = 3;

    hal_settings_foreach(settings_scan_line, NULL);

    for (int i = 0; i < set_count; i++) {
        snprintf(item_label[i], sizeof(item_label[0]), "%s", set_key[i]);
        if (strcmp(set_key[i], "hid") == 0)
            strcpy(item_value[i], set_on[i] ? "[switch]" : "[persist]");
        else
            strcpy(item_value[i], set_on[i] ? "[on]" : "[off]");
    }
    item_count = set_count;
}

static void settings_toggle(int idx)
{
    // Not a big fan of this - noproto
    if (idx >= set_count) return;
    bool on = !set_on[idx];

    /* "ble" has live side effects (start/stop the stack), so route it through
     * the same code path as the `ble on|off` command; its console output goes
     * to this session's discarding transport. Anything else just persists. */
    if (strcmp(set_key[idx], "ble") == 0) {
        const cli_command_t *cmd = cli_lookup("ble");
        char a0[] = "ble", on_s[] = "on", off_s[] = "off";
        char *argv[] = { a0, on ? on_s : off_s };
        if (cmd) cmd->fn(2, argv);
    } else if (strcmp(set_key[idx], "hid") == 0) {
        /* on == switch mode. Persist the string value and apply live - the mode
         * change re-enumerates so the host re-reads the interface set. */
        hal_settings_set("hid", on ? "switch" : "persistent");
        hal_hid_set_persistent(!on);
        hal_usb_reenumerate();
    } else if (strcmp(set_key[idx], "msc") == 0) {
        /* on == drive present. Persist and apply live (re-enumerate). */
        hal_settings_set("msc", on ? "1" : "0");
        hal_msc_set_enabled(on);
        hal_usb_reenumerate();
    } else {
        hal_settings_set(set_key[idx], on ? "1" : "0");
    }

    settings_scan();
}

/* ---- state machine ---- */

static void draw_state(void)
{
    switch (state) {
    case ST_MAIN:      draw_menu("Main Menu", sel_main, top_main); break;
    case ST_APPS:      draw_menu("Apps",      sel_apps, top_apps); break;
    case ST_SHORTCUTS: draw_menu("Shortcuts", sel_sc,   top_sc);   break;
    case ST_SETTINGS:  draw_menu("Settings",  sel_set,  top_set);  break;
    case ST_SPLASH:    break;   /* hal.c's display_refresh owns this screen */
    }
}

static void launch_selected(void)
{
    if (item_count == 0) return;

    /* The app gets a cleared screen to draw over (it may never draw at all -
     * console-only apps leave it blank until BACK). app_run acquires the
     * display on top of our hold, so its release on exit doesn't repaint the
     * splash - we redraw the Apps menu ourselves below. */
    display_lock();
    display_clear();
    display_flush();
    display_unlock();

    app_run(item_path[sel_apps]);

    xQueueReset(btn_q);   /* drop presses queued during the app's teardown */
    apps_scan();          /* the app may have changed /ramfs or /apps */
    clamp(&sel_apps, &top_apps);
}

static void launch_shortcut(void)
{
    if (item_count == 0 || !item_path[sel_sc][0]) return;   /* empty slot */

    display_lock();
    display_clear();
    display_flush();
    display_unlock();

    app_run(item_path[sel_sc]);

    xQueueReset(btn_q);
    shortcuts_scan();
    clamp(&sel_sc, &top_sc);
}

static void gui_task(void *arg)
{
    (void)arg;

    gui_cli_ctx.transport.write     = gui_tp_write;
    gui_cli_ctx.transport.read      = gui_tp_read;
    gui_cli_ctx.transport.connected = gui_tp_connected;
    cli_bind_ctx(&gui_cli_ctx);

    for (;;) {
        uint8_t b;
        if (xQueueReceive(btn_q, &b, portMAX_DELAY) != pdTRUE)
            continue;

        /* Releasing a button sputters press-edges too as the contact breaks,
         * so an edge alone isn't a press - a long hold would fire once more
         * on release (double-toggle, menu double-step). Wait out the bounce,
         * then require the button to still be physically down; a release
         * ghost reads as released here and is dropped. */
        vTaskDelay(pdMS_TO_TICKS(20));
        if (!(hal_app_buttons() & b))
            continue;

        /* An app launched from another session (serial/BLE/WebUSB) owns the
         * screen and reads buttons via api->buttons(); the menu stays out of
         * the way. (Apps we launch ourselves never reach this loop - the task
         * is blocked inside app_run for their whole run.) */
        if (app_running_pid() >= 0)
            continue;

        switch (state) {
        case ST_SPLASH:
            if (b == FANTASI_BTN_UP) {
                hal_app_display_acquire();
                state = ST_MAIN;
                main_scan();
                clamp(&sel_main, &top_main);
            }
            break;

        case ST_MAIN:
            nav(b, &sel_main, &top_main);
            if (b == FANTASI_BTN_BACK) {
                state = ST_SPLASH;
                hal_app_display_release();   /* repaints the splash */
            } else if (b == FANTASI_BTN_OK) {
                if (sel_main == 0) {
                    state = ST_APPS;
                    apps_scan();
                    clamp(&sel_apps, &top_apps);
                } else if (sel_main == 1) {
                    state = ST_SHORTCUTS;
                    shortcuts_scan();
                    clamp(&sel_sc, &top_sc);
                } else {
                    state = ST_SETTINGS;
                    settings_scan();
                    clamp(&sel_set, &top_set);
                }
            }
            break;

        case ST_APPS:
            nav(b, &sel_apps, &top_apps);
            if (b == FANTASI_BTN_BACK) {
                state = ST_MAIN;
                main_scan();
                clamp(&sel_main, &top_main);
            } else if (b == FANTASI_BTN_OK) {
                launch_selected();
            }
            break;

        case ST_SHORTCUTS:
            nav(b, &sel_sc, &top_sc);
            if (b == FANTASI_BTN_BACK) {
                state = ST_MAIN;
                main_scan();
                clamp(&sel_main, &top_main);
            } else if (b == FANTASI_BTN_OK) {
                launch_shortcut();
            }
            break;

        case ST_SETTINGS:
            nav(b, &sel_set, &top_set);
            if (b == FANTASI_BTN_BACK) {
                state = ST_MAIN;
                main_scan();
                clamp(&sel_main, &top_main);
            } else if (b == FANTASI_BTN_OK) {
                settings_toggle(sel_set);
            }
            break;
        }

        draw_state();
    }
}

void gui_init(void)
{
    btn_q = xQueueCreate(GUI_BTN_QUEUE_LEN, sizeof(uint8_t));
    if (!btn_q) return;
    xTaskCreate(gui_task, "gui", GUI_TASK_STACK, NULL, tskIDLE_PRIORITY + 1, NULL);
}
