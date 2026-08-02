/* Host CLI color theme. A theme is a small role-based palette (neutral / accent /
 * alert, plus an interior fill and a border) that colored CLI output draws from, so
 * the whole client shares one visual identity instead of scattered hard-coded ANSI.
 * The active theme is chosen from the device's saved `theme` setting at startup
 * (default "synthwave"); `list` deliberately keeps its own fixed coloring. Presets
 * mirror tools/mfc_output.py. */
#ifndef FANTASI_CLI_THEME_H
#define FANTASI_CLI_THEME_H

#include <stdbool.h>

typedef struct {
    const char *name;
    const char *neutral;   /* labels, plain text */
    const char *accent;    /* structure, values */
    const char *alert;     /* headers, keys, marks */
    const char *bg;        /* interior fill (may be "") */
    const char *border;    /* box border */
} theme_t;

/* The active theme - never NULL (defaults to synthwave). Colored output reads its
 * role strings; when color is disabled every role is "" so the same code prints plain. */
extern const theme_t *g_theme;

/* Select the active theme by name. NULL / "" / unknown -> synthwave; "none" -> no color.
 * Also forced to no-color under NO_COLOR or when stdout isn't a TTY. Call once at start. */
void theme_set(const char *name);

/* Whether the active theme renders color at all (false under none/NO_COLOR/non-TTY). */
bool theme_color(void);

/* Newline-separated list of preset names, for help/error text. */
const char *theme_names(void);

#endif /* FANTASI_CLI_THEME_H */
