/* Host CLI theme presets + active-theme selection. See theme.h. Presets are ported
 * verbatim from tools/mfc_output.py (same 256-color role palettes). */
#include "theme.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define FG(n) "\033[38;5;" #n "m"
#define BG(n) "\033[48;5;" #n "m"

/* name, neutral, accent, alert, bg, border */
static const theme_t PRESETS[] = {
    { "synthwave",   FG(253), FG(51),  FG(198), "",      FG(129) },  /* default: cyan / hot-pink */
    { "sunset",      FG(230), FG(220), FG(197), BG(234), FG(130) },
    { "cyberpunk",   FG(228), FG(45),  FG(201), BG(233), FG(226) },
    { "ember",       FG(246), FG(208), FG(196), BG(233), FG(130) },
    { "hotdogstand", FG(16),  FG(226), FG(231), BG(196), FG(226) },
    { "templeos",    FG(16),  FG(21),  FG(196), BG(231), FG(28)  },
    { "aurora",      FG(159), FG(84),  FG(141), BG(234), FG(24)  },
    { "midnight",    FG(189), FG(39),  FG(205), BG(233), FG(25)  },
    { "copper",      FG(180), FG(173), FG(79),  BG(234), FG(94)  },
    { "mint",        FG(194), FG(43),  FG(209), BG(235), FG(29)  },
};
#define NPRESETS ((int)(sizeof PRESETS / sizeof PRESETS[0]))
#define DEFAULT_PRESET 0                          /* synthwave */

/* No-color view of any theme: every role empty so colored code prints plain. */
static const theme_t THEME_NONE = { "none", "", "", "", "", "" };

static theme_t   g_active = PRESETS[DEFAULT_PRESET];
const theme_t   *g_theme  = &g_active;
static bool      g_color  = true;

bool theme_color(void) { return g_color; }

void theme_set(const char *name)
{
    /* No color on an explicit "none", under NO_COLOR, or when not writing to a TTY. */
    if ((name && !strcmp(name, "none")) || getenv("NO_COLOR") || !isatty(1)) {
        g_active = THEME_NONE;
        g_color = false;
        return;
    }
    g_color = true;
    int pick = DEFAULT_PRESET;
    if (name && *name)
        for (int i = 0; i < NPRESETS; i++)
            if (!strcmp(name, PRESETS[i].name)) { pick = i; break; }
    g_active = PRESETS[pick];
}

const char *theme_names(void)
{
    static char buf[256];
    if (!buf[0]) {
        size_t o = 0;
        for (int i = 0; i < NPRESETS && o < sizeof buf - 2; i++)
            o += (size_t)snprintf(buf + o, sizeof buf - o, "%s%s", i ? ", " : "", PRESETS[i].name);
    }
    return buf;
}
