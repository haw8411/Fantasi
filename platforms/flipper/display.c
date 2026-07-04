/* ST7565/ST7567 display driver for Flipper Zero.
 *
 * 128×64 monochrome LCD on SPI2:
 *   PD1  = SCK   (AF5)
 *   PB15 = MOSI  (AF5)
 *   PC11 = CS    (GPIO, active low)
 *   PB1  = D/I   (GPIO, 0=cmd 1=data)
 *   PB0  = RST   (GPIO, active low)
 *
 * Runs at 4 MHz SPI clock (16 MHz APB1 / 4).
 * display_init() runs from hal_init() before the scheduler starts - all
 * delays are busy-loops. At runtime several tasks share the framebuffer and
 * the SPI link (backlight status repaint, gui menu, app drawing); compose+
 * flush sequences must hold display_lock() or their interleaved SPI streams
 * reach the ST7565 as garbage commands (a stray data byte in the command phase
 * can set the display start line and tear the panel). */

#include "display.h"
#include "stm32wbxx.h"
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

static uint8_t fb[DISPLAY_PAGES * DISPLAY_WIDTH];
static SemaphoreHandle_t fb_lock;

/* ---- font ---- */

/* Public-domain 5×7 ASCII font, columns top-to-bottom (bit 0 = top row).
 * Covers printable ASCII 32-126. Each glyph is 5 bytes (5 columns). */
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /*   */
    {0x00,0x00,0x5F,0x00,0x00}, /* ! */
    {0x00,0x07,0x00,0x07,0x00}, /* " */
    {0x14,0x7F,0x14,0x7F,0x14}, /* # */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* $ */
    {0x23,0x13,0x08,0x64,0x62}, /* % */
    {0x36,0x49,0x55,0x22,0x50}, /* & */
    {0x00,0x05,0x03,0x00,0x00}, /* ' */
    {0x00,0x1C,0x22,0x41,0x00}, /* ( */
    {0x00,0x41,0x22,0x1C,0x00}, /* ) */
    {0x14,0x08,0x3E,0x08,0x14}, /* * */
    {0x08,0x08,0x3E,0x08,0x08}, /* + */
    {0x00,0x50,0x30,0x00,0x00}, /* , */
    {0x08,0x08,0x08,0x08,0x08}, /* - */
    {0x00,0x60,0x60,0x00,0x00}, /* . */
    {0x20,0x10,0x08,0x04,0x02}, /* / */
    {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E}, /* 9 */
    {0x00,0x36,0x36,0x00,0x00}, /* : */
    {0x00,0x56,0x36,0x00,0x00}, /* ; */
    {0x08,0x14,0x22,0x41,0x00}, /* < */
    {0x14,0x14,0x14,0x14,0x14}, /* = */
    {0x00,0x41,0x22,0x14,0x08}, /* > */
    {0x02,0x01,0x51,0x09,0x06}, /* ? */
    {0x32,0x49,0x79,0x41,0x3E}, /* @ */
    {0x7E,0x11,0x11,0x11,0x7E}, /* A */
    {0x7F,0x49,0x49,0x49,0x36}, /* B */
    {0x3E,0x41,0x41,0x41,0x22}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C}, /* D */
    {0x7F,0x49,0x49,0x49,0x41}, /* E */
    {0x7F,0x09,0x09,0x09,0x01}, /* F */
    {0x3E,0x41,0x49,0x49,0x7A}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F}, /* H */
    {0x00,0x41,0x7F,0x41,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01}, /* J */
    {0x7F,0x08,0x14,0x22,0x41}, /* K */
    {0x7F,0x40,0x40,0x40,0x40}, /* L */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F}, /* N */
    {0x3E,0x41,0x41,0x41,0x3E}, /* O */
    {0x7F,0x09,0x09,0x09,0x06}, /* P */
    {0x3E,0x41,0x51,0x21,0x5E}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46}, /* R */
    {0x46,0x49,0x49,0x49,0x31}, /* S */
    {0x01,0x01,0x7F,0x01,0x01}, /* T */
    {0x3F,0x40,0x40,0x40,0x3F}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F}, /* V */
    {0x3F,0x40,0x38,0x40,0x3F}, /* W */
    {0x63,0x14,0x08,0x14,0x63}, /* X */
    {0x07,0x08,0x70,0x08,0x07}, /* Y */
    {0x61,0x51,0x49,0x45,0x43}, /* Z */
    {0x00,0x7F,0x41,0x41,0x00}, /* [ */
    {0x02,0x04,0x08,0x10,0x20}, /* \ */
    {0x00,0x41,0x41,0x7F,0x00}, /* ] */
    {0x04,0x02,0x01,0x02,0x04}, /* ^ */
    {0x40,0x40,0x40,0x40,0x40}, /* _ */
    {0x00,0x01,0x02,0x04,0x00}, /* ` */
    {0x20,0x54,0x54,0x54,0x78}, /* a */
    {0x7F,0x48,0x44,0x44,0x38}, /* b */
    {0x38,0x44,0x44,0x44,0x20}, /* c */
    {0x38,0x44,0x44,0x48,0x7F}, /* d */
    {0x38,0x54,0x54,0x54,0x18}, /* e */
    {0x08,0x7E,0x09,0x01,0x02}, /* f */
    {0x0C,0x52,0x52,0x52,0x3E}, /* g */
    {0x7F,0x08,0x04,0x04,0x78}, /* h */
    {0x00,0x44,0x7D,0x40,0x00}, /* i */
    {0x20,0x40,0x44,0x3D,0x00}, /* j */
    {0x7F,0x10,0x28,0x44,0x00}, /* k */
    {0x00,0x41,0x7F,0x40,0x00}, /* l */
    {0x7C,0x04,0x18,0x04,0x78}, /* m */
    {0x7C,0x08,0x04,0x04,0x78}, /* n */
    {0x38,0x44,0x44,0x44,0x38}, /* o */
    {0x7C,0x14,0x14,0x14,0x08}, /* p */
    {0x08,0x14,0x14,0x18,0x7C}, /* q */
    {0x7C,0x08,0x04,0x04,0x08}, /* r */
    {0x48,0x54,0x54,0x54,0x20}, /* s */
    {0x04,0x3F,0x44,0x40,0x20}, /* t */
    {0x3C,0x40,0x40,0x20,0x7C}, /* u */
    {0x1C,0x20,0x40,0x20,0x1C}, /* v */
    {0x3C,0x40,0x30,0x40,0x3C}, /* w */
    {0x44,0x28,0x10,0x28,0x44}, /* x */
    {0x0C,0x50,0x50,0x50,0x3C}, /* y */
    {0x44,0x64,0x54,0x4C,0x44}, /* z */
    {0x00,0x08,0x36,0x41,0x00}, /* { */
    {0x00,0x00,0x7F,0x00,0x00}, /* | */
    {0x00,0x41,0x36,0x08,0x00}, /* } */
    {0x10,0x08,0x08,0x10,0x08}, /* ~ */
    {0x20,0x38,0x3E,0x38,0x20}, /* 0x7F: up arrow (DISPLAY_CHAR_UP) */
    {0x02,0x0E,0x3E,0x0E,0x02}, /* 0x80: down arrow (DISPLAY_CHAR_DOWN) */
};

/* ---- low-level SPI / GPIO ---- */

static void delay_ms(uint32_t ms)
{
    for (volatile uint32_t i = 0; i < ms * 4000u; i++) {}
}

static void spi_tx(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        while (!(SPI2->SR & SPI_SR_TXE)) {}
        *(volatile uint8_t *)&SPI2->DR = buf[i];
    }
    while (SPI2->SR & SPI_SR_BSY) {}
    /* Drain any RX bytes clocked in during TX. */
    while (SPI2->SR & SPI_SR_RXNE)
        (void)*(volatile uint8_t *)&SPI2->DR;
}

static void lcd_cmd(uint8_t c)
{
    GPIOB->BSRR = (1U << (1 + 16));   /* PB1 (D/I) LOW  = command */
    GPIOC->BSRR = (1U << (11 + 16));  /* PC11 (CS) LOW  = select  */
    spi_tx(&c, 1);
    GPIOC->BSRR = (1U << 11);         /* CS HIGH */
}

__attribute__((unused))   /* unused in the FANTASI_DISPLAY_COMPANION (Kiisu) flush */
static void lcd_data(const uint8_t *buf, uint32_t len)
{
    GPIOB->BSRR = (1U << 1);          /* PB1 (D/I) HIGH = data    */
    GPIOC->BSRR = (1U << (11 + 16));  /* CS LOW */
    spi_tx(buf, len);
    GPIOC->BSRR = (1U << 11);         /* CS HIGH */
}

/* ---- public API ---- */

/* Serialize compose+flush sequences between tasks. Callers before the
 * scheduler starts (hal_init's first splash paint) skip the lock - nothing
 * else can run yet. */
void display_lock(void)
{
    if (fb_lock && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
        xSemaphoreTake(fb_lock, portMAX_DELAY);
}

void display_unlock(void)
{
    if (fb_lock && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
        xSemaphoreGive(fb_lock);
}

void display_init(void)
{
    fb_lock = xSemaphoreCreateMutex();

    /* Enable GPIO port clocks (B, C, D) and SPI2. */
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN
                   |  RCC_AHB2ENR_GPIOCEN
                   |  RCC_AHB2ENR_GPIODEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;
    (void)RCC->APB1ENR1;

    /* PB0  = RST  (output push-pull) */
    GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE0)
                 | (1U << GPIO_MODER_MODE0_Pos);
    /* PB1  = D/I  (output push-pull) */
    GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE1)
                 | (1U << GPIO_MODER_MODE1_Pos);
    /* PB15 = MOSI (AF5, push-pull, very-high speed) */
    GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE15)
                 | (2U << GPIO_MODER_MODE15_Pos);
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(0xFU << ((15 - 8) * 4)))
                   | (5U << ((15 - 8) * 4));
    GPIOB->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED15_Pos);

    /* PC11 = CS   (output push-pull, start HIGH = deselected) */
    GPIOC->BSRR = (1U << 11);
    GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODE11)
                 | (1U << GPIO_MODER_MODE11_Pos);

    /* PD1  = SCK  (AF5, push-pull, very-high speed) */
    GPIOD->MODER = (GPIOD->MODER & ~GPIO_MODER_MODE1)
                 | (2U << GPIO_MODER_MODE1_Pos);
    GPIOD->AFR[0] = (GPIOD->AFR[0] & ~(0xFU << (1 * 4)))
                   | (5U << (1 * 4));
    GPIOD->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED1_Pos);

    /* SPI2: master, mode 0, MSB-first, 8-bit.
     * Flipper (direct ST7565): /4 prescaler = 4 MHz.
     * Kiisu (FANTASI_DISPLAY_COMPANION): the companion MCU forwards our stream
     * to the SH1106 one byte at a time in an IRQ. A full frame is one tight
     * 1 KB burst (see display_flush); at 4 MHz that can outrun the companion's
     * per-byte IRQ under its own I2C/timer load, overrunning and dropping bytes
     * mid-frame. Drop to /16 = 1 MHz for margin - ~8 ms per 1 KB frame, fine
     * for a menu UI. */
#ifdef FANTASI_DISPLAY_COMPANION
    SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI
              | (3U << SPI_CR1_BR_Pos);   /* /16 = 1 MHz */
#else
    SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI
              | (1U << SPI_CR1_BR_Pos);   /* /4 = 4 MHz */
#endif
    SPI2->CR2 = SPI_CR2_FRXTH | (7U << SPI_CR2_DS_Pos);
    SPI2->CR1 |= SPI_CR1_SPE;

    /* Hardware reset: RST LOW ≥1 ms, then HIGH, then wait ≥1 ms. */
    GPIOB->BSRR = (1U << (0 + 16));   /* PB0 LOW  */
    delay_ms(2);
    GPIOB->BSRR = (1U << 0);          /* PB0 HIGH */
    delay_ms(2);

    /* ST7565 init (ERC variant - same as stock Flipper firmware). */
    lcd_cmd(0xE2);          /* software reset             */
    lcd_cmd(0xA2);          /* bias 1/9                   */
    lcd_cmd(0xA0);          /* SEG direction normal       */
    lcd_cmd(0xC8);          /* COM direction reverse      */
    lcd_cmd(0x40);          /* start line 0               */
    lcd_cmd(0x20 | 0x05);   /* regulation ratio = 5       */
    lcd_cmd(0x81);          /* set electronic volume ...  */
    lcd_cmd(32);            /* ... contrast = 32          */
    lcd_cmd(0x28 | 0x07);   /* power control: VB+VR+VF   */
    lcd_cmd(0xA4);          /* all-pixel-on OFF (normal)  */
    lcd_cmd(0xAF);          /* display ON                 */

    display_clear();
    display_flush();
}

void display_clear(void)
{
    memset(fb, 0, sizeof(fb));
}

void display_print(int col, int row, const char *str)
{
    if (row < 0 || row >= DISPLAY_PAGES || !str) return;
    uint8_t *page = &fb[row * DISPLAY_WIDTH];

    while (*str && col >= 0 && col < DISPLAY_WIDTH) {
        uint8_t c = (uint8_t)*str++;
        if (c < 32 || c > 128) c = '?';
        const uint8_t *glyph = font5x7[c - 32];

        for (int i = 0; i < 5 && col < DISPLAY_WIDTH; i++, col++)
            page[col] = glyph[i];
        if (col < DISPLAY_WIDTH)
            page[col++] = 0x00;
    }
}

void display_print_inv(int col, int row, const char *str)
{
    if (row < 0 || row >= DISPLAY_PAGES || !str) return;
    uint8_t *page0 = &fb[row * DISPLAY_WIDTH];
    uint8_t *page1 = (row + 1 < DISPLAY_PAGES) ? &fb[(row + 1) * DISPLAY_WIDTH] : NULL;

    int start_col = col;
    while (*str && col >= 0 && col < DISPLAY_WIDTH) {
        uint8_t c = (uint8_t)*str++;
        if (c < 32 || c > 128) c = '?';
        const uint8_t *glyph = font5x7[c - 32];

        for (int i = 0; i < 5 && col < DISPLAY_WIDTH; i++, col++) {
            uint16_t shifted = ~((uint16_t)glyph[i] << 2);
            page0[col] = (uint8_t)(shifted & 0xFF) | 0x03;
            if (page1) page1[col] = (page1[col] & 0xFE) | ((shifted >> 8) & 0x01);
        }
        if (col < DISPLAY_WIDTH) {
            page0[col] = 0xFF;
            if (page1) page1[col] = page1[col] | 0x01;
            col++;
        }
    }
}

void display_hline(int y)
{
    if (y < 0 || y >= DISPLAY_HEIGHT) return;
    uint8_t *page = &fb[(y / 8) * DISPLAY_WIDTH];
    uint8_t bit = (uint8_t)(1U << (y % 8));
    for (int col = 0; col < DISPLAY_WIDTH; col++)
        page[col] |= bit;
}

/* ---- pixel-level drawing (menus/buttons sit off the 8px page grid) ---- */

static inline void px_put(int x, int y, bool on)
{
    if (x < 0 || x >= DISPLAY_WIDTH || y < 0 || y >= DISPLAY_HEIGHT) return;
    uint8_t bit = (uint8_t)(1U << (y & 7));
    if (on) fb[(y >> 3) * DISPLAY_WIDTH + x] |= bit;
    else    fb[(y >> 3) * DISPLAY_WIDTH + x] &= (uint8_t)~bit;
}

void display_fill_rect(int x, int y, int w, int h, bool on)
{
    for (int yy = y; yy < y + h; yy++)
        for (int xx = x; xx < x + w; xx++)
            px_put(xx, yy, on);
}

/* Proportional text (font_item / font_bold from font_data.c) with the text
 * top - the ascent line - at pixel row y; descenders extend below y+ascent.
 * inverse=true knocks the glyphs out of an already-drawn filled bar. */
void display_text(const fz_font_t *f, int x, int y, const char *str, bool inverse)
{
    while (*str && x < DISPLAY_WIDTH) {
        uint8_t c = (uint8_t)*str++;
        if (c < f->first || c > f->last) c = '?';
        const fz_glyph_t *g = &f->glyphs[c - f->first];

        for (int i = 0; i < g->w; i++) {
            uint16_t bits = f->cols[g->off + i];
            for (int b = 0; bits; b++, bits >>= 1)
                if (bits & 1)
                    px_put(x + g->xoff + i, y + g->ytop + b, !inverse);
        }
        x += g->adv;
    }
}

/* Advance width of `str` - for centering and right-alignment. */
int display_text_width(const fz_font_t *f, const char *str)
{
    int w = 0;
    while (*str) {
        uint8_t c = (uint8_t)*str++;
        if (c < f->first || c > f->last) c = '?';
        w += f->glyphs[c - f->first].adv;
    }
    return w;
}

/* 1px-bordered button with softly rounded corners, `label` (font_item)
 * centered inside; cx is the center column, y the top pixel row. Height is
 * the font band + a padding row and border each side. Punches its footprint
 * out of whatever is behind it (the splash art). */
void display_button(int cx, int y, const char *label)
{
    int tw = display_text_width(&font_item, label);
    int w  = tw + 9;   /* border+4px pad left; the last advance pads right */
    int h  = font_item.ascent + font_item.descent + 4;
    int x  = cx - w / 2;

    /* Punch a rounded footprint - the three pixels outside the curve at each
     * corner keep their background, so the button has no square notches. */
    display_fill_rect(x + 2, y, w - 4, h, false);
    display_fill_rect(x, y + 2, 2, h - 4, false);
    display_fill_rect(x + w - 2, y + 2, 2, h - 4, false);
    display_fill_rect(x + 1, y + 1, 1, 1, false);
    display_fill_rect(x + w - 2, y + 1, 1, 1, false);
    display_fill_rect(x + 1, y + h - 2, 1, 1, false);
    display_fill_rect(x + w - 2, y + h - 2, 1, 1, false);

    for (int xx = x + 2; xx < x + w - 2; xx++) {
        px_put(xx, y, true);
        px_put(xx, y + h - 1, true);
    }
    for (int yy = y + 2; yy < y + h - 2; yy++) {
        px_put(x, yy, true);
        px_put(x + w - 1, yy, true);
    }
    px_put(x + 1, y + 1, true);
    px_put(x + w - 2, y + 1, true);
    px_put(x + 1, y + h - 2, true);
    px_put(x + w - 2, y + h - 2, true);

    display_text(&font_item, x + 5, y + 2, label, false);
}

void display_blit(const uint8_t *data, uint32_t len)
{
    if (len > sizeof(fb)) len = sizeof(fb);
    memcpy(fb, data, len);
}

#ifdef FANTASI_DISPLAY_COMPANION
/* Kiisu: the LCD is an SH1106 driven by a companion MCU (STM32G431) that
 * emulates the Flipper's ST7565 - it receives our SPI stream as a slave and
 * re-drives the panel. It runs the panel in horizontal addressing mode with a
 * fixed 128x8 window (cols 0..127, pages 0..7): each data byte auto-advances
 * the column, wrapping to the next page at column 127 and back to (0,0) after
 * the last page. So the frame is one contiguous 1024-byte stream - no per-page
 * column/page commands are needed (the companion drops page commands anyway,
 * and column commands are ignored in horizontal mode).
 *
 * We therefore send the WHOLE framebuffer under a single CS assertion with one
 * command->data transition. Our fb is page-major (fb[page*128 + col]), which
 * is exactly the horizontal-addressing fill order, so it maps 1:1. Framing it
 * per page instead (8 CS / command->data boundaries) let the companion's
 * IRQ-driven CS/DI tracking drop a few bytes at each boundary, shearing every
 * page a little further across -> the static left/right banding. One boundary
 * per frame removes that; streaming exactly 1024 bytes keeps the panel's
 * auto-advance pointer wrapped back to (0,0) for the next frame. */
void display_flush(void)
{
    GPIOC->BSRR = (1U << (11 + 16));  /* PC11 (CS) LOW  = select  */
    GPIOB->BSRR = (1U << (1 + 16));   /* PB1 (D/I) LOW  = command */
    uint8_t col[2] = { 0x00, 0x10 }; /* lower col = 0, upper col = 0 (window start) */
    spi_tx(col, sizeof(col));
    GPIOB->BSRR = (1U << 1);          /* PB1 (D/I) HIGH = data    */
    spi_tx(fb, DISPLAY_PAGES * DISPLAY_WIDTH);   /* whole frame, one burst */
    GPIOC->BSRR = (1U << 11);         /* CS HIGH */
}
#else
void display_flush(void)
{
    /* Re-assert start line 0 every flush: interleaved SPI traffic or noise can
     * latch a stray 0x40|n start-line command in the controller, scrolling the
     * panel until re-init. Cheap self-heal. */
    lcd_cmd(0x40);
    for (uint8_t page = 0; page < DISPLAY_PAGES; page++) {
        lcd_cmd(0xB0 | page);
        lcd_cmd(0x10);
        lcd_cmd(0x00);
        lcd_data(&fb[page * DISPLAY_WIDTH], DISPLAY_WIDTH);
    }
}
#endif

#ifdef FANTASI_ENABLE_APPS
/* Bridge the loadable-app display API (the weak hal_app_display_* hooks in
 * core/app_run.c) to this panel. The Flipper has a screen, so apps may draw;
 * the firmware initialises the LCD at boot (hal_init -> display_init). Coordinate
 * convention matches display_print() used elsewhere in this file: `col` is a
 * pixel X (0..DISPLAY_WIDTH-1) and `row` a text line (0..DISPLAY_ROWS-1). */
#include "app_run.h"

bool hal_app_has_display(void) { return true; }
void hal_app_display_clear(void) { display_lock(); display_clear(); display_unlock(); }
void hal_app_display_print(int col, int row, const char *s) { display_lock(); display_print(col, row, s); display_unlock(); }
void hal_app_display_flush(void) { display_lock(); display_flush(); display_unlock(); }
#endif
