/* Flipper Zero microSD - SD-over-SPI driver. See sd_spi.h for the wiring notes. */
#include "sd_spi.h"

#include "stm32wb55xx.h"
#include "display.h"      /* display_lock/unlock serialise the shared SPI2 bus */
#include "FreeRTOS.h"
#include "task.h"

/* ---- Pins ----
 * The card is on SPI2 (the Flipper's "bus D", shared with the ST7565 display):
 * SCK=PD1, MOSI=PB15, MISO=PC2 (all AF5); CS=PC12 (GPIO push-pull, idle high). */
#define CS_PORT   GPIOC
#define CS_PIN    12

/* SPI2 is on APB1 (PCLK1 = 16 MHz on this 32 MHz-SYSCLK build).
 *   init : /128 -> 125 kHz  (the SD spec wants <=400 kHz during CMD0..ACMD41)
 *   data : /2   ->   8 MHz  (the card handles it fine; the display still runs at
 *                            its own /4 - bus_release restores the display's CR1) */
#define BR_SLOW   6u   /* /128 */
#define BR_FAST   0u   /* /2   */

/* ---- SD command set (SPI mode) ---- */
#define CMD0    0   /* GO_IDLE_STATE      */
#define CMD8    8   /* SEND_IF_COND       */
#define CMD9    9   /* SEND_CSD           */
#define CMD12   12  /* STOP_TRANSMISSION  */
#define CMD16   16  /* SET_BLOCKLEN       */
#define CMD17   17  /* READ_SINGLE_BLOCK  */
#define CMD18   18  /* READ_MULTIPLE_BLOCK*/
#define CMD24   24  /* WRITE_BLOCK        */
#define CMD25   25  /* WRITE_MULTIPLE_BLK */
#define CMD55   55  /* APP_CMD            */
#define CMD58   58  /* READ_OCR           */
#define ACMD41  41  /* SD_SEND_OP_COND    */

#define TOKEN_DATA        0xFEu   /* single-block / CMD9 start token       */
#define TOKEN_MULTI_WRITE 0xFCu   /* CMD25 per-block start token           */
#define TOKEN_STOP_TRAN   0xFDu   /* CMD25 stop token                      */

static bool     s_ready;
static bool     s_block_addr;     /* SDHC/SDXC address in blocks, not bytes */
static uint32_t s_sectors;
static uint8_t  s_br = BR_SLOW;

/* Diagnostics for the boot status line (see sd_spi.h). */
static int      s_stage;
static uint8_t  s_r1;
static int      s_card_type;
int     sd_spi_diag_stage(void)     { return s_stage; }
uint8_t sd_spi_diag_r1(void)        { return s_r1; }
int     sd_spi_diag_card_type(void) { return s_card_type; }

/* ---- Low-level SPI1 ----
 * Every spin is bounded: a stuck/misconfigured SPI must never hang the caller
 * (the SD is reached from the USB/MSC task; an infinite wait there stalls the
 * whole device). On timeout we return 0xFF - an idle MISO reads as 0xFF, so the
 * SD command layer treats a timed-out byte as "no response" and fails cleanly. */
#define XFER_SPIN 200000u
static uint8_t xfer(uint8_t v)
{
    uint32_t t = XFER_SPIN;
    while (!(SPI2->SR & SPI_SR_TXE)) { if (!--t) return 0xFF; }
    *(volatile uint8_t *)&SPI2->DR = v;
    t = XFER_SPIN;
    while (!(SPI2->SR & SPI_SR_RXNE)) { if (!--t) return 0xFF; }
    return *(volatile uint8_t *)&SPI2->DR;
}

static void cs_low(void)  { CS_PORT->BSRR = (1u << (CS_PIN + 16)); }
static void cs_high(void) { CS_PORT->BSRR = (1u << CS_PIN); }

/* Take the shared SPI2 bus (the ST7565 display is the other user) and remember
 * its clock/mode so we can hand the bus back exactly as we found it - the display
 * configures CR1 once at init and assumes it persists. */
static uint32_t s_saved_cr1;
static void bus_acquire(void)
{
    display_lock();
    s_saved_cr1 = SPI2->CR1;
}
static void bus_release(void)
{
    SPI2->CR1 &= ~SPI_CR1_SPE;
    SPI2->CR1 = s_saved_cr1;          /* restores the display's baud/mode + SPE */
    display_unlock();
}

/* Configure SPI1 for the SD card: master, mode 0 (CPOL=0 CPHA=0), MSB-first,
 * 8-bit, software NSS, baud `br`. Re-applied at each burst so it overrides any
 * mode the NFC driver (mode 1) left behind on the shared bus. */
static void bus_config(uint8_t br)
{
    SPI2->CR1 &= ~SPI_CR1_SPE;
    SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | ((uint32_t)br << SPI_CR1_BR_Pos);
    SPI2->CR2 = SPI_CR2_FRXTH | (7u << SPI_CR2_DS_Pos);   /* 8-bit; RXNE at 8 bits */
    SPI2->CR1 |= SPI_CR1_SPE;
    s_br = br;
}

static void pins_init(void)
{
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIODEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;
    (void)RCC->APB1ENR1;

    /* PD1 = SCK (AF5) */
    GPIOD->MODER   = (GPIOD->MODER & ~GPIO_MODER_MODE1) | (2u << GPIO_MODER_MODE1_Pos);
    GPIOD->AFR[0]  = (GPIOD->AFR[0] & ~(0xFu << (1 * 4))) | (5u << (1 * 4));
    GPIOD->OSPEEDR |= (3u << GPIO_OSPEEDR_OSPEED1_Pos);
    /* PB15 = MOSI (AF5) */
    GPIOB->MODER   = (GPIOB->MODER & ~GPIO_MODER_MODE15) | (2u << GPIO_MODER_MODE15_Pos);
    GPIOB->AFR[1]  = (GPIOB->AFR[1] & ~(0xFu << ((15 - 8) * 4))) | (5u << ((15 - 8) * 4));
    GPIOB->OSPEEDR |= (3u << GPIO_OSPEEDR_OSPEED15_Pos);
    /* PC2 = MISO (AF5). Pull-up: SD DO floats between the CS edge and the first
     * response, and an open slot reads as all-ones (0xFF) -> "no card". */
    GPIOC->MODER   = (GPIOC->MODER & ~GPIO_MODER_MODE2) | (2u << GPIO_MODER_MODE2_Pos);
    GPIOC->AFR[0]  = (GPIOC->AFR[0] & ~(0xFu << (2 * 4))) | (5u << (2 * 4));
    GPIOC->OSPEEDR |= (3u << GPIO_OSPEEDR_OSPEED2_Pos);
    GPIOC->PUPDR   = (GPIOC->PUPDR & ~GPIO_PUPDR_PUPD2) | (1u << GPIO_PUPDR_PUPD2_Pos);
    /* PC12 = CS (output push-pull, idle HIGH) */
    CS_PORT->BSRR  = (1u << CS_PIN);
    CS_PORT->MODER = (CS_PORT->MODER & ~GPIO_MODER_MODE12) | (1u << GPIO_MODER_MODE12_Pos);
    CS_PORT->OSPEEDR |= (3u << GPIO_OSPEEDR_OSPEED12_Pos);
}

/* ---- SD command layer ---- */

/* Wait until the card releases the bus (MISO high) after a busy period. */
static bool wait_ready(uint32_t tries)
{
    while (tries--) {
        if (xfer(0xFF) == 0xFF) return true;
    }
    return false;
}

/* Send a command; return the R1 response byte (0x80 on timeout). CS must be low.
 * `crc` matters only for CMD0 (0x95) and CMD8 (0x87); CRC is off otherwise. */
static uint8_t send_cmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    /* Let any prior write finish (except when issuing the stop command). */
    if (cmd != CMD12) wait_ready(50000);

    xfer(0x40 | cmd);
    xfer((uint8_t)(arg >> 24));
    xfer((uint8_t)(arg >> 16));
    xfer((uint8_t)(arg >> 8));
    xfer((uint8_t)arg);
    xfer(crc);

    if (cmd == CMD12) xfer(0xFF);   /* CMD12: skip one stuff byte */

    uint8_t r = 0x80;
    for (int i = 0; i < 20; i++) {
        r = xfer(0xFF);
        if (!(r & 0x80)) break;
    }
    return r;
}

/* CMD55 + ACMDxx. Returns the R1 of the application command. */
static uint8_t send_acmd(uint8_t acmd, uint32_t arg)
{
    send_cmd(CMD55, 0, 0xFF);
    return send_cmd(acmd, arg, 0xFF);
}

/* Wait for a data-start token (0xFE / multi-block). Returns true when seen. */
static bool wait_token(uint8_t token, uint32_t tries)
{
    uint8_t t;
    do {
        t = xfer(0xFF);
        if (t == token) return true;
        if (t != 0xFF) return false;    /* an error token (0x0x) */
    } while (tries--);
    return false;
}

static bool read_data_block(uint8_t *buf, uint32_t len)
{
    if (!wait_token(TOKEN_DATA, 100000)) return false;
    for (uint32_t i = 0; i < len; i++) buf[i] = xfer(0xFF);
    xfer(0xFF); xfer(0xFF);             /* discard CRC */
    return true;
}

/* ---- CSD -> capacity ---- */
static void read_capacity(void)
{
    cs_low();
    if (send_cmd(CMD9, 0, 0xFF) != 0) { cs_high(); xfer(0xFF); return; }
    uint8_t csd[16];
    bool ok = read_data_block(csd, 16);
    cs_high(); xfer(0xFF);
    if (!ok) return;

    if ((csd[0] >> 6) == 1) {
        /* CSD v2 (SDHC/SDXC): C_SIZE in bits [69:48]; capacity = (C_SIZE+1)*512KB. */
        uint32_t c_size = ((uint32_t)(csd[7] & 0x3F) << 16) |
                          ((uint32_t)csd[8] << 8) | csd[9];
        s_sectors = (c_size + 1) * 1024u;          /* *512KB / 512B = *1024 */
    } else {
        /* CSD v1 (SDSC): capacity = (C_SIZE+1) * 2^(C_SIZE_MULT+2) * 2^(READ_BL_LEN). */
        uint32_t read_bl_len = csd[5] & 0x0F;
        uint32_t c_size = ((uint32_t)(csd[6] & 0x03) << 10) |
                          ((uint32_t)csd[7] << 2) | (csd[8] >> 6);
        uint32_t c_size_mult = ((csd[9] & 0x03) << 1) | (csd[10] >> 7);
        uint32_t blocks = (c_size + 1) << (c_size_mult + 2);
        uint32_t block_len = 1u << read_bl_len;
        s_sectors = blocks * (block_len / 512u);
    }
}

/* ---- Public API ---- */

static bool sd_init_inner(void)
{
    pins_init();
    bus_config(BR_SLOW);

    /* Power-up: >=74 clocks with CS high and DI high. */
    cs_high();
    for (int i = 0; i < 10; i++) xfer(0xFF);

    /* CMD0: enter idle (SPI) state. Retry a few times - some cards are sluggish. */
    uint8_t r1 = 0xFF;
    for (int i = 0; i < 10; i++) {
        cs_low();
        r1 = send_cmd(CMD0, 0, 0x95);
        cs_high(); xfer(0xFF);
        if (r1 == 0x01) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    s_r1 = r1;
    if (r1 != 0x01) return false;
    s_stage = 1;

    /* CMD8: voltage check. R1 illegal-command => v1 card; else read the 32-bit R7. */
    bool v2;
    cs_low();
    r1 = send_cmd(CMD8, 0x000001AA, 0x87);
    if (r1 & 0x04) {           /* illegal command bit set -> SD v1 / MMC */
        v2 = false;
    } else {
        uint8_t r7[4];
        for (int i = 0; i < 4; i++) r7[i] = xfer(0xFF);
        v2 = (r7[2] == 0x01 && r7[3] == 0xAA);
    }
    cs_high(); xfer(0xFF);
    if (!v2 && (r1 & 0x04) == 0 && r1 != 0x01) { s_r1 = r1; return false; }
    s_card_type = v2 ? 2 : 1;
    s_stage = 2;

    /* ACMD41 until ready (HCS bit set for v2 cards). ~1s budget. */
    uint32_t hcs = v2 ? 0x40000000u : 0u;
    bool ready = false;
    for (int i = 0; i < 1000; i++) {
        cs_low();
        r1 = send_acmd(ACMD41, hcs);
        cs_high(); xfer(0xFF);
        if (r1 == 0x00) { ready = true; break; }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    s_r1 = r1;
    if (!ready) return false;
    s_stage = 3;

    /* CMD58: read OCR, check CCS (bit 30) for block vs byte addressing. */
    if (v2) {
        cs_low();
        r1 = send_cmd(CMD58, 0, 0xFF);
        uint8_t ocr[4] = {0};
        if (r1 == 0x00) for (int i = 0; i < 4; i++) ocr[i] = xfer(0xFF);
        cs_high(); xfer(0xFF);
        s_block_addr = (ocr[0] & 0x40) != 0;   /* CCS */
    }
    s_stage = 4;

    /* Byte-addressed cards: fix block length at 512. */
    if (!s_block_addr) {
        cs_low();
        r1 = send_cmd(CMD16, 512, 0xFF);
        cs_high(); xfer(0xFF);
        if (r1 != 0x00) { s_r1 = r1; return false; }
    }

    bus_config(BR_FAST);
    read_capacity();

    s_stage = 5;
    s_ready = true;
    return true;
}

bool sd_spi_init(void)
{
    s_ready = false;
    s_block_addr = false;
    s_sectors = 0;
    s_stage = 0;
    s_r1 = 0xFF;
    s_card_type = 0;

    bus_acquire();
    bool ok = sd_init_inner();
    bus_release();               /* hand SPI2 back at the display's clock/mode */
    return ok;
}

bool sd_spi_ready(void) { return s_ready; }
uint32_t sd_spi_sector_count(void) { return s_sectors; }

static uint32_t addr_of(uint32_t lba)
{
    return s_block_addr ? lba : lba * 512u;
}

static int sd_read_inner(uint32_t lba, uint8_t *buf, uint32_t count)
{
    bus_config(s_br);

    if (count == 1) {
        cs_low();
        int rc = -1;
        if (send_cmd(CMD17, addr_of(lba), 0xFF) == 0 && read_data_block(buf, 512))
            rc = 0;
        cs_high(); xfer(0xFF);
        return rc;
    }

    /* Multi-block read (CMD18) - one command streams `count` blocks. */
    cs_low();
    if (send_cmd(CMD18, addr_of(lba), 0xFF) != 0) { cs_high(); xfer(0xFF); return -1; }
    int rc = 0;
    for (uint32_t i = 0; i < count; i++) {
        if (!read_data_block(buf + i * 512u, 512)) { rc = -1; break; }
    }
    send_cmd(CMD12, 0, 0xFF);           /* STOP_TRANSMISSION */
    cs_high(); xfer(0xFF);
    return rc;
}

int sd_spi_read(uint32_t lba, uint8_t *buf, uint32_t count)
{
    if (!s_ready || count == 0) return -1;
    bus_acquire();
    int rc = sd_read_inner(lba, buf, count);
    bus_release();
    return rc;
}

/* Send one 512-byte block with the given start token. CS already low.
 * Returns true if the card accepted it. */
static bool write_data_block(const uint8_t *buf, uint8_t token)
{
    if (!wait_ready(500000)) return false;
    xfer(token);
    for (uint32_t i = 0; i < 512; i++) xfer(buf[i]);
    xfer(0xFF); xfer(0xFF);             /* dummy CRC */
    uint8_t resp = xfer(0xFF);
    if ((resp & 0x1F) != 0x05) return false;   /* not "data accepted" */
    return wait_ready(500000);          /* wait out the internal program time */
}

static int sd_write_inner(uint32_t lba, const uint8_t *buf, uint32_t count)
{
    bus_config(s_br);

    if (count == 1) {
        cs_low();
        int rc = -1;
        if (send_cmd(CMD24, addr_of(lba), 0xFF) == 0 &&
            write_data_block(buf, TOKEN_DATA))
            rc = 0;
        cs_high(); xfer(0xFF);
        return rc;
    }

    /* Multi-block write (CMD25). */
    cs_low();
    if (send_cmd(CMD25, addr_of(lba), 0xFF) != 0) { cs_high(); xfer(0xFF); return -1; }
    int rc = 0;
    for (uint32_t i = 0; i < count; i++) {
        if (!write_data_block(buf + i * 512u, TOKEN_MULTI_WRITE)) { rc = -1; break; }
    }
    /* Stop token, then wait out the final program. */
    wait_ready(500000);
    xfer(TOKEN_STOP_TRAN);
    xfer(0xFF);                          /* skip one byte after stop token */
    wait_ready(500000);
    cs_high(); xfer(0xFF);
    return rc;
}

int sd_spi_write(uint32_t lba, const uint8_t *buf, uint32_t count)
{
    if (!s_ready || count == 0) return -1;
    bus_acquire();
    int rc = sd_write_inner(lba, buf, count);
    bus_release();
    return rc;
}
