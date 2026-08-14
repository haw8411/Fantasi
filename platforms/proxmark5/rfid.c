/* Fantasi / Proxmark5 (AT32F435) - RFID frontend driver.
 *
 * The Proxmark5 keeps an FPGA RF frontend (a GOWIN GW1N-4B on the PM5, replacing
 * the PM3's Spartan-II) that AUTOBOOTS the factory gateware from its own internal
 * flash - so, unlike the PM3, we do not load a bitstream: the frontend is already
 * live. This driver talks to that resident gateware exactly the way the PM3 does:
 *
 *   - a 16-bit "config register" written over a software-CS SPI (the FPGA command
 *     bus), carrying the major/minor mode and the LF divisor, and
 *   - a sample stream (the SSP bus, PB6-9) read back via DMA.
 *
 * The command-word encoding is the classic Proxmark protocol, carried forward for
 * the GW1N. The protocol ops reuse the portable decoders in core/rfid/.
 *
 * FPGA command bus pins: SCK=PC10, MISO=PC11, MOSI=PC12, CS=PA15; JTAGSEL(PD2)
 * held HIGH so the pins act as GPIO (not JTAG). See platforms/proxmark5/at32f435.h.
 */

#include "at32f435.h"
#include "../../hal/hal_rfid.h"
#include "../../apps/app_rfid.h"   /* FANTASI_RFID_SNIFF_BUFSZ - shared caller/HAL sniff scratch size */
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* LF envelope demod constants (same as the PM3/Chameleon software demod). */
#define LF_SPAN     12288   /* max samples streamed per acquire (~3 EM4100 frames) */
#define LF_WARMUP   256     /* settle dc/amp trackers before arming edge detection */
#define LF_SMOOTH   4       /* boxcar taps */

/* ---- FPGA command bus (bit-banged, software CS) ---- */
#define FC_SCK_PORT   GPIOC
#define FC_SCK_PIN    10
#define FC_MISO_PORT  GPIOC
#define FC_MISO_PIN   11
#define FC_MOSI_PORT  GPIOC
#define FC_MOSI_PIN   12
#define FC_CS_PORT    GPIOA
#define FC_CS_PIN     15
#define FC_SEL_PORT   GPIOD    /* JTAGSEL: HIGH = pins are GPIO (not JTAG) */
#define FC_SEL_PIN    2

/* ---- Classic Proxmark FPGA config-register protocol (PM3 fpga.h; the same
 * values carry forward for the GW1N gateware) ---- */
#define FPGA_CMD_SET_CONFREG         (1u << 12)
#define FPGA_CMD_SET_DIVISOR         (2u << 12)
#define FPGA_CMD_SET_PWR_PWM_LOW_CNT (4u << 12)   /* PM5-only: antenna drive voltage (12-bit) */
#define FPGA_MAJOR_MODE_LF_READER    (0u << 6)
#define FPGA_MAJOR_MODE_HF_ISO14443A (2u << 6)
#define FPGA_MAJOR_MODE_OFF          (7u << 6)
#define FPGA_LF_ADC_READER_FIELD     0x1        /* conf bit: energise LF antenna */
#define FPGA_HF_14A_READER_LISTEN    3          /* HF reader field on, listening */
#define FPGA_HF_14A_READER_MOD       4          /* HF reader modulating (transmit) */
#define FPGA_HF_14A_SNIFFER          0          /* HF minor mode: passive sniff      */
#define FPGA_HF_14A_TAGSIM_LISTEN    1          /* HF minor mode: TAG rx (Miller)    */
#define FPGA_HF_14A_TAGSIM_MOD       2          /* HF minor mode: TAG tx (subcarrier)*/
#define LF_DIVISOR_125               95         /* 12 MHz / (95+1) = 125 kHz */

/* ISO14443-A reader modified-Miller symbols (one SEC byte per air bit). */
#define SEC_X 0x0c   /* pause in 2nd quarter -> '1'                        */
#define SEC_Y 0x00   /* no pause -> '0' after a '1', idle, end-of-comm     */
#define SEC_Z 0xc0   /* pause at start -> '0' after '0'/start, start-of-comm */

/* HF RX loop bounds (in SSP bytes), from the PM3 reader. */
#define HF_RX_NOANSWER_BYTES  768
#define HF_RX_ABORT_BYTES     2560

static inline void fc_set(gpio_type *p, int pin, bool hi)
{
    if (hi) p->SCR = (1u << pin); else p->CLR = (1u << pin);
}
static inline void fc_delay(void)
{
    for (volatile int i = 0; i < 12; i++) __asm volatile("nop");
}

/* Microsecond/millisecond busy-waits for the T5577 downlink gap timing.
 * spin_us uses the Cortex-M4 DWT cycle counter (CPU = 288 MHz); the iteration
 * guard keeps it from hanging in a critical section if CYCCNT ever stalls. */
#define PM5_CYC_PER_US 288u
static void spin_us(uint32_t us)
{
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    uint32_t start = DWT->CYCCNT, want = us * PM5_CYC_PER_US;
    uint32_t guard = want * 4u + 100000u;
    while ((DWT->CYCCNT - start) < want && guard--) { }
}
static void spin_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms ? ms : 1)); }

static void fpga_cmd_setup(void)
{
    CRM->AHBEN1 |= CRM_AHBEN1_GPIOAEN | CRM_AHBEN1_GPIOCEN | CRM_AHBEN1_GPIODEN;
    (void)CRM->AHBEN1;
    /* JTAGSEL high: the shared pins act as GPIO, not the JTAG TAP. */
    fc_set(FC_SEL_PORT, FC_SEL_PIN, true);
    gpio_set_mode(FC_SEL_PORT, FC_SEL_PIN, GPIO_MODE_OUTPUT);
    /* Idle levels before enabling outputs: CS high (deasserted), SCK low (mode 0). */
    fc_set(FC_CS_PORT,  FC_CS_PIN,  true);
    fc_set(FC_SCK_PORT, FC_SCK_PIN, false);
    gpio_set_mode(FC_CS_PORT,   FC_CS_PIN,   GPIO_MODE_OUTPUT);
    gpio_set_mode(FC_SCK_PORT,  FC_SCK_PIN,  GPIO_MODE_OUTPUT);
    gpio_set_mode(FC_MOSI_PORT, FC_MOSI_PIN, GPIO_MODE_OUTPUT);
    gpio_set_mode(FC_MISO_PORT, FC_MISO_PIN, GPIO_MODE_INPUT);
}

/* Write one 16-bit config word, MSB-first, SPI mode 0 (CPOL=0/CPHA=0: data set
 * while SCK low, sampled by the FPGA on the rising edge). CS is active-low. */
static void fpga_send_cmd(uint16_t w)
{
    fc_set(FC_CS_PORT, FC_CS_PIN, false);
    fc_delay();
    for (int i = 15; i >= 0; i--) {
        fc_set(FC_MOSI_PORT, FC_MOSI_PIN, (w >> i) & 1);
        fc_delay();
        fc_set(FC_SCK_PORT, FC_SCK_PIN, true);   /* rising edge: FPGA samples */
        fc_delay();
        fc_set(FC_SCK_PORT, FC_SCK_PIN, false);
    }
    fc_delay();
    fc_set(FC_CS_PORT, FC_CS_PIN, true);
}

static void fpga_conf(uint16_t conf)  { fpga_send_cmd(FPGA_CMD_SET_CONFREG | conf); }
static void fpga_divisor(uint8_t d)   { fpga_send_cmd(FPGA_CMD_SET_DIVISOR | d); }
/* PM5-only: set the antenna drive-voltage PWM (12-bit; 4095 = max). */
static void fpga_pwr_pwm(uint16_t count) { fpga_send_cmd(FPGA_CMD_SET_PWR_PWM_LOW_CNT | (count & 0xFFFu)); }

/* ---- ADC mux (PM5): the FPGA_SWITCH pin (PB5). LOW = LF peak-detector into the
 * ADC, HIGH = HF. Boot leaves it HIGH, so LF reads nothing until we pull it low. */
#define FSW_PORT  GPIOB
#define FSW_PIN   5
static void adc_mux(bool hf)                      /* PB5: HIGH = HF (HIPKD), LOW = LF (LOPKD) */
{
    CRM->AHBEN1 |= CRM_AHBEN1_GPIOBEN;
    (void)CRM->AHBEN1;
    fc_set(FSW_PORT, FSW_PIN, hf);
    gpio_set_mode(FSW_PORT, FSW_PIN, GPIO_MODE_OUTPUT);
}

/* ---- Software (bit-banged) I2C on PC6=SCL / PC7=SDA (open-drain). Bus devices:
 * 0x47 TUSB320 (USB-C CC), 0x48 mainboard RGB LED, 0x50 24C02 EEPROM (factory
 * info), 0x51 antenna controller. The antenna's reg 0x02 ("map") selects the
 * resonant tap: bit7=125kHz, bit2=HFLED, bit1=LFLED, bit0=Q. 0x87 = 125kHz+Q+LEDs.
 * The RGB (pm5_rgb_set) is the blue "Fantasi on" indicator. */
#define I2C_SCL_PIN  6
#define I2C_SDA_PIN  7
#define ANT_ADDR7    0x51
#define ANT_REG_MAP  0x02
#define ANT_MAP_125K 0x87

/* ~100 kHz half-period. The antenna (0x51) and RGB (0x48) controllers are
 * MCU-based and clock-stretch, so keep to spec and honour SCL held low. */
static inline void i2c_dly(void) { for (volatile int i = 0; i < 1400; i++) __asm volatile("nop"); }
static inline void scl(bool hi)
{
    fc_set(GPIOC, I2C_SCL_PIN, hi);
    if (hi)   /* clock-stretch: wait (bounded) until the slave releases SCL high */
        for (volatile int i = 0; i < 200000 && !(GPIOC->IDT & (1u << I2C_SCL_PIN)); i++) { }
    i2c_dly();
}
static inline void sda(bool hi) { fc_set(GPIOC, I2C_SDA_PIN, hi); i2c_dly(); }
static inline bool sda_rd(void) { return (GPIOC->IDT & (1u << I2C_SDA_PIN)) != 0; }

static void i2c_setup(void)
{
    CRM->AHBEN1 |= CRM_AHBEN1_GPIOCEN;
    (void)CRM->AHBEN1;
    GPIOC->SCR = (1u << I2C_SCL_PIN) | (1u << I2C_SDA_PIN);     /* released (high) */
    gpio_set_otype(GPIOC, I2C_SCL_PIN, GPIO_OTYPE_OD);
    gpio_set_otype(GPIOC, I2C_SDA_PIN, GPIO_OTYPE_OD);
    gpio_set_pull(GPIOC, I2C_SCL_PIN, GPIO_PULL_UP);
    gpio_set_pull(GPIOC, I2C_SDA_PIN, GPIO_PULL_UP);
    gpio_set_mode(GPIOC, I2C_SCL_PIN, GPIO_MODE_OUTPUT);
    gpio_set_mode(GPIOC, I2C_SDA_PIN, GPIO_MODE_OUTPUT);
}
static void i2c_start(void) { sda(true); scl(true); sda(false); scl(false); }
static void i2c_stop(void)  { sda(false); scl(true); sda(true); }
static bool i2c_wr(uint8_t b)   /* returns ACK (true = acked) */
{
    for (int i = 0; i < 8; i++) { sda((b & 0x80) != 0); scl(true); scl(false); b <<= 1; }
    sda(true);                                   /* release SDA for ACK */
    scl(true);
    bool ack = !sda_rd();                         /* slave pulls low to ACK */
    scl(false);
    return ack;
}
static bool i2c_write_reg(uint8_t addr7, uint8_t reg, uint8_t data)
{
    i2c_start();
    bool ok = i2c_wr((uint8_t)(addr7 << 1));      /* write */
    ok = i2c_wr(reg) && ok;
    ok = i2c_wr(data) && ok;
    i2c_stop();
    return ok;
}
/* Board RGB status LED, on the same software-I2C bus as the antenna controller
 * but at device 0x48. Protocol (from PR at32_unit_test.c test_i2c_rgb_simple):
 * reg 0x02 = index (which lamp), reg 0x01 = count (# lamps, must be >=1 or the
 * lamp stays dark), reg 0x03 = RGB888 data. Blue = pm5_rgb_set(0, 0, 200). */
#define RGB_ADDR7    0x48
bool pm5_rgb_set(uint8_t r, uint8_t g, uint8_t b)
{
    i2c_setup();
    i2c_write_reg(RGB_ADDR7, 0x02, 0x00);        /* index 0 */
    i2c_write_reg(RGB_ADDR7, 0x01, 0x01);        /* 1 lamp on the chain */
    i2c_start();                                 /* reg 0x03 + 3 data bytes */
    bool ack = i2c_wr((uint8_t)(RGB_ADDR7 << 1));  /* did the RGB MCU answer? */
    i2c_wr(0x03);
    i2c_wr(r); i2c_wr(g); i2c_wr(b);
    i2c_stop();
    return ack;
}

static void antenna_lf_on(void)
{
    i2c_setup();
    i2c_write_reg(ANT_ADDR7, ANT_REG_MAP, ANT_MAP_125K);   /* select 125 kHz tap */
}

/* The composite antenna board carries two extra LEDs - HF and LF - driven by the same
 * antenna controller (0x51), as bits in its map register: bit2 = HFLED, bit1 = LFLED
 * (active-high). Writing just the LED bits leaves the LF resonant tap unselected, which
 * is fine at rest; antenna_lf_on() reasserts the tap (and both LEDs, via 0x87) when LF
 * operation starts. Returns the I2C ack. */
bool pm5_ant_led(bool hf, bool lf)
{
    i2c_setup();
    return i2c_write_reg(ANT_ADDR7, ANT_REG_MAP,
                         (uint8_t)((hf ? 0x04u : 0u) | (lf ? 0x02u : 0u)));
}

static rfid_mode_t s_mode = RFID_OFF;
static bool s_setup_done;

/* Defined further below (SSP sample channel); used by hal_rfid_set_mode. */
static void fpga_24mhz_clk(void);
static void fpga_ssc_setup(bool wide16);
static void ssp_clk_start(void);

static void ensure_setup(void)
{
    if (!s_setup_done) { fpga_cmd_setup(); s_setup_done = true; }
}

/* ---- HAL surface ---- */

uint32_t hal_rfid_caps(void) { return RFID_CAP_LF_READ | RFID_CAP_HF_READ | RFID_CAP_HF_EMU; }

int hal_rfid_set_mode(rfid_mode_t mode)
{
    ensure_setup();
    fpga_24mhz_clk();
    switch (mode) {
    case RFID_OFF:
        fpga_conf(FPGA_MAJOR_MODE_OFF);
        break;
    case RFID_LF_READER:
        /* Full PM5 LF configuration (field still off until hal_rfid_lf_field). */
        fpga_divisor(LF_DIVISOR_125);
        fpga_pwr_pwm(4095);                       /* antenna drive voltage = max */
        antenna_lf_on();                          /* I2C: 125 kHz antenna tap */
        adc_mux(false);                           /* PB5 low: LF peak-detector -> ADC */
        fpga_ssc_setup(false);                    /* SPI4 TI-slave, 8-bit */
        fpga_conf(FPGA_MAJOR_MODE_OFF);
        break;
    case RFID_HF_READER:
        /* HF 14443A: simpler analog path (no antenna I2C / PWR_PWM - the HF gateware
         * drives the 13.56 MHz field itself). PB5 HIGH = HF peak-detector -> ADC. */
        adc_mux(true);
        fpga_ssc_setup(false);                    /* 8-bit TI-slave (14443A reader) */
        fpga_conf(FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_14A_READER_LISTEN);  /* field on */
        spin_ms(50);                              /* antenna + card power-up settle */
        break;
    case RFID_HF_EMU:
        /* 14443-A card emulation: same 8-bit SSP, HF ADC mux. The gateware's
         * TAGSIM modes demodulate the reader (Miller) and load-modulate our reply. */
        adc_mux(true);
        fpga_ssc_setup(false);
        fpga_conf(FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_14A_TAGSIM_LISTEN);
        spin_ms(50);                              /* demod settle (matches the PM3 iso14443a_setup) */
        ssp_clk_start();                          /* after conf, so FRAME/SCK toggle for the sync */
        break;
    default:
        return RFID_ERR_UNSUPP;
    }
    s_mode = mode;
    return 0;
}

void hal_rfid_field(bool on)
{
    ensure_setup();
    if (s_mode == RFID_LF_READER)
        fpga_conf(on ? (FPGA_MAJOR_MODE_LF_READER | FPGA_LF_ADC_READER_FIELD)
                     : FPGA_MAJOR_MODE_OFF);
    else if (s_mode == RFID_HF_READER)
        fpga_conf(on ? (FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_14A_READER_LISTEN)
                     : FPGA_MAJOR_MODE_OFF);
}

int hal_rfid_lf_field(bool on, uint32_t divisor)
{
    ensure_setup();
    fpga_divisor(divisor ? (uint8_t)divisor : LF_DIVISOR_125);
    fpga_conf(on ? (FPGA_MAJOR_MODE_LF_READER | FPGA_LF_ADC_READER_FIELD)
                 : FPGA_MAJOR_MODE_OFF);
    return 0;
}

/* ================= SSP sample channel ================= */

/* The GW1N gateware's RF logic runs off a 24 MHz clock the MCU provides on PA8
 * (CLKOUT1 = PLL 288 MHz / 3 / 4). Without it the frontend is configured but not
 * clocked, so no samples flow. Idempotent. */
static bool s_clk_done;
static void fpga_24mhz_clk(void)
{
    if (s_clk_done) return;
    CRM->AHBEN1 |= CRM_AHBEN1_GPIOAEN;
    (void)CRM->AHBEN1;
    gpio_set_mux(GPIOA, 8, MUX_CLKOUT1);
    gpio_set_mode(GPIOA, 8, GPIO_MODE_MUX);
    CRM->CFG = (CRM->CFG & ~(CRM_CFG_CLKOUT1SEL_MSK | CRM_CFG_CLKOUT1DIV1_MSK))
             | CRM_CFG_CLKOUT1SEL_PLL | CRM_CFG_CLKOUT1DIV1_3;
    CRM->MISC1 = (CRM->MISC1 & ~CRM_MISC1_CLKOUT1DIV2_MSK) | CRM_MISC1_CLKOUT1DIV2_4;
    s_clk_done = true;
}

/* SPI4 as a TI/"SSP" slave: the FPGA is master and drives FRAME(PB6)+CLK(PB7);
 * samples arrive on MOSI(PB9). 8-bit frames for LF, 16-bit (I/Q) for HF reader. */
static void fpga_ssc_setup(bool wide16)
{
    CRM->APB2EN |= CRM_APB2EN_SPI4EN;
    CRM->AHBEN1 |= CRM_AHBEN1_GPIOBEN;
    (void)CRM->APB2EN;
    for (int p = 6; p <= 9; p++) {
        gpio_set_mux(GPIOB, p, MUX_SPI34);
        gpio_set_mode(GPIOB, p, GPIO_MODE_MUX);
    }
    SPI4->CTRL1 = 0;                                   /* disable + slave (MSTEN=0), MSB, mode 0 */
    SPI4->CTRL1 = wide16 ? SPI_CTRL1_FBN : 0;          /* 16- vs 8-bit frame */
    SPI4->CTRL2 = SPI_CTRL2_TIEN;                      /* TI (frame-synchronised) mode */
    SPI4->CTRL1 |= SPI_CTRL1_SPIEN;
}

/* ---- ssp_clk counter: TMR2 CH2 (PB3, MUX1) in external-clock mode A, counting
 * the FPGA's ssp_clk. The HF-emu tag reply must be fed at an 8-tick (byte) SSP
 * boundary or the FPGA's delay-line release is mis-phased and the reader can't
 * decode it - emu_send phase-locks on this before feeding (fork armsrc/iso14443a.c
 * EmSendCmd14443aRaw). Free-running; a fixed phase offset vs the frame is fine. */
static void ssp_clk_start(void)
{
    CRM->APB1EN |= CRM_APB1EN_TMR2EN;
    (void)CRM->APB1EN;
    CRM->AHBEN1 |= CRM_AHBEN1_GPIOBEN;
    (void)CRM->AHBEN1;
    gpio_set_mux(GPIOB, 3, 1);                          /* PB3 -> MUX1 = TMR2_CH2 */
    gpio_set_mode(GPIOB, 3, GPIO_MODE_MUX);

    /* Matches the fork's StartCountSspClk (common_arm/ticks/ticks_hw_at32.c): TMR2 CH2
     * on PB3, TI2 direct with a digital filter (C2DF2), rising edge, external-clock
     * mode A, 32-bit, count up. No frame sync: the counter free-runs from an arbitrary
     * zero and emu_send's `& 7` locks to a consistent phase the FPGA delay-line absorbs. */
    TMR2->CTRL1 = 0;                                    /* stop; count up */
    TMR2->CM1   = (TMR2->CM1 & ~0x0000FF00u) | (1u << 8) | (2u << 12); /* C2C=01 (TI2), C2DF=2 (filter) */
    TMR2->CCTRL &= ~(1u << 5);                          /* C2P=0: rising edge */
    TMR2->STCTRL = (6u << 4) | 7u;                      /* STIS=C2DF2, SMSEL=external clock mode A */
    TMR2->CTRL1 |= TMR_CTRL1_PMEN;                      /* 32-bit */
    TMR2->DIV = 0;
    TMR2->PR  = 0xFFFFFFFFu;
    TMR2->CVAL = 0;
    TMR2->CTRL1 |= TMR_CTRL1_TMREN;                     /* enable, free-running */
}
static inline uint32_t ssp_clk_get(void) { return TMR2->CVAL; }
/* Phase-lock to an 8-tick SSP boundary (guarded so a stalled counter can't hang
 * the critical section). */
static inline void ssp_phase_lock(void)
{
    for (uint32_t g = 0; (ssp_clk_get() & 7u) && ++g < 2000000u; ) { }
}

/* DMA capture of `n` 8-bit samples: SPI4->DT -> buf via DMA1 channel 1 (DMAMUX
 * SPI4_RX), single-shot. Gap-free (unlike polling), so no first-shot overrun.
 * Returns true if the full transfer completed within the deadline. */
static bool ssc_dma_capture(uint8_t *buf, uint16_t n)
{
    CRM->AHBEN1 |= CRM_AHBEN1_DMA1EN;
    (void)CRM->AHBEN1;

    dma_channel_type *ch = &DMA1->CH[0];       /* channel 1 */
    ch->CTRL = 0;                              /* disable while configuring */
    DMA1->CLR = DMA_CLR_CH1;
    ch->PADDR = (uint32_t)&SPI4->DT;
    ch->MADDR = (uint32_t)buf;
    ch->DTCNT = n;
    /* peripheral->memory (DTD=0), mem-increment, periph-fixed, 8-bit (widths=0),
     * very-high priority, single-shot (no LM). */
    ch->CTRL = DMA_CTRL_MINCM | DMA_CTRL_CHPL_VHI;
    DMA1->MUXSEL |= DMA_MUXSEL_TBL_SEL;        /* flexible request table */
    DMA1->MUXCCTRL[0] = DMAMUX_REQ_SPI4_RX;    /* route SPI4_RX onto channel 1 */

    if (SPI4->STS & SPI_STS_RDBF) (void)SPI4->DT;   /* drain stale sample + clear overrun */
    (void)SPI4->STS;

    SPI4->CTRL2 |= SPI_CTRL2_DMAREN;           /* SPI4 RX generates DMA requests */
    ch->CTRL |= DMA_CTRL_CHEN;

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(300);
    while (!(DMA1->STS & DMA_STS_FDT1))
        if (xTaskGetTickCount() > deadline) break;
    bool ok = (DMA1->STS & DMA_STS_FDT1) != 0;

    ch->CTRL = 0;
    SPI4->CTRL2 &= ~SPI_CTRL2_DMAREN;
    DMA1->CLR = DMA_CLR_CH1;
    return ok;
}

/* ================= HF ISO14443-A reader =================
 * Miller-encode the reader->tag frame, modulate it out over SPI4 in READER_MOD,
 * switch to READER_LISTEN, and Manchester-decode the tag's subcarrier answer.
 * The encoder and decoder are the same algorithms as the PM3/Chameleon; only the
 * physical I/O (SSC -> SPI4) differs. */

static inline uint8_t oddparity8(uint8_t x) { return (uint8_t)(!__builtin_parity(x)); }

/* Encode a reader frame as modified-Miller SEC bytes (one per air bit) with a
 * parity bit after each *whole* byte. `par` supplies the parity bit per byte
 * (for MIFARE Crypto1 encrypted frames); NULL = compute odd parity. Returns the
 * SEC-byte count, or -1. */
static int code_14a_reader_ex(const uint8_t *cmd, int bits, const uint8_t *par,
                              uint8_t *ts, int cap)
{
    int last = 0, n = 0;
#define TS_PUSH(b) do { if (n >= cap) return -1; ts[n++] = (uint8_t)(b); } while (0)
    TS_PUSH(SEC_Z);                                 /* start of communication */
    int bytecount = (bits + 7) / 8;
    for (int i = 0; i < bytecount; i++) {
        uint8_t b = cmd[i];
        int bitsleft = bits - i * 8;
        if (bitsleft > 8) bitsleft = 8;
        int j;
        for (j = 0; j < bitsleft; j++) {            /* LSB first */
            if (b & 1)          { TS_PUSH(SEC_X); last = 1; }
            else if (last == 0) { TS_PUSH(SEC_Z); }
            else                { TS_PUSH(SEC_Y); last = 0; }
            b >>= 1;
        }
        if (j == 8) {                               /* parity only on a whole byte */
            uint8_t p = par ? (uint8_t)(par[i] & 1) : oddparity8(cmd[i]);
            if (p)              { TS_PUSH(SEC_X); last = 1; }
            else if (last == 0) { TS_PUSH(SEC_Z); }
            else                { TS_PUSH(SEC_Y); last = 0; }
        }
    }
    if (last == 0) TS_PUSH(SEC_Z);                   /* end of communication */
    else           TS_PUSH(SEC_Y);
    TS_PUSH(SEC_Y);
#undef TS_PUSH
    return n;
}
static int code_14a_reader(const uint8_t *cmd, int bits, uint8_t *ts, int cap)
{ return code_14a_reader_ex(cmd, bits, NULL, ts, cap); }

/* A nibble counts as subcarrier modulation if it has 3 or 4 set bits. */
static const uint8_t MANCH_LUT[16] = { 0,0,0,0,0,0,0,1, 0,0,0,1,0,1,1,1 };

static struct {
    int      state;         /* 0 = unsynced, 1 = manchester data */
    uint16_t twoBits;       /* sliding 2-byte window */
    uint16_t highCnt;
    uint16_t bitCount;
    uint16_t syncBit;
    uint16_t shiftReg;      /* 9 bits: data in [0:7], on-air parity in [8] */
    uint16_t len;
    int      output_len;
    uint8_t *output;
    uint8_t *par;           /* optional: received parity bit per byte (MIFARE) */
} s_demod;

static void demod_reset(uint8_t *out, uint8_t *par, int cap)
{
    s_demod.state = 0; s_demod.twoBits = 0xFFFF; s_demod.highCnt = 0;
    s_demod.bitCount = 0; s_demod.syncBit = 0xFFFF; s_demod.shiftReg = 0;
    s_demod.len = 0; s_demod.output = out; s_demod.output_len = cap;
    s_demod.par = par;
}
/* Commit a complete 9-bit group: data byte + its on-air parity bit. */
static inline void demod_store(void)
{
    if (s_demod.par) s_demod.par[s_demod.len] = (uint8_t)((s_demod.shiftReg >> 8) & 1);
    s_demod.output[s_demod.len++] = (uint8_t)(s_demod.shiftReg & 0xff);
    s_demod.bitCount = 0; s_demod.shiftReg = 0;
}

/* Feed one SPI4 RX byte. Syncs to the tag bit phase, then decodes Sequence D
 * (subcarrier in 1st half = '1') / E (2nd = '0') / F (none = end). Returns 1
 * when a complete frame is in s_demod.output. */
static int manchester_decode(uint8_t bit)
{
    if ((int)s_demod.len >= s_demod.output_len) return 1;
    s_demod.twoBits = (uint16_t)((s_demod.twoBits << 8) | bit);

    if (s_demod.state == 0) {
        if (s_demod.highCnt < 2) {
            if (s_demod.twoBits == 0x0000) s_demod.highCnt++;
            else s_demod.highCnt = 0;
            return 0;
        }
        uint16_t t = s_demod.twoBits, s = 0xFFFF;
        if      ((t & 0x7700) == 0x7000) s = 7;
        else if ((t & 0x3B80) == 0x3800) s = 6;
        else if ((t & 0x1DC0) == 0x1C00) s = 5;
        else if ((t & 0x0EE0) == 0x0E00) s = 4;
        else if ((t & 0x0770) == 0x0700) s = 3;
        else if ((t & 0x03B8) == 0x0380) s = 2;
        else if ((t & 0x01DC) == 0x01C0) s = 1;
        else if ((t & 0x00EE) == 0x00E0) s = 0;
        if (s != 0xFFFF) { s_demod.syncBit = s; s_demod.bitCount = 0; s_demod.state = 1; }
        return 0;
    }

    uint16_t w = (uint16_t)(s_demod.twoBits >> s_demod.syncBit);
    int mod1 = MANCH_LUT[(w & 0x00F0) >> 4];
    int mod2 = MANCH_LUT[w & 0x000F];
    if (mod1) {
        s_demod.bitCount++;
        s_demod.shiftReg = (uint16_t)((s_demod.shiftReg >> 1) | 0x100);
        if (s_demod.bitCount == 9) demod_store();
    } else if (mod2) {
        s_demod.bitCount++;
        s_demod.shiftReg = (uint16_t)(s_demod.shiftReg >> 1);
        if (s_demod.bitCount >= 9) demod_store();
    } else {
        if (s_demod.bitCount > 0) {                  /* truncated final byte (no parity) */
            s_demod.shiftReg = (uint16_t)(s_demod.shiftReg >> (9 - s_demod.bitCount));
            if (s_demod.par) s_demod.par[s_demod.len] = 0;
            s_demod.output[s_demod.len++] = (uint8_t)(s_demod.shiftReg & 0xff);
            return 1;
        }
        if (s_demod.len) return 1;
        demod_reset(s_demod.output, s_demod.par, s_demod.output_len);
    }
    return 0;
}

/* Drive one Miller-encoded reader->tag frame out over SPI4 (READER_MOD), then
 * Manchester-decode the tag's answer (READER_LISTEN). SPI4 is a TI slave clocked
 * by the FPGA, so we only busy-wait on TDBE/RDBF. IRQs off for the exchange: one
 * SSP byte is ~9 us and a dropped byte breaks the (drop-intolerant) decode. */
static int hf_txrx(const uint8_t *tosend, int tlen, uint8_t *rx, int rx_cap, uint8_t *rx_par)
{
    vTaskDelay(pdMS_TO_TICKS(1));                    /* inter-frame guard (>= FDT) */

    taskENTER_CRITICAL();
    /* drain RX backlog + clear any overrun latched during the idle LISTEN period. */
    for (int i = 0; i < 64 && (SPI4->STS & SPI_STS_RDBF); i++) (void)SPI4->DT;
    (void)SPI4->STS;
    fpga_conf(FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_14A_READER_MOD);   /* transmit */
    if (SPI4->STS & SPI_STS_TDBE) SPI4->DT = SEC_Y;  /* prime: flush an idle byte first */
    for (int c = 0; c < tlen; ) {
        uint32_t w = 0;
        while (!(SPI4->STS & SPI_STS_TDBE) && ++w < 200000u) { }
        if (!(SPI4->STS & SPI_STS_TDBE)) { taskEXIT_CRITICAL(); return RFID_ERR_FRAMING; }
        SPI4->DT = tosend[c++];
    }
    for (uint32_t w = 0; (SPI4->STS & SPI_STS_BF) && ++w < 200000u; ) { }   /* drain last byte */

    fpga_conf(FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_14A_READER_LISTEN);/* receive */
    demod_reset(rx, rx_par, rx_cap);
    if (SPI4->STS & SPI_STS_RDBF) (void)SPI4->DT;    /* drop stale self-field byte */
    (void)SPI4->STS;
    int done = 0;
    for (int nb = 0; !done && nb < HF_RX_ABORT_BYTES; nb++) {
        uint32_t w = 0;
        while (!(SPI4->STS & SPI_STS_RDBF) && ++w < 4000) { }
        if (!(SPI4->STS & SPI_STS_RDBF)) break;
        done = manchester_decode((uint8_t)SPI4->DT);
        if (!done && s_demod.state == 0 && nb >= HF_RX_NOANSWER_BYTES) break;
    }
    taskEXIT_CRITICAL();

    if (!done || s_demod.len == 0) return RFID_ERR_TIMEOUT;
    return (int)s_demod.len * 8;
}

int hal_rfid_hf_transceive(const uint8_t *tx, int tx_bits, uint32_t flags,
                           uint8_t *rx, int rx_cap, uint32_t timeout_us)
{
    (void)flags; (void)timeout_us;
    if (s_mode != RFID_HF_READER || !tx || tx_bits <= 0 || !rx || rx_cap <= 0)
        return RFID_ERR_UNSUPP;
    static uint8_t tosend[96];
    int tlen = code_14a_reader(tx, tx_bits, tosend, (int)sizeof tosend);
    if (tlen <= 0) return RFID_ERR_FRAMING;
    return hf_txrx(tosend, tlen, rx, rx_cap, NULL);
}

/* HF transceive carrying explicit per-byte parity in and out - MIFARE Crypto1
 * sends encrypted frames whose parity is part of the cipher, not odd parity. */
int hal_rfid_hf_transceive_par(const uint8_t *tx, int nbytes, const uint8_t *par,
                               uint8_t *rx, uint8_t *rx_par, int rx_cap, uint32_t timeout_us)
{
    (void)timeout_us;
    if (s_mode != RFID_HF_READER || !tx || nbytes <= 0 || !par || !rx || rx_cap <= 0)
        return RFID_ERR_UNSUPP;
    static uint8_t tosend[96];
    int tlen = code_14a_reader_ex(tx, nbytes * 8, par, tosend, (int)sizeof tosend);
    if (tlen <= 0) return RFID_ERR_FRAMING;
    return hf_txrx(tosend, tlen, rx, rx_cap, rx_par);
}

int hal_rfid_hf_probe(void) { return -1; }

/* ================= HF 14443-A card emulation ================= *
 * The gateware's TAGSIM modes demodulate the reader (Miller, TAGSIM_LISTEN) and
 * load-modulate our reply (subcarrier, TAGSIM_MOD). Ported from the PM3 emu path;
 * the SSC is replaced by SPI4. The PM3 phase-locks the reply to an 8-tick SSP
 * boundary with a chained-timer ssp_clk counter - on the PM5 the SPI4 is byte-
 * framed by the FPGA, so a byte boundary *is* an 8-tick boundary and the framing
 * gives that alignment for free (no separate counter). */

/* nibble -> "is this 4-tick half a modulation?" (stock Mod_Miller LUT) */
static const uint8_t MILLER_LUT[16] = { 0,1,0,1,0,0,0,1,0,1,0,0,0,0,0,0 };

enum { U_UNSYNCD = 0, U_SOC, U_MILLER_X, U_MILLER_Y, U_MILLER_Z };

static struct sn_uart {                            /* reader->tag (Miller) decoder */
    int state, syncBit, bitCount, len, cap;
    uint32_t fourBits, startTime, endTime, shiftReg, parerr;
    uint8_t *out;
} s_u;

static void sn_uart_reset(void)
{
    s_u.state = U_UNSYNCD; s_u.fourBits = 0; s_u.syncBit = -1;
    s_u.bitCount = 0; s_u.len = 0; s_u.shiftReg = 0; s_u.parerr = 0;
    s_u.startTime = 0; s_u.endTime = 0;
}

/* store a completed 9-bit group: data byte + odd-parity-violation flag into parerr */
#define SN_STORE(S, SHIFT) do { \
    uint8_t _b = (uint8_t)((SHIFT) & 0xff); int _i = (S).len; \
    if ((S).len < (S).cap) (S).out[(S).len++] = _b; \
    if (_i < 32 && (((SHIFT) >> 8) & 1) != oddparity8(_b)) (S).parerr |= (1u << _i); \
    (S).bitCount = 0; (S).shiftReg = 0; \
} while (0)

/* Feed one reader byte (8 Miller ticks). Returns 1 on a complete frame. */
static int sn_miller(uint8_t bit, uint32_t t)
{
    if (s_u.len >= s_u.cap) return 1;
    s_u.fourBits = (s_u.fourBits << 8) | bit;

    if (s_u.state == U_UNSYNCD) {
        s_u.syncBit = -1;
        for (int sh = 0; sh <= 7; sh++)
            if ((s_u.fourBits & (0x07FFEF80u >> sh)) == (0x07FF8F80u >> sh)) { s_u.syncBit = 7 - sh; break; }
        if (s_u.syncBit >= 0) { s_u.startTime = t - (uint32_t)s_u.syncBit; s_u.endTime = s_u.startTime; s_u.state = U_SOC; }
        return 0;
    }

    uint32_t w = s_u.fourBits >> s_u.syncBit;
    int m1 = MILLER_LUT[(w >> 4) & 0x0F], m2 = MILLER_LUT[w & 0x0F];

    if (m1 && m2) { sn_uart_reset(); return 0; }        /* modulation both halves -> error */
    if (m1) {                                           /* Sequence Z = logic 0 */
        if (s_u.state == U_MILLER_X) { sn_uart_reset(); return 0; }
        s_u.bitCount++; s_u.shiftReg >>= 1; s_u.state = U_MILLER_Z;
        s_u.endTime = s_u.startTime + 8u * (9u * s_u.len + s_u.bitCount + 1) - 6;
        if (s_u.bitCount >= 9) SN_STORE(s_u, s_u.shiftReg);
    } else if (m2) {                                    /* Sequence X = logic 1 */
        s_u.bitCount++; s_u.shiftReg = (s_u.shiftReg >> 1) | 0x100; s_u.state = U_MILLER_X;
        s_u.endTime = s_u.startTime + 8u * (9u * s_u.len + s_u.bitCount + 1) - 2;
        if (s_u.bitCount >= 9) SN_STORE(s_u, s_u.shiftReg);
    } else {                                            /* no modulation -> Sequence Y */
        if (s_u.state == U_MILLER_Z || s_u.state == U_MILLER_Y) {   /* Y after 0 = end of comm */
            s_u.state = U_UNSYNCD; s_u.bitCount--; s_u.shiftReg <<= 1;
            if (s_u.bitCount > 0) {                     /* trailing partial byte (no parity) */
                s_u.shiftReg >>= (9 - s_u.bitCount);
                if (s_u.len < s_u.cap) s_u.out[s_u.len++] = (uint8_t)(s_u.shiftReg & 0xff);
                return 1;
            }
            return s_u.len ? 1 : (sn_uart_reset(), 0);
        }
        if (s_u.state == U_SOC) { sn_uart_reset(); return 0; }      /* Y right after SOC = error */
        s_u.bitCount++; s_u.shiftReg >>= 1; s_u.state = U_MILLER_Y; /* a logic 0 */
        if (s_u.bitCount >= 9) SN_STORE(s_u, s_u.shiftReg);
    }
    return 0;
}

/* ================= HF 14443-A passive sniffer ================= *
 * In FPGA SNIFFER mode the gateware demodulates both sides of a nearby
 * transaction and streams them de-interleaved over the SSP sample bus (two 8-bit
 * words per bit period: the reader Miller nibble in one, the tag Manchester
 * nibble in the other). We keep no field of our own. The reader side reuses the
 * Miller decoder above (s_u/sn_miller); the tag side needs its own Manchester
 * decoder (s_dm/sn_manch, identical to the reader-RX manchester_decode but with
 * per-frame timing + parity, matching the PM3 sniff). Ported from the PM3 port's
 * hal_rfid_hf_sniff_capture; the AT91 SSC+PDC ring becomes SPI4 + a DMA1 circular
 * ring, everything above the sample transport is unchanged. */

enum { D_UNSYNCD = 0, D_DATA };

static struct sn_demod {                           /* tag->reader (Manchester) decoder */
    int state, syncBit, bitCount, len, cap;
    uint16_t twoBits, highCnt, shiftReg;
    uint32_t startTime, endTime, parerr;
    uint8_t *out;
} s_dm;

static void sn_demod_reset(void)
{
    s_dm.state = D_UNSYNCD; s_dm.twoBits = 0xFFFF; s_dm.highCnt = 0;
    s_dm.bitCount = 0; s_dm.syncBit = -1; s_dm.shiftReg = 0; s_dm.len = 0;
    s_dm.startTime = 0; s_dm.endTime = 0; s_dm.parerr = 0;
}

/* Feed one de-interleaved tag byte (8 Manchester ticks). Returns 1 on a complete
 * frame. MANCH_LUT (defined above for the reader-RX decoder) is the PM3 MANCH_LUT2. */
static int sn_manch(uint8_t bit, uint32_t t)
{
    if (s_dm.len >= s_dm.cap) return 1;
    s_dm.twoBits = (uint16_t)((s_dm.twoBits << 8) | bit);

    if (s_dm.state == D_UNSYNCD) {
        if (s_dm.highCnt < 2) {                         /* wait for a stable unmodulated run */
            if (s_dm.twoBits == 0x0000) s_dm.highCnt++; else s_dm.highCnt = 0;
            return 0;
        }
        int sb = -1; uint16_t tb = s_dm.twoBits;
        if      ((tb & 0x7700) == 0x7000) sb = 7;
        else if ((tb & 0x3B80) == 0x3800) sb = 6;
        else if ((tb & 0x1DC0) == 0x1C00) sb = 5;
        else if ((tb & 0x0EE0) == 0x0E00) sb = 4;
        else if ((tb & 0x0770) == 0x0700) sb = 3;
        else if ((tb & 0x03B8) == 0x0380) sb = 2;
        else if ((tb & 0x01DC) == 0x01C0) sb = 1;
        else if ((tb & 0x00EE) == 0x00E0) sb = 0;
        if (sb >= 0) {
            s_dm.syncBit = sb; s_dm.startTime = t - (uint32_t)sb;
            s_dm.bitCount = 0; s_dm.state = D_DATA;
        }
        return 0;
    }

    uint16_t w = (uint16_t)(s_dm.twoBits >> s_dm.syncBit);
    int m1 = MANCH_LUT[(w >> 4) & 0x0F], m2 = MANCH_LUT[w & 0x0F];

    if (m1) {                                           /* Sequence D = 1 (m1&&m2 collision, still 1) */
        s_dm.bitCount++; s_dm.shiftReg = (uint16_t)((s_dm.shiftReg >> 1) | 0x100);
        if (s_dm.bitCount == 9) SN_STORE(s_dm, s_dm.shiftReg);
        s_dm.endTime = s_dm.startTime + 8u * (9u * s_dm.len + s_dm.bitCount + 1) - 4;
    } else if (m2) {                                    /* Sequence E = 0 */
        s_dm.bitCount++; s_dm.shiftReg = (uint16_t)(s_dm.shiftReg >> 1);
        if (s_dm.bitCount >= 9) SN_STORE(s_dm, s_dm.shiftReg);
        s_dm.endTime = s_dm.startTime + 8u * (9u * s_dm.len + s_dm.bitCount + 1);
    } else {                                            /* no modulation -> end of comm */
        if (s_dm.bitCount > 0) {
            s_dm.shiftReg = (uint16_t)(s_dm.shiftReg >> (9 - s_dm.bitCount));
            if (s_dm.len < s_dm.cap) s_dm.out[s_dm.len++] = (uint8_t)(s_dm.shiftReg & 0xff);
            return 1;
        }
        if (s_dm.len) return 1;
        sn_demod_reset();
    }
    return 0;
}

/* Append a base-10 uint32 (no libc, runs inside the sample loop's emit path). */
static char *sn_u32(char *p, uint32_t v)
{
    char tb[10]; int k = 0;
    do { tb[k++] = (char)('0' + v % 10u); v /= 10u; } while (v);
    while (k) *p++ = tb[--k];
    return p;
}

/* Emit one decoded frame as a host sniff line: "<R|C> <start> <end> <hex>[!]..".
 * Hand-rolled (no snprintf): cheap enough to keep the sample loop from overrunning. */
static int sn_emit(char *txt, int pos, int cap, char dir, const uint8_t *b, int n,
                   uint32_t start, uint32_t end, uint32_t parerr)
{
    static const char HEX[] = "0123456789ABCDEF";
    if (n <= 0 || pos > cap - (24 + n * 4)) return pos;   /* worst case: 2x10-digit ts + " XX!" per byte */
    char *p = txt + pos;
    *p++ = dir; *p++ = ' ';
    p = sn_u32(p, start); *p++ = ' ';
    p = sn_u32(p, end);
    for (int i = 0; i < n; i++) {
        *p++ = ' '; *p++ = HEX[b[i] >> 4]; *p++ = HEX[b[i] & 0x0F];
        if ((parerr >> i) & 1) *p++ = '!';
    }
    *p++ = '\n';
    return (int)(p - txt);
}

#define SNIFF_HDR       40     /* bytes reserved at the front of txt for the prepended L-header */
#define SNIFF_DMA_SIZE  3584   /* SPI4->mem circular capture ring (matches the PM3 PDC ring) */
#define SNIFF_FRAME_CAP 64     /* max decoded bytes per frame (a READ response is 18) */
#define SNIFF_QUIET_MS  250    /* a transaction is "done" once this long passes with no new frame */
#define SNIFF_MAX_MS    2500   /* hard cap on one active capture regardless of quiet */
#define SNIFF_CYCLE_MS  50     /* mid-capture gap that marks a new field cycle (keep the newest) */

/* Keep the decoded-text buffer to its most recent whole lines when near-full, so a long
 * transaction's trailing READ burst always survives (drop the oldest ~third). */
static int sn_compact(char *txt, int pos, int cap)
{
    if (pos <= cap - (24 + SNIFF_FRAME_CAP * 4)) return pos;   /* room for one more worst-case frame */
    int cut = SNIFF_HDR + (pos - SNIFF_HDR) / 3;
    while (cut < pos && txt[cut] != '\n') cut++;               /* cut on a line boundary */
    if (cut < pos) cut++;
    memmove(txt + SNIFF_HDR, txt + cut, (size_t)(pos - cut));
    return SNIFF_HDR + (pos - cut);
}

/* Arm DMA1 channel 1 as a self-reloading (circular) ring: SPI4->DT -> ring[0..n).
 * The write cursor is n - ch->DTCNT (DTCNT counts remaining, reloads to n each lap). */
static void sniff_dma_start(uint8_t *ring, uint16_t n)
{
    CRM->AHBEN1 |= CRM_AHBEN1_DMA1EN;
    (void)CRM->AHBEN1;
    dma_channel_type *ch = &DMA1->CH[0];       /* channel 1 */
    ch->CTRL = 0;
    DMA1->CLR = DMA_CLR_CH1;
    ch->PADDR = (uint32_t)&SPI4->DT;
    ch->MADDR = (uint32_t)ring;
    ch->DTCNT = n;
    /* peripheral->memory, mem-increment, periph-fixed, 8-bit, very-high priority, circular (LM). */
    ch->CTRL = DMA_CTRL_MINCM | DMA_CTRL_LM | DMA_CTRL_CHPL_VHI;
    DMA1->MUXSEL |= DMA_MUXSEL_TBL_SEL;
    DMA1->MUXCCTRL[0] = DMAMUX_REQ_SPI4_RX;
    if (SPI4->STS & SPI_STS_RDBF) (void)SPI4->DT;   /* drain stale sample + clear overrun */
    (void)SPI4->STS;
    SPI4->CTRL2 |= SPI_CTRL2_DMAREN;
    ch->CTRL |= DMA_CTRL_CHEN;
}

int hal_rfid_hf_sniff_capture(uint8_t *buf, uint32_t cap_bytes, uint32_t quiet_ms, uint32_t max_ms)
{
    if (s_mode != RFID_HF_READER || !buf) return RFID_ERR_UNSUPP;
    if (cap_bytes < FANTASI_RFID_SNIFF_BUFSZ) return RFID_ERR_UNSUPP;   /* too small to carve safely */
    if (!quiet_ms) quiet_ms = SNIFF_QUIET_MS;
    if (!max_ms)   max_ms   = SNIFF_MAX_MS;

    /* Carve the caller's ephemeral buffer: cap(text) + dma(ring) + ub + db == SNIFF_BUFSZ (5760). */
    const int cap = 2048;
    char    *txt = (char *)buf;
    uint8_t *dma = (uint8_t *)(((uintptr_t)buf + cap + 3) & ~(uintptr_t)3);   /* 4-align for DMA */
    uint8_t *ub  = dma + SNIFF_DMA_SIZE;
    uint8_t *db  = ub + SNIFF_FRAME_CAP;

    adc_mux(true);                                 /* HF peak-detector -> ADC */
    fpga_ssc_setup(false);                         /* 8-bit MSB-first framing (sniffer word) */
    fpga_conf(FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_14A_SNIFFER);   /* passive - no field */
    spin_ms(8);                                    /* let the peak-detector bias settle once */

    dma_channel_type *ch = &DMA1->CH[0];
    sniff_dma_start(dma, SNIFF_DMA_SIZE);

    s_u.out  = ub; s_u.cap  = SNIFF_FRAME_CAP; sn_uart_reset();
    s_dm.out = db; s_dm.cap = SNIFF_FRAME_CAP; sn_demod_reset();

    int pos = SNIFF_HDR;                           /* reserve the front for the L-header */
    int valid = 0, saw_activity = 0;
    uint32_t rx_samples = 0;
    uint8_t *data = dma, previous = 0;
    int TagActive = 0, ReaderActive = 0;

    TickType_t t0 = xTaskGetTickCount(), last_active = t0;

    for (;;) {
        TickType_t now = xTaskGetTickCount();
        if (saw_activity) {
            /* stay in one call for a whole cascade->AUTH->READ burst (gaps < quiet_ms),
             * but always bail out by max_ms so a continuous field can't hang us */
            if ((now - last_active) >= pdMS_TO_TICKS(quiet_ms)) break;
            if ((now - t0)        >= pdMS_TO_TICKS(max_ms))     break;
        } else if ((now - t0) >= pdMS_TO_TICKS(quiet_ms)) {
            break;                                 /* idle: return so the module can poll for a stop keypress */
        }

        int wr = SNIFF_DMA_SIZE - (int)ch->DTCNT;  /* circular write cursor */
        int rd = (int)(data - dma);
        int avail = (rd <= wr) ? (wr - rd) : (SNIFF_DMA_SIZE - rd + wr);

        if (avail == 0)                            /* caught up with the DMA: spin (return decided at top) */
            continue;
        if (avail > 9 * SNIFF_DMA_SIZE / 10) {     /* fell behind the writer: skip to the head */
            data = dma + wr; rx_samples += avail; sn_uart_reset(); sn_demod_reset();
            continue;
        }

        for (int i = 0; i < avail; i++) {
            uint8_t cur = *data;
            if (rx_samples & 1) {                  /* two SSP words de-interleave into one byte per direction */
                if (!TagActive) {
                    uint8_t rdr = (uint8_t)((previous & 0xF0) | (cur >> 4));
                    if (sn_miller(rdr, (rx_samples - 1) * 4)) {
                        if (s_u.len > 0) {
                            TickType_t fn = xTaskGetTickCount();
                            if (saw_activity && (fn - last_active) >= pdMS_TO_TICKS(SNIFF_CYCLE_MS))
                                pos = SNIFF_HDR;               /* new field cycle: keep only the newest */
                            pos = sn_compact(txt, pos, cap);
                            pos = sn_emit(txt, pos, cap, 'R', ub, s_u.len,
                                          s_u.startTime, s_u.endTime, s_u.parerr);
                            valid++; saw_activity = 1; last_active = fn;
                        }
                        sn_uart_reset(); sn_demod_reset();
                    }
                    ReaderActive = (s_u.state != U_UNSYNCD);
                }
                if (!ReaderActive) {
                    uint8_t tag = (uint8_t)((previous << 4) | (cur & 0x0F));
                    if (sn_manch(tag, (rx_samples - 1) * 4)) {
                        if (s_dm.len > 0) {
                            TickType_t fn = xTaskGetTickCount();
                            if (saw_activity && (fn - last_active) >= pdMS_TO_TICKS(SNIFF_CYCLE_MS))
                                pos = SNIFF_HDR;               /* new field cycle: keep only the newest */
                            pos = sn_compact(txt, pos, cap);
                            pos = sn_emit(txt, pos, cap, 'C', db, s_dm.len,
                                          s_dm.startTime, s_dm.endTime, s_dm.parerr);
                            valid++; saw_activity = 1; last_active = fn;
                        }
                        sn_uart_reset(); sn_demod_reset();
                    }
                    TagActive = (s_dm.state != D_UNSYNCD);
                }
            }
            previous = cur;
            rx_samples++;
            if (++data == dma + SNIFF_DMA_SIZE) data = dma;
        }
    }

    /* Stop DMA; leave the front-end in READER_LISTEN so a follow-up raw/search is ready. */
    ch->CTRL = 0;
    SPI4->CTRL2 &= ~SPI_CTRL2_DMAREN;
    DMA1->CLR = DMA_CLR_CH1;
    fpga_conf(FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_14A_READER_LISTEN);

    if (valid == 0) return 0;                      /* nothing decoded: the module prints nothing */

    /* The L-header frame count is what survived in the buffer (per-cycle clear + compaction drop
     * earlier lines), not the total ever decoded. Count the frame lines actually left. */
    int shown = 0;
    for (int k = SNIFF_HDR; k < pos; k++) if (txt[k] == '\n') shown++;

    /* Prepend the L-header (host CLI tabulates the frames): p1180 = each timestamp tick is 1180 ns. */
    char h[SNIFF_HDR]; char *hp = h;
    *hp++ = 'L'; *hp++ = '9'; *hp++ = ' '; *hp++ = 'v';
    hp = sn_u32(hp, shown ? shown : 1);
    *hp++ = ' '; *hp++ = 'p'; *hp++ = '1'; *hp++ = '1'; *hp++ = '8'; *hp++ = '0'; *hp++ = '\n';
    int hl = (int)(hp - h);
    int off = SNIFF_HDR - hl, len = pos - off;
    memcpy(txt + off, h, (size_t)hl);
    memmove(txt, txt + off, (size_t)len);
    return len;
}

static uint16_t s_queue_delay;   /* FPGA send-queue depth reported back over SSP (for the flush) */

/* Receive one reader command as a tag (Miller over TAGSIM_LISTEN). Fills rx[] and
 * rx_par[i] = the on-air parity bit of byte i; returns the byte count, 0 on
 * timeout (reader idle), or <0. Retains s_u so hf_emu_send picks the FDT
 * correction bit. Must be followed by hf_emu_send/send_stream. */
int hal_rfid_hf_emu_recv(uint8_t *rx, uint8_t *rx_par, int cap, uint32_t timeout_ms)
{
    if (s_mode != RFID_HF_EMU || !rx || !rx_par || cap <= 0) return RFID_ERR_UNSUPP;

    fpga_conf(FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_14A_TAGSIM_LISTEN);
    s_u.out = rx; s_u.cap = cap; sn_uart_reset();
    for (int f = 0; f < 64 && (SPI4->STS & SPI_STS_RDBF); f++) (void)SPI4->DT;  /* flush stale */

    int n = 0;

    /* PHASE 1 - catch an imminent command IRQs-off (the reader's next command lands
     * ~1 ms after our reply; a preemption there overruns the 1-byte SPI RX). */
    taskENTER_CRITICAL();
    {
        uint32_t spins = 0;
        for (;;) {
            if (SPI4->STS & SPI_STS_RDBF) {
                if (sn_miller((uint8_t)SPI4->DT, 0)) { n = (int)s_u.len; break; }
            }
            if (++spins > 45000u && s_u.state == U_UNSYNCD) break;   /* nothing arriving */
            if (spins > 600000u) break;                              /* never hold crit forever */
        }
    }
    taskEXIT_CRITICAL();

    /* PHASE 2 - wait the rest of the timeout preemptibly; decode IRQs-off once a
     * command starts. A missed idle poll (REQA/WUPA) is simply retried by the reader. */
    if (!n && s_u.state == U_UNSYNCD) {
        TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms ? timeout_ms : 200);
        uint32_t spins = 0;
        int crit = 0;
        for (;;) {
            spins++;
            if (crit) { if (spins > 300000u) break; }
            else if ((spins & 0xFFF) == 0) { if (xTaskGetTickCount() > deadline) break; }
            if (SPI4->STS & SPI_STS_RDBF) {
                int done = sn_miller((uint8_t)SPI4->DT, 0);
                if (!crit && s_u.state != U_UNSYNCD) { taskENTER_CRITICAL(); crit = 1; spins = 0; }
                if (done) { n = (int)s_u.len; break; }
            }
        }
        if (crit) taskEXIT_CRITICAL();
    }

    for (int i = 0; i < n && i < 32; i++)
        rx_par[i] = oddparity8(rx[i]) ^ (uint8_t)((s_u.parerr >> i) & 1);
    return n;
}

/* Whether to keep the leading FDT correction symbol (-> 1236 fc) or drop it
 * (-> 1172), from the last received frame's final bit (see EmSendCmd14443aRaw). */
static bool emu_correction(void)
{
    if (s_u.len == 1) return (s_u.out[0] & 0x40) != 0;
    int li = s_u.len - 1;
    return (oddparity8(s_u.out[li]) ^ (uint8_t)((s_u.parerr >> li) & 1)) != 0;
}

/* Wait (IRQs already off) for the FPGA fdt_indicator: a non-zero SPI4 RX byte. */
static void emu_wait_fdt(void)
{
    uint32_t w;
    for (w = 0; !(SPI4->STS & SPI_STS_RDBF) && ++w < 200000u; ) { }
    (void)SPI4->DT;
    for (uint8_t j = 0; j < 5; j++) {
        for (w = 0; !(SPI4->STS & SPI_STS_RDBF) && ++w < 200000u; ) { }
        if (SPI4->DT) break;
    }
}

static void emu_feed(uint8_t sym)   /* feed one subcarrier symbol; read back the queue depth */
{
    uint32_t w;
    for (w = 0; !(SPI4->STS & SPI_STS_TDBE) && ++w < 200000u; ) { }
    SPI4->DT = sym;
    for (w = 0; !(SPI4->STS & SPI_STS_RDBF) && ++w < 200000u; ) { }   /* TX/RX lockstep */
    s_queue_delay = (uint8_t)SPI4->DT;
}

static void emu_drain(void)         /* push idle symbols so the last reply symbol exits the coil */
{
    uint8_t queued = (uint8_t)(s_queue_delay >> 3);
    int flush = (queued >> 3) + 1; if (flush < 6) flush = 6;
    for (int k = 0; k < flush; k++) emu_feed(0x00);
}

/* Load-modulate a pre-encoded reply (one byte per subcarrier symbol) at the ISO14443-A
 * frame delay time. Must directly follow hf_emu_recv. */
int hal_rfid_hf_emu_send(const uint8_t *tosend, int len)
{
    if (s_mode != RFID_HF_EMU || !tosend || len <= 0) return RFID_ERR_UNSUPP;
    fpga_conf(FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_14A_TAGSIM_MOD);
    int i = emu_correction() ? 0 : 1;

    taskENTER_CRITICAL();
    emu_wait_fdt();
    ssp_phase_lock();                                /* align the feed to an 8-tick SSP boundary */
    if (SPI4->STS & SPI_STS_TDBE) SPI4->DT = 0x00;   /* prime */
    for (; i < len; i++) emu_feed(tosend[i]);
    emu_drain();
    taskEXIT_CRITICAL();
    return 0;
}

/* Streaming reply: each symbol fetched from next(ctx) just before it's fed, so a
 * module can compute a Crypto1-encrypted answer one symbol per ~9.4 us feed gap. */
int hal_rfid_hf_emu_send_stream(uint8_t (*next)(void *ctx), void *ctx, int nsymbols)
{
    if (s_mode != RFID_HF_EMU || !next || nsymbols <= 0) return RFID_ERR_UNSUPP;
    fpga_conf(FPGA_MAJOR_MODE_HF_ISO14443A | FPGA_HF_14A_TAGSIM_MOD);
    bool correction = emu_correction();

    taskENTER_CRITICAL();
    emu_wait_fdt();
    ssp_phase_lock();                                /* align the feed to an 8-tick SSP boundary */
    if (SPI4->STS & SPI_STS_TDBE) SPI4->DT = 0x00;   /* prime */
    if (correction) emu_feed(0x08);                  /* keep the correction symbol -> 1236 FDT */
    for (int k = 0; k < nsymbols; k++) emu_feed(next(ctx));
    emu_drain();
    taskEXIT_CRITICAL();
    return 0;
}

/* Stream the LF envelope from SPI4 and turn it into inter-edge intervals (in
 * sample counts) for the EM4100 decoder - the same software demod as the PM3/
 * Chameleon: boxcar-smooth, detrend with an EMA, take rising mean-crossings
 * through an adaptive hysteresis band as edges, emit the gap to the previous
 * edge. Streaming: one SPI4 sample at a time, only intervals stored. */
int hal_rfid_lf_acquire(uint8_t *buf, int max, uint32_t opts)
{
    if (s_mode != RFID_LF_READER || !buf || max <= 0) return RFID_ERR_UNSUPP;

    /* Capture a clean, gap-free block of raw envelope samples via DMA, then demod
     * it. (A static raw buffer keeps this off the caller's small heap/stack.) */
    static uint8_t raw[LF_SPAN];
    if (!ssc_dma_capture(raw, LF_SPAN)) return RFID_ERR_TIMEOUT;

    int32_t dc = -1, amp = 0;
    uint8_t win[LF_SMOOTH] = {0};
    int32_t wsum = 0;
    int count = 0, last = -1, prev = 0;
    bool armed = true;
    const int warmup = (opts & 1u) ? 24 : LF_WARMUP;

    for (int i = 0; i < LF_SPAN && count < max; i++) {
        uint8_t s = raw[i];

        wsum += (int32_t)s - win[i % LF_SMOOTH];
        win[i % LF_SMOOTH] = s;
        int32_t v = (wsum / LF_SMOOTH) << 8;

        if (dc < 0) dc = v;
        dc += (v - dc) >> 7;
        int ac = (int)((v - dc) >> 8);
        amp += ((((int32_t)(ac < 0 ? -ac : ac)) << 8) - amp) >> 6;

        int band = (int)(amp >> 8) / 2;
        if (band < 2) band = 2;

        if (i >= warmup) {
            if (armed && ac > band && prev <= band) {
                if (last >= 0) { int d = i - last; buf[count++] = (d > 0xFF) ? 0xFF : (uint8_t)d; }
                last = i;
                armed = false;
            } else if (!armed && ac < -band) {
                armed = true;
            }
        }
        prev = ac;
    }
    if ((amp >> 8) < 2) return 0;
    return count;
}

/* ---- T5577 downlink (LF write / raw transceive) ----
 * "Fixed bit length" timing, PM3 standard-antenna defaults (1 fc = 8 us @125 kHz):
 * a data bit = field ON for write_0/write_1 us, then a write_gap field-OFF; the
 * command is preceded by a start_gap. */
#define T55_START_GAP_US   248
#define T55_WRITE_GAP_US   160
#define T55_WRITE_0_US     144
#define T55_WRITE_1_US     400
#define T55_POWERUP_MS     8
#define T55_PROGRAM_MS     6
#define T55_READ_SETTLE_US 120

/* Fast LF field on/off for gap modulation - just the config word (divisor/drive
 * are already set by set_mode(LF_READER)). */
static void lf_tx_on(void)  { fpga_conf(FPGA_MAJOR_MODE_LF_READER | FPGA_LF_ADC_READER_FIELD); }
static void lf_tx_off(void) { fpga_conf(FPGA_MAJOR_MODE_OFF); }

/* Gap-modulate the T5577 command in `p` (one byte per bit, 0/1, MSB order set by
 * caller). Charges the tag, then IRQs-off sends start_gap + each bit's ON/gap. The
 * field is left ON at exit; the caller decides what happens next (a write holds it
 * for the EEPROM commit, a read holds it for the reply). */
static void lf_tx_cmd(const uint8_t *p, int nbits)
{
    lf_tx_on();
    spin_ms(T55_POWERUP_MS);                             /* charge the tag (IRQs on) */

    taskENTER_CRITICAL();                                /* gap timing must not be preempted */
    lf_tx_off(); spin_us(T55_START_GAP_US);
    for (int i = 0; i < nbits; i++) {
        lf_tx_on();  spin_us(p[i] ? T55_WRITE_1_US : T55_WRITE_0_US);
        lf_tx_off(); spin_us(T55_WRITE_GAP_US);
    }
    lf_tx_on();
    taskEXIT_CRITICAL();
}

/* T5577 block WRITE downlink (TX only): gap-modulate `p` then hold the field for
 * the EEPROM commit. */
int hal_rfid_lf_modulate(const uint8_t *p, int nbits, uint32_t opts)
{
    (void)opts;
    if (s_mode != RFID_LF_READER || !p || nbits <= 0) return RFID_ERR_UNSUPP;
    lf_tx_cmd(p, nbits);
    spin_ms(T55_PROGRAM_MS);
    lf_tx_off();
    return 0;
}

/* LF reader->tag round-trip: optionally gap-modulate a downlink, hold the field,
 * and stream-demodulate the reply into `buf` as inter-edge run lengths (sample
 * counts of each level run), up to `cap` runs. The runs alternate low, high, ...
 * from the first falling edge (the gap->reply transition); the caller frames the
 * block. Returns the run count, or <0. */
int hal_rfid_lf_transceive(const uint8_t *cmd, int nbits, uint8_t *buf, int cap)
{
    if (s_mode != RFID_LF_READER || !buf || cap <= 0) return RFID_ERR_UNSUPP;

    if (cmd && nbits > 0) lf_tx_cmd(cmd, nbits);         /* send downlink; field left ON */
    else                  lf_tx_on();
    fpga_ssc_setup(false);                               /* (re)arm the envelope sample stream */
    spin_us(T55_READ_SETTLE_US);
    if (SPI4->STS & SPI_STS_RDBF) (void)SPI4->DT;        /* flush stale */
    (void)SPI4->STS;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(300);

    int32_t dc = -1, amp = 0;
    uint8_t win[LF_SMOOTH] = {0};
    int32_t wsum = 0;
    int count = 0, last_edge = -1;
    int high = 1, started = 0;

    for (int i = 0; i < LF_SPAN && count < cap; i++) {
        uint32_t w = 0;
        while (!(SPI4->STS & SPI_STS_RDBF)) {
            if (xTaskGetTickCount() > deadline || ++w > 2000000u) { lf_tx_off(); return count; }
        }
        uint8_t s = (uint8_t)SPI4->DT;

        wsum += (int32_t)s - win[i % LF_SMOOTH];
        win[i % LF_SMOOTH] = s;
        int32_t v = (wsum / LF_SMOOTH) << 8;

        if (dc < 0) dc = v;
        dc += (v - dc) >> 7;
        int ac = (int)((v - dc) >> 8);
        amp += ((((int32_t)(ac < 0 ? -ac : ac)) << 8) - amp) >> 6;

        int band = (int)(amp >> 8) / 2;
        if (band < 2) band = 2;

        if (i >= 24) {
            if (high && ac < -band) {                    /* falling edge */
                if (started) { int d = i - last_edge; buf[count++] = d > 0xFF ? 0xFF : (uint8_t)d; }
                else started = 1;                        /* first falling edge = gap->reply */
                last_edge = i; high = 0;
            } else if (!high && ac > band) {             /* rising edge */
                if (started) { int d = i - last_edge; buf[count++] = d > 0xFF ? 0xFF : (uint8_t)d; }
                last_edge = i; high = 1;
            }
        }
    }
    lf_tx_off();
    return count;
}
