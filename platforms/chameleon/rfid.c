/* Fantasi / Chameleon Ultra - RFID HAL (HF reader via MFRC522).
 *
 * Implements the hal_rfid.h contract for the Chameleon's HF *read* path, which
 * is a discrete MFRC522 on SPI0 (the nRF's own NFCT is emulation-only and is a
 * later phase; the LF chain likewise). Ported from the stock ChameleonUltra
 * driver (application/src/rfid/reader/hf/rc522.c) but against bare nRF registers
 * - no Nordic SDK. SPI0 is driven directly (NRF_SPI0->TXD/RXD/EVENTS_READY) with
 * a software-toggled CS, exactly as the stock firmware does.
 *
 * Pin map: Ultra hw_ver 1 (matches platforms/chameleon/hal.c LED map):
 *   CS=P1.06 MISO=P0.11 MOSI=P1.07 SCK=P1.04  HF_ANT_SEL=P1.10  READER_POWER=P1.15
 */
#include "nrf.h"
#include "ble.h"
#include "../../hal/hal_rfid.h"
#include "../../core/vfs.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

/* ---- Pins (absolute nRF pin numbers: P1.x == 32 + x) ---- */
#define PIN_CS        (32 + 6)    /* P1.06 */
#define PIN_MISO      (0  + 11)   /* P0.11 */
#define PIN_MOSI      (32 + 7)    /* P1.07 */
#define PIN_SCK       (32 + 4)    /* P1.04 */
#define PIN_HF_ANTSEL (32 + 10)   /* P1.10 */
#define PIN_RDR_POWER (32 + 15)   /* P1.15 */
#define PIN_LF_DRV    (0  + 31)   /* P0.31  LF antenna PWM carrier out */
#define PIN_LF_OA     (0  + 29)   /* P0.29  LF op-amp comparator edges in */

static NRF_GPIO_Type *port_of(uint32_t pin) { return (pin < 32) ? NRF_P0 : NRF_P1; }
static uint32_t       bit_of(uint32_t pin)  { return pin & 31; }

static void pin_out(uint32_t pin, int level)
{
    NRF_GPIO_Type *p = port_of(pin);
    uint32_t b = bit_of(pin);
    if (level) p->OUTSET = (1UL << b); else p->OUTCLR = (1UL << b);
    p->PIN_CNF[b] = (GPIO_PIN_CNF_DIR_Output       << GPIO_PIN_CNF_DIR_Pos)
                  | (GPIO_PIN_CNF_INPUT_Disconnect  << GPIO_PIN_CNF_INPUT_Pos);
}
static void pin_in(uint32_t pin)
{
    NRF_GPIO_Type *p = port_of(pin);
    uint32_t b = bit_of(pin);
    p->PIN_CNF[b] = (GPIO_PIN_CNF_DIR_Input   << GPIO_PIN_CNF_DIR_Pos)
                  | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos);
}
static void pin_set(uint32_t pin, int level)
{
    NRF_GPIO_Type *p = port_of(pin);
    if (level) p->OUTSET = (1UL << bit_of(pin)); else p->OUTCLR = (1UL << bit_of(pin));
}

#define CS_LOW()   pin_set(PIN_CS, 0)
#define CS_HIGH()  pin_set(PIN_CS, 1)

/* ---- MFRC522 register + command set (subset; see rc522.h upstream) ---- */
#define CommandReg     0x01
#define ComIrqReg      0x04
#define ErrorReg       0x06
#define Status2Reg     0x08
#define FIFODataReg    0x09
#define FIFOLevelReg   0x0A
#define Control522Reg  0x0C
#define BitFramingReg  0x0D
#define ModeReg        0x11
#define TxControlReg   0x14
#define TxAutoReg      0x15
#define MfRxReg        0x1D
#define TModeReg       0x2A
#define VersionReg     0x37

/* Receiver registers used by the passive-RX card side of hal_rfid_hf_sniff. */
#define RxModeReg      0x13
#define RxSelReg       0x17
#define RxThresholdReg 0x18
#define DemodReg       0x19
#define RFCfgReg       0x26

#define PCD_IDLE       0x00
#define PCD_RECEIVE    0x08
#define PCD_TRANSCEIVE 0x0C
#define PCD_RESET      0x0F

static bool s_spi_up;
static rfid_mode_t s_mode = RFID_OFF;

/* ---- Raw SPI0 (legacy, non-DMA) - mode 0, 8 MHz, MSB first ---- */
static void spi_init(void)
{
    if (s_spi_up) return;

    pin_out(PIN_SCK, 0);      /* CPOL=0 idle low */
    pin_out(PIN_MOSI, 0);
    pin_in(PIN_MISO);
    pin_out(PIN_CS, 1);       /* CS idle high */

    NRF_SPI0->ENABLE   = 0;
    NRF_SPI0->PSEL.SCK  = PIN_SCK;
    NRF_SPI0->PSEL.MOSI = PIN_MOSI;
    NRF_SPI0->PSEL.MISO = PIN_MISO;
    NRF_SPI0->FREQUENCY = SPI_FREQUENCY_FREQUENCY_M8;
    NRF_SPI0->CONFIG    = (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos)
                        | (SPI_CONFIG_CPHA_Leading    << SPI_CONFIG_CPHA_Pos)
                        | (SPI_CONFIG_ORDER_MsbFirst  << SPI_CONFIG_ORDER_Pos);
    NRF_SPI0->EVENTS_READY = 0;
    NRF_SPI0->ENABLE   = SPI_ENABLE_ENABLE_Enabled;
    s_spi_up = true;
}

static void spi_deinit(void)
{
    if (!s_spi_up) return;
    NRF_SPI0->ENABLE = 0;
    s_spi_up = false;
}

static uint8_t spi_xfer(uint8_t v)
{
    NRF_SPI0->EVENTS_READY = 0;
    NRF_SPI0->TXD = v;
    /* Bounded wait: an 8 MHz byte completes in ~1 us, so this guard is orders of magnitude of slack.
     * Without it a glitched/stuck SPI transaction busy-waits forever and freezes the whole task.
     * Never hang; return whatever RXD holds. */
    uint32_t guard = 100000;
    while (NRF_SPI0->EVENTS_READY == 0 && --guard) { }
    NRF_SPI0->EVENTS_READY = 0;
    return (uint8_t)NRF_SPI0->RXD;
}

/* MFRC522 SPI address byte: bit7 = read, bits[6:1] = addr, bit0 = 0. */
static uint8_t reg_read(uint8_t addr)
{
    uint8_t a = (uint8_t)(((addr << 1) & 0x7E) | 0x80);
    CS_LOW();
    spi_xfer(a);            /* address phase */
    uint8_t v = spi_xfer(a);/* clock out value (repeat addr per datasheet) */
    CS_HIGH();
    return v;
}
static void reg_write(uint8_t addr, uint8_t val)
{
    uint8_t a = (uint8_t)((addr << 1) & 0x7E);
    CS_LOW();
    spi_xfer(a);
    spi_xfer(val);
    CS_HIGH();
}
static void reg_set(uint8_t addr, uint8_t mask)   { reg_write(addr, reg_read(addr) | mask); }
static void reg_clear(uint8_t addr, uint8_t mask) { reg_write(addr, reg_read(addr) & (uint8_t)~mask); }

static void reg_write_fifo(const uint8_t *buf, int n)
{
    uint8_t a = (uint8_t)((FIFODataReg << 1) & 0x7E);
    CS_LOW();
    spi_xfer(a);
    for (int i = 0; i < n; i++) spi_xfer(buf[i]);
    CS_HIGH();
}
static void reg_read_fifo(uint8_t *buf, int n)
{
    uint8_t a = (uint8_t)(((FIFODataReg << 1) & 0x7E) | 0x80);
    CS_LOW();
    spi_xfer(a);
    for (int i = 0; i < n; i++) buf[i] = spi_xfer(a);
    CS_HIGH();
}

static void rc522_reset(void)
{
    reg_write(CommandReg, PCD_IDLE);
    reg_write(CommandReg, PCD_RESET);
    vTaskDelay(pdMS_TO_TICKS(10));
    reg_clear(TxControlReg, 0x03);   /* antenna off */
    reg_write(TModeReg, 0x00);       /* use MCU timeout, not the 522 timer */
    reg_write(TxAutoReg, 0x40);      /* 100% ASK */
    reg_write(ModeReg, 0x3D);        /* CRC preset 0x6363, common tx/rx */
    vTaskDelay(pdMS_TO_TICKS(10));
}

/* ---- LF 125 kHz reader (ASK read via SAADC envelope + software demod) ----
 * A 125 kHz carrier on LF_ANT_DRIVER (PWM0, 4 MHz base / COUNTERTOP) energises
 * the tag; its load modulation comes back demodulated on the op-amp output
 * LF_OA_OUT (= AIN5). The SAADC samples that envelope once per carrier cycle
 * (PWMPERIODEND -> SAMPLE via PPI) into RAM, and hal_rfid_lf_acquire recovers the
 * inter-edge intervals in software (see there for why this beats a GPIOTE digital
 * edge detector). core/rfid/rfid_em4100.c turns the intervals into a UID.
 * Frontend + timing reference: ChameleonUltra lf_125khz_radio.c (no nrfx). */

#define LF_PPI_SAADC   1   /* PWMPERIODEND -> SAADC SAMPLE */
#define LF_PPI_PHASE   2   /* PWMPERIODEND -> TIMER3 COUNT (32-bit response phase) */
#define LF_PPI_ARM     4   /* TIMER3 count 2047 -> enable SAADC group */
#define LF_PPI_REPHASE 5   /* completed downlink -> reset response phase */
#define LF_PPI_GRP_ADC 0
#define LF_PPI_GRP_ARM 1
#define LF_AIN         SAADC_CH_PSELP_PSELP_AnalogInput5   /* LF_OA_OUT = P0.29 = AIN5 */

/* Carrier via a 4 MHz PWM clock: f = 4 MHz / COUNTERTOP, so top 32 => 125 kHz.
 * s_pwm_val is the compare (duty) value; a low duty (~1/8) keeps the op-amp gain
 * stages linear - see the acquire comment for why that matters. */
static uint16_t  s_lf_top = 32;                /* 4 MHz / 32 = 125 kHz (bit rate lands in the op-amp band) */
#define LF_DUTY_RX  4                          /* ~1/8 duty: clean OOK gaps + linear receive amplifier */
#define LF_DUTY_WRITE 16                       /* 1/2 duty: stock CU field strength for EEPROM programming */
#define LF_PWM_INVERTED 0x8000u                /* sequence polarity bit: stock CU parks write gaps high */
/* EasyDMA re-reads this every PWM period (SEQ REFRESH=0 + loop shorts), so a plain store changes the drive
 * within one carrier cycle - volatile because the peripheral, not this code, is the other reader. */
static volatile uint16_t s_pwm_val = LF_DUTY_RX;
static bool      s_lf_inited;
static bool      s_pwm_running;

/* Envelope sample buffer, sized for ~2 EM4100 frames (64 bits x ~64 carrier
 * cycles/bit x 2) so an aligned 64-bit frame is always contained. */
#define LF_ENV_SAMPLES 9216
static int16_t s_lf_env[LF_ENV_SAMPLES];

static void lf_saadc_disconnect(void)
{
    for (int ch = 0; ch < 8; ch++) {
        NRF_SAADC->CH[ch].PSELP = 0;
        NRF_SAADC->CH[ch].PSELN = 0;
    }
}

static void lf_hw_init(void)
{
    if (s_lf_inited) return;

    /* A warm handoff can leave peripheral state behind even though C state was reset. Quiesce every LF
     * endpoint before installing the fixed PPI topology so no stale channel can prefill a later DMA buffer. */
    NRF_PPI->TASKS_CHG[LF_PPI_GRP_ADC].DIS = 1;
    NRF_PPI->TASKS_CHG[LF_PPI_GRP_ARM].DIS = 1;
    NRF_PPI->CHENCLR = (1u << LF_PPI_SAADC) | (1u << LF_PPI_PHASE) |
                       (1u << LF_PPI_ARM) | (1u << LF_PPI_REPHASE);
    NRF_PPI->FORK[LF_PPI_SAADC].TEP = 0;
    NRF_PPI->FORK[LF_PPI_PHASE].TEP = 0;
    NRF_PPI->FORK[LF_PPI_ARM].TEP = 0;
    NRF_PPI->FORK[LF_PPI_REPHASE].TEP = 0;
    if (NRF_PWM0->ENABLE == PWM_ENABLE_ENABLE_Enabled) {
        NRF_PWM0->SHORTS = 0;
        NRF_PWM0->EVENTS_STOPPED = 0;
        NRF_PWM0->TASKS_STOP = 1;
        for (uint32_t g = 0; !NRF_PWM0->EVENTS_STOPPED && g < 1000000; g++) { }
    }
    if (NRF_SAADC->ENABLE == SAADC_ENABLE_ENABLE_Enabled) {
        NRF_SAADC->EVENTS_STOPPED = 0;
        NRF_SAADC->TASKS_STOP = 1;
        for (uint32_t g = 0; !NRF_SAADC->EVENTS_STOPPED && g < 1000000; g++) { }
        NRF_SAADC->ENABLE = 0;
    }
    lf_saadc_disconnect();

    pin_out(PIN_LF_DRV, 0);
    /* LF_OA_OUT read as analog (AIN5) by the SAADC; leave the pin's digital
     * input buffer disconnected so it doesn't load the op-amp output. */
    NRF_P0->PIN_CNF[29] = (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos)
                        | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

    /* PWM0: gapless 125 kHz carrier on LF_ANT_DRIVER. */
    NRF_PWM0->PSEL.OUT[0] = PIN_LF_DRV;
    NRF_PWM0->PSEL.OUT[1] = 0xFFFFFFFF;
    NRF_PWM0->PSEL.OUT[2] = 0xFFFFFFFF;
    NRF_PWM0->PSEL.OUT[3] = 0xFFFFFFFF;
    NRF_PWM0->ENABLE     = PWM_ENABLE_ENABLE_Enabled;
    NRF_PWM0->MODE       = PWM_MODE_UPDOWN_Up;
    NRF_PWM0->PRESCALER  = PWM_PRESCALER_PRESCALER_DIV_4;    /* 16 MHz/4 = 4 MHz */
    NRF_PWM0->COUNTERTOP = s_lf_top;                         /* 4 MHz/32 = 125 kHz */
    NRF_PWM0->LOOP       = 1;
    NRF_PWM0->DECODER    = (PWM_DECODER_LOAD_Common << 0) |
                           (PWM_DECODER_MODE_RefreshCount << 8);
    NRF_PWM0->SEQ[0].PTR     = (uint32_t)(uintptr_t)&s_pwm_val;
    NRF_PWM0->SEQ[0].CNT     = 1;
    /* REFRESH=0 + LOOPSDONE->SEQSTART0: EasyDMA re-fetches the duty every period, so the drive level can be
     * changed by storing to s_pwm_val with no STOP/SEQSTART. */
    NRF_PWM0->SEQ[0].REFRESH = 0;
    NRF_PWM0->SEQ[0].ENDDELAY = 0;
    /* SEQ[1] must mirror SEQ[0]: the loop mechanism plays SEQ0 then SEQ1 before LOOPSDONE, so with SEQ1
     * unconfigured the playback never loops and EasyDMA never re-fetches - the duty then appears frozen
     * (carrier fine, but a gap silently does nothing). nrfx_pwm_simple_playback sets both for this reason. */
    NRF_PWM0->SEQ[1].PTR      = (uint32_t)(uintptr_t)&s_pwm_val;
    NRF_PWM0->SEQ[1].CNT      = 1;
    NRF_PWM0->SEQ[1].REFRESH  = 0;
    NRF_PWM0->SEQ[1].ENDDELAY = 0;
    NRF_PWM0->SHORTS = PWM_SHORTS_LOOPSDONE_SEQSTART0_Msk;

    /* PPI: the PWM period end drives each SAADC sample, so we sample exactly once
     * per carrier cycle (a sample index is therefore a carrier-cycle count). */
    NRF_PPI->CH[LF_PPI_SAADC].EEP = (uint32_t)(uintptr_t)&NRF_PWM0->EVENTS_PWMPERIODEND;
    NRF_PPI->CH[LF_PPI_SAADC].TEP = (uint32_t)(uintptr_t)&NRF_SAADC->TASKS_SAMPLE;

    /* Carrier-cycle phase over one complete 32-bit RF/64 response (2048 carrier periods). A bit-only modulo
     * would preserve the Manchester cell grid but still allow a calibration and target to be captured at
     * different whole-bit rotations. The final field restore of every hardware downlink clears this counter. */
    NRF_TIMER3->TASKS_STOP  = 1;
    NRF_TIMER3->MODE        = TIMER_MODE_MODE_Counter;
    NRF_TIMER3->BITMODE     = TIMER_BITMODE_BITMODE_16Bit;
    NRF_TIMER3->TASKS_CLEAR = 1;
    NRF_TIMER3->CC[0]       = 2048;
    NRF_TIMER3->CC[2]       = 2047;
    NRF_TIMER3->SHORTS      = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    NRF_TIMER3->TASKS_START = 1;
    NRF_PPI->CH[LF_PPI_PHASE].EEP = (uint32_t)(uintptr_t)&NRF_PWM0->EVENTS_PWMPERIODEND;
    NRF_PPI->CH[LF_PPI_PHASE].TEP = (uint32_t)(uintptr_t)&NRF_TIMER3->TASKS_COUNT;

    /* Hardware capture arm. At carrier count 2047, enable the PWMEND->SAADC channel. The following PWM period
     * end is count 0, so sample zero has exact 32-bit response phase if a priority-0 SoftDevice interrupt
     * preempts the CPU while the capture is being arranged. The arm stays enabled during DMA (re-enabling an
     * already-enabled channel is harmless) and teardown disables both groups before stopping the SAADC. This
     * avoids a simultaneous group-EN/self-DIS corner at the compare event. */
    NRF_PPI->TASKS_CHG[LF_PPI_GRP_ADC].DIS = 1;
    NRF_PPI->TASKS_CHG[LF_PPI_GRP_ARM].DIS = 1;
    NRF_PPI->CHG[LF_PPI_GRP_ADC] = (1u << LF_PPI_SAADC);
    NRF_PPI->CHG[LF_PPI_GRP_ARM] = (1u << LF_PPI_ARM);
    NRF_PPI->CH[LF_PPI_ARM].EEP = (uint32_t)(uintptr_t)&NRF_TIMER3->EVENTS_COMPARE[2];
    NRF_PPI->CH[LF_PPI_ARM].TEP = (uint32_t)(uintptr_t)&NRF_PPI->TASKS_CHG[LF_PPI_GRP_ADC].EN;
    NRF_PPI->FORK[LF_PPI_ARM].TEP = 0;
    NRF_PPI->CH[LF_PPI_REPHASE].EEP = (uint32_t)(uintptr_t)&NRF_PWM0->EVENTS_SEQEND[1];
    NRF_PPI->CH[LF_PPI_REPHASE].TEP = (uint32_t)(uintptr_t)&NRF_TIMER3->TASKS_CLEAR;

    s_lf_inited = true;
}

static void lf_start(void)
{
    if (!s_lf_inited) return;
    s_pwm_val = LF_DUTY_RX;
    NRF_PPI->CHENSET = (1u << LF_PPI_PHASE);
    if (!s_pwm_running) {
        NRF_PWM0->LOOP = 1;
        NRF_PWM0->SEQ[0].PTR = (uint32_t)(uintptr_t)&s_pwm_val;
        NRF_PWM0->SEQ[0].CNT = 1;
        NRF_PWM0->SEQ[0].REFRESH = 0;
        NRF_PWM0->SEQ[0].ENDDELAY = 0;
        NRF_PWM0->SEQ[1].PTR = (uint32_t)(uintptr_t)&s_pwm_val;
        NRF_PWM0->SEQ[1].CNT = 1;
        NRF_PWM0->SEQ[1].REFRESH = 0;
        NRF_PWM0->SEQ[1].ENDDELAY = 0;
        NRF_PWM0->SHORTS = PWM_SHORTS_LOOPSDONE_SEQSTART0_Msk;
        NRF_PWM0->TASKS_SEQSTART[0] = 1;
        s_pwm_running = true;
    }
}

static void lf_stop(void)
{
    if (!s_lf_inited) return;
    NRF_PPI->TASKS_CHG[LF_PPI_GRP_ADC].DIS = 1;
    NRF_PPI->TASKS_CHG[LF_PPI_GRP_ARM].DIS = 1;
    NRF_PPI->CHENCLR = (1u << LF_PPI_SAADC) | (1u << LF_PPI_PHASE) |
                       (1u << LF_PPI_ARM) | (1u << LF_PPI_REPHASE);
    if (NRF_SAADC->ENABLE == SAADC_ENABLE_ENABLE_Enabled) {
        NRF_SAADC->EVENTS_STOPPED = 0;
        NRF_SAADC->TASKS_STOP = 1;
        for (uint32_t g = 0; !NRF_SAADC->EVENTS_STOPPED && g < 1000000; g++) { }
        NRF_SAADC->ENABLE = 0;
    }
    lf_saadc_disconnect();
    NRF_PWM0->SHORTS = 0;                         /* a LOOPSDONE shortcut can otherwise restart after STOP */
    NRF_PWM0->EVENTS_STOPPED = 0;
    NRF_PWM0->TASKS_STOP = 1;
    for (uint32_t g = 0; !NRF_PWM0->EVENTS_STOPPED && g < 1000000; g++) { }
    s_pwm_running = false;
    pin_set(PIN_LF_DRV, 0);
}

/* DMA-sample AIN5 for `nsamp` samples at the carrier rate; returns 0 on success. The buffer is int16_t
 * (SAADC result width) regardless of resolution. PWMPERIODEND drives SAMPLE over PPI, so a sample index is a
 * carrier-cycle count - what the EM4100 decoder's interval arithmetic depends on. */

static int lf_sample_prepare(int16_t *raw, int nsamp, uint32_t resolution)
{
    /* Disable both the direct sample path and its hardware arm before START. If a prior task was cancelled
     * mid-capture, this prevents carrier edges from filling arbitrary-phase samples into the new buffer. */
    NRF_PPI->TASKS_CHG[LF_PPI_GRP_ARM].DIS = 1;
    NRF_PPI->TASKS_CHG[LF_PPI_GRP_ADC].DIS = 1;
    NRF_PPI->CHENCLR = (1u << LF_PPI_ARM) | (1u << LF_PPI_SAADC);
    if (NRF_SAADC->ENABLE == SAADC_ENABLE_ENABLE_Enabled) {
        NRF_SAADC->EVENTS_STOPPED = 0;
        NRF_SAADC->TASKS_STOP = 1;
        for (uint32_t g = 0; !NRF_SAADC->EVENTS_STOPPED && g < 1000000; g++) { }
        NRF_SAADC->ENABLE = 0;
    }

    /* Normalize inherited scan/oversample state before installing the one-channel, task-triggered capture. */
    lf_saadc_disconnect();
    NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Bypass;
    NRF_SAADC->SAMPLERATE = (SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos);
    NRF_SAADC->RESOLUTION = resolution;
    NRF_SAADC->CH[0].PSELP  = LF_AIN;
    NRF_SAADC->CH[0].PSELN  = 0;
    NRF_SAADC->CH[0].CONFIG =
        (SAADC_CH_CONFIG_GAIN_Gain1_6   << SAADC_CH_CONFIG_GAIN_Pos)   |
        (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
        (SAADC_CH_CONFIG_TACQ_3us       << SAADC_CH_CONFIG_TACQ_Pos)   |
        (SAADC_CH_CONFIG_MODE_SE         << SAADC_CH_CONFIG_MODE_Pos)   |
        (SAADC_CH_CONFIG_RESP_Bypass     << SAADC_CH_CONFIG_RESP_Pos);
    NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled;

    NRF_SAADC->RESULT.PTR    = (uint32_t)(uintptr_t)raw;
    NRF_SAADC->RESULT.MAXCNT = (uint32_t)nsamp;
    NRF_SAADC->EVENTS_STARTED = 0;
    NRF_SAADC->EVENTS_END     = 0;
    NRF_SAADC->TASKS_START = 1;
    for (uint32_t g = 0; !NRF_SAADC->EVENTS_STARTED && g < 1000000; g++) { }   /* never hang on a stuck SAADC */

    if (!NRF_SAADC->EVENTS_STARTED) {
        NRF_SAADC->ENABLE = 0;
        lf_saadc_disconnect();
        return -1;
    }
    return 0;
}

static int lf_sample_finish(int nsamp)
{
    /* nsamp/125kHz ~ 72 ms; poll for completion with margin. */
    TickType_t started = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(nsamp / 100 + 50);
    while (!NRF_SAADC->EVENTS_END && (TickType_t)(xTaskGetTickCount() - started) < timeout)
        vTaskDelay(pdMS_TO_TICKS(5));

    int complete = NRF_SAADC->EVENTS_END != 0;
    NRF_PPI->TASKS_CHG[LF_PPI_GRP_ARM].DIS = 1;
    NRF_PPI->TASKS_CHG[LF_PPI_GRP_ADC].DIS = 1;
    NRF_PPI->CHENCLR = (1u << LF_PPI_ARM) | (1u << LF_PPI_SAADC);
    NRF_SAADC->EVENTS_STOPPED = 0;
    NRF_SAADC->TASKS_STOP = 1;
    for (uint32_t g = 0; !NRF_SAADC->EVENTS_STOPPED && g < 1000000; g++) { }
    NRF_SAADC->ENABLE = 0;
    lf_saadc_disconnect();
    return complete ? 0 : -1;
}

static int lf_sample(int16_t *raw, int nsamp)
{
    if (lf_sample_prepare(raw, nsamp, SAADC_RESOLUTION_VAL_8bit) != 0) return -1;

    NRF_PPI->CHENSET = (1u << LF_PPI_SAADC);   /* let PWMPERIODEND drive SAMPLE */
    return lf_sample_finish(nsamp);
}

/* HF tag-emulation state (defined with the NFCT emulation block below; declared here for set_mode). */
static int s_emu_armed;
static int s_emu_active;
static bool s_emu_hfclk;
static TaskHandle_t s_emu_hot_task;
static UBaseType_t s_emu_hot_prio;

static void emu_hot_enter(void)
{
    if (s_emu_hot_task) return;
    s_emu_hot_task = xTaskGetCurrentTaskHandle();
    s_emu_hot_prio = uxTaskPriorityGet(NULL);
    if (s_emu_hot_prio < configMAX_PRIORITIES - 1)
        vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
}

static void emu_hot_leave(void)
{
    if (!s_emu_hot_task) return;
    if (xTaskGetCurrentTaskHandle() == s_emu_hot_task &&
        uxTaskPriorityGet(NULL) != s_emu_hot_prio)
        vTaskPrioritySet(NULL, s_emu_hot_prio);
    s_emu_hot_task = NULL;
}

static void emu_stop(void)
{
    emu_hot_leave();
    NRF_NFCT->SHORTS &= ~NFCT_SHORTS_TXFRAMEEND_ENABLERXDATA_Msk;
    NRF_NFCT->TASKS_DISABLE = 1;
    s_emu_armed = 0;
    s_emu_active = 0;
    if (s_emu_hfclk) {
        cu_hfclk_release();
        s_emu_hfclk = false;
    }
}

/* ---- HAL contract ---- */

uint32_t hal_rfid_caps(void)
{
    /* HF read (Ci522) + LF read (EM4100/T5577) + HF ISO14443-A tag emulation on the nRF's own NFCT.
     * LF emulation is still a later phase (see docs/rfid.md). */
    return RFID_CAP_HF_READ | RFID_CAP_LF_READ | RFID_CAP_HF_EMU;
}

int hal_rfid_set_mode(rfid_mode_t mode)
{
    if (mode == s_mode) return 0;
    if (s_mode == RFID_HF_EMU) emu_stop();

    switch (mode) {
    case RFID_HF_READER:
        pin_out(PIN_RDR_POWER, 1);   /* reader analog power on */
        pin_out(PIN_HF_ANTSEL, 0);   /* HF antenna -> reader (RC522) */
        spi_init();
        rc522_reset();
        reg_set(TxControlReg, 0x03); /* energise the HF field: mfc modules transceive right after set_mode
                                      * and never call field(1), so bring it up here (CU antenna_on). */
        vTaskDelay(pdMS_TO_TICKS(5));/* field settle + card power-up before the first WUPA */
        s_mode = RFID_HF_READER;
        return 0;

    case RFID_LF_READER:
        pin_out(PIN_RDR_POWER, 1);   /* reader analog power on (LF op-amp) */
        pin_out(PIN_HF_ANTSEL, 0);
        /* The stock reader_mode_enter also brings up the RC522 here; it shares
         * READER_POWER and the reader analog frontend, so mirror that. */
        spi_init();
        rc522_reset();
        lf_hw_init();
        s_mode = RFID_LF_READER;
        return 0;

    case RFID_HF_EMU:
        /* Tag emulation runs on the nRF's own NFCT, not the Ci522: route the shared coil to the NFCT tap and
         * put the reader frontend away so it cannot drive the antenna. Auto-collision-resolution is disabled
         * so every frame reaches software - the module owns anticollision (it answers ATQA/UID/SAK itself)
         * and, more importantly, MIFARE auth frames must not be answered by hardware. */
        if (!cu_hfclk_request()) return -1;
        s_emu_hfclk = true;
        reg_clear(TxControlReg, 0x03);            /* make sure the reader field is off before switching */
        spi_deinit();
        pin_out(PIN_RDR_POWER, 0);
        pin_out(PIN_HF_ANTSEL, 1);                /* shared antenna -> NFCT */
        NRF_NFCT->FRAMEDELAYMODE = 3;             /* WindowGrid: HW places the reply on the 14443-A timing grid */
        NRF_NFCT->FRAMEDELAYMAX  = 0xFFFF;
        NRF_NFCT->SENSRES = (NRF_NFCT->SENSRES & ~NFCT_SENSRES_BITFRAMESDD_Msk)
                          | (NFCT_SENSRES_BITFRAMESDD_SDD00100 << NFCT_SENSRES_BITFRAMESDD_Pos);
        *(volatile uint32_t *)0x4000559C |= 0x1UL;   /* AUTOCOLRESCONFIG.MODE = Disabled (undocumented reg) */
        NRF_NFCT->EVENTS_FIELDDETECTED = 0;
        NRF_NFCT->EVENTS_FIELDLOST     = 0;
        NRF_NFCT->EVENTS_RXFRAMESTART  = 0;
        NRF_NFCT->EVENTS_RXFRAMEEND    = 0;
        NRF_NFCT->EVENTS_RXERROR       = 0;
        NRF_NFCT->EVENTS_STARTED       = 0;
        NRF_NFCT->EVENTS_TXFRAMEEND    = 0;
        NRF_NFCT->SHORTS &= ~NFCT_SHORTS_TXFRAMEEND_ENABLERXDATA_Msk;
        NRF_NFCT->ERRORSTATUS = 0xFFFFFFFFUL;
        NRF_NFCT->TASKS_SENSE = 1;
        s_emu_armed = 0;
        s_emu_active = 0;
        s_mode = RFID_HF_EMU;
        return 0;

    case RFID_OFF:
        if (s_spi_up) reg_clear(TxControlReg, 0x03);  /* HF antenna off */
        spi_deinit();
        lf_stop();
        pin_out(PIN_RDR_POWER, 0);
        s_mode = RFID_OFF;
        return 0;

    default:
        return -1;   /* emulation not yet implemented on CU */
    }
}

void hal_rfid_field(bool on)
{
    if (!s_spi_up) return;
    if (on) reg_set(TxControlReg, 0x03);
    else    reg_clear(TxControlReg, 0x03);
}

int hal_rfid_hf_probe(void)
{
    if (!s_spi_up) return -1;
    return reg_read(VersionReg);
}

/* Unwrap an NFCT NoParity (RXD.FRAMECONFIG bit0=0) RX frame: the HW delivers the raw 9-bits-per-byte stream
 * (8 data LSB-first + 1 parity bit), so pull out the data bytes and record which parity bits violate
 * ISO14443-A odd parity (bit i of *par set = byte i's parity was wrong). Short frames (<9 bits: 7-bit
 * WUPA/REQA, 4-bit ACK/NAK) carry no parity - pass the partial byte through. Returns the byte count.
 * total_bits = (RXD.AMOUNT & 0xFFF): the register packs RXDATABYTES<<3 | RXDATABITS = total bits directly. */
static int nfct_unwrap(const uint8_t *raw, int total_bits, uint8_t *out, uint32_t *par, int cap)
{
    *par = 0;
    if (total_bits <= 0) return 0;
    if (total_bits < 9) {                        /* short frame: one (partial) byte, no parity */
        if (cap > 0) out[0] = (uint8_t)(raw[0] & ((1u << total_bits) - 1));
        return cap > 0 ? 1 : 0;
    }
    int n = 0;
    for (int bp = 0; bp + 8 <= total_bits && n < cap; bp += 9) {
        uint8_t b = 0;
        for (int i = 0; i < 8; i++)
            if (raw[(bp + i) >> 3] & (1u << ((bp + i) & 7))) b |= (uint8_t)(1u << i);
        out[n] = b;
        int pp = bp + 8;
        if (pp < total_bits) {                    /* odd parity: correct bit = !parity(byte) */
            int pbit = (raw[pp >> 3] >> (pp & 7)) & 1;
            if (pbit != (!__builtin_parity(b))) *par |= (1u << n);
        }
        n++;
    }
    return n;
}

/* EasyDMA RX buffer for the NFCT reader-direction receive (word-aligned RAM). */
static uint8_t s_nfc_rx[261] __attribute__((aligned(4)));

/* ---- HF ISO14443-A tag emulation (nRF NFCT) ----
 * The module speaks the PM3's symbol contract: one byte per subcarrier symbol, TAG_SEC_D/E for a logic 1/0
 * and TAG_SEC_F for the stop bit, with the start bit first and ISO14443-A parity already interleaved after
 * every byte. NFCT is frame-oriented instead, but its BIT mode maps onto that one-for-one: each symbol is a
 * bit, so we drop the leading start symbol (NFCT's SOF supplies it), stop at TAG_SEC_F, and hand the packed
 * bits to TXD with FRAMECONFIG = SOF only.
 *
 * Leaving NFCT's automatic PARITY off is what makes MIFARE work at all: Crypto1 ENCRYPTS the parity bits, so
 * they do not follow the odd-parity rule and hardware-generated parity would be wrong on every encrypted
 * response. The module already produced the right (encrypted) parity symbols; we just pass them through. */
#define EMU_SEC_D 0xF0     /* logic 1 - subcarrier in the first half-bit  */
#define EMU_SEC_F 0x00     /* stop bit / no modulation                    */

static uint8_t s_nfc_tx[64] __attribute__((aligned(4)));
/* s_emu_armed: EasyDMA RX is armed for the next reader frame.
 * s_emu_active: NFCT has been ACTIVATEd in the current field. (Both declared above set_mode.) */

/* Arm EasyDMA for one reader frame. NoParity RX (FRAMECONFIG bit0 = 0) hands us the raw 9-bits-per-byte
 * stream so nfct_unwrap can recover the reader's parity bits too. */
static void emu_prepare_rx(void)
{
    /* SOF expected (bit 2) so the HW consumes the start-of-frame symbol; PARITY off (bit 0) so the reader's
     * parity bits stay in the stream for nfct_unwrap. Clearing SOF instead makes NFCT hand the start bit over
     * as data, which shifts every byte left by one. */
    NRF_NFCT->RXD.FRAMECONFIG = NFCT_RXD_FRAMECONFIG_SOF_Msk;
    NRF_NFCT->PACKETPTR = (uint32_t)(uintptr_t)s_nfc_rx;
    NRF_NFCT->MAXLEN = sizeof s_nfc_rx;
    NRF_NFCT->EVENTS_RXFRAMESTART = 0;
    NRF_NFCT->EVENTS_RXFRAMEEND = 0;
    NRF_NFCT->EVENTS_RXERROR    = 0;
    s_emu_armed = 1;
}

static void emu_arm_rx(void)
{
    emu_prepare_rx();
    NRF_NFCT->TASKS_ENABLERXDATA = 1;
}

/* Once STARTED captures the TX descriptor, PACKETPTR can describe the next RX.
 * Let hardware re-arm at the final transmitted symbol so SoftDevice activity
 * cannot make us miss a reader frame at the minimum turnaround. */
static bool emu_rearm_at_tx_end(void)
{
    for (uint32_t g = 0; !NRF_NFCT->EVENTS_STARTED && g < 200000; g++) { }
    if (!NRF_NFCT->EVENTS_STARTED) return false;
    NRF_NFCT->EVENTS_STARTED = 0;
    emu_prepare_rx();
    NRF_NFCT->SHORTS |= NFCT_SHORTS_TXFRAMEEND_ENABLERXDATA_Msk;
    return true;
}

static void emu_finish_tx(bool hw_rearm)
{
    for (uint32_t g = 0; !NRF_NFCT->EVENTS_TXFRAMEEND && g < 2000000; g++) { }
    bool tx_done = NRF_NFCT->EVENTS_TXFRAMEEND != 0;
    NRF_NFCT->EVENTS_TXFRAMEEND = 0;
    NRF_NFCT->SHORTS &= ~NFCT_SHORTS_TXFRAMEEND_ENABLERXDATA_Msk;
    if (!hw_rearm || !tx_done) {
        s_emu_armed = 0;
        emu_arm_rx();
    }
}

/* Pack an iso14a_tag_encode symbol run into s_nfc_tx and fire it. The first two
 * symbols are the PM3 timing template and start bit; NFCT supplies its own SOF.
 * Returns the number of bits sent, or <0. */
static int emu_tx_bits(const uint8_t *sym, int n)
{
    int nbits = 0;
    for (int i = 2; i < n && nbits < (int)sizeof s_nfc_tx * 8; i++) {
        if (sym[i] == EMU_SEC_F) break;            /* stop bit ends the frame */
        if (sym[i] == EMU_SEC_D) s_nfc_tx[nbits >> 3] |=  (uint8_t)(1u << (nbits & 7));
        else                     s_nfc_tx[nbits >> 3] &= (uint8_t)~(1u << (nbits & 7));
        nbits++;
    }
    if (nbits <= 0) return RFID_ERR_UNSUPP;

    NRF_NFCT->PACKETPTR = (uint32_t)(uintptr_t)s_nfc_tx;
    /* AMOUNT packs TXDATABYTES<<3 | TXDATABITS, so writing the raw bit count sets both fields correctly. */
    NRF_NFCT->TXD.AMOUNT = (uint32_t)nbits;
    NRF_NFCT->TXD.FRAMECONFIG = NFCT_TXD_FRAMECONFIG_SOF_Msk;   /* SOF only: parity is already in the stream */
    NRF_NFCT->FRAMEDELAYMODE = 3;                  /* WindowGrid - HW lands the reply on the 14443-A grid */
    NRF_NFCT->EVENTS_STARTED = 0;
    NRF_NFCT->EVENTS_TXFRAMEEND = 0;
    NRF_NFCT->TASKS_STARTTX = 1;

    emu_finish_tx(emu_rearm_at_tx_end());
    return nbits;
}

int hal_rfid_hf_emu_recv(uint8_t *rx, uint8_t *rx_par, int cap, uint32_t timeout_ms)
{
    emu_hot_leave();
    if (s_mode != RFID_HF_EMU || !rx || cap <= 0) return RFID_ERR_UNSUPP;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms ? timeout_ms : 100);

    for (;;) {
        if (NRF_NFCT->EVENTS_FIELDDETECTED) {      /* reader arrived: leave SENSE for the active state */
            NRF_NFCT->EVENTS_FIELDDETECTED = 0;
            *(volatile uint32_t *)0x4000559C |= 0x1UL;
            NRF_NFCT->TASKS_ACTIVATE = 1;
            s_emu_active = 1;
            emu_arm_rx();
        }
        if (NRF_NFCT->EVENTS_FIELDLOST) {          /* reader left: back to sensing, drop any half-frame */
            NRF_NFCT->EVENTS_FIELDLOST = 0;
            NRF_NFCT->TASKS_SENSE = 1;
            s_emu_active = 0; s_emu_armed = 0;
            emu_hot_leave();
        }
        if (s_emu_active && !s_emu_armed) emu_arm_rx();

        if (NRF_NFCT->EVENTS_RXFRAMESTART) {
            NRF_NFCT->EVENTS_RXFRAMESTART = 0;
            NRF_NFCT->EVENTS_STARTED = 0;          /* next STARTED belongs to the reply's TX descriptor */
            NRF_NFCT->ERRORSTATUS = 0xFFFFFFFFUL;
            emu_hot_enter();
        }
        if (NRF_NFCT->EVENTS_RXERROR) {            /* framing/parity glitch: requeue, the reader will retry */
            NRF_NFCT->EVENTS_RXERROR = 0;
            NRF_NFCT->ERRORSTATUS = 0xFFFFFFFFUL;
            s_emu_armed = 0;
            emu_hot_leave();
            continue;
        }
        if (NRF_NFCT->EVENTS_RXFRAMEEND) {
            NRF_NFCT->EVENTS_RXFRAMEEND = 0;
            if (NRF_NFCT->EVENTS_STARTED) {        /* short-frame fallback when polling missed RXFRAMESTART */
                NRF_NFCT->EVENTS_STARTED = 0;
                NRF_NFCT->ERRORSTATUS = 0xFFFFFFFFUL;
            }
            s_emu_armed = 0;
            int bits = (int)(NRF_NFCT->RXD.AMOUNT & 0xFFF);
            if (bits >= 0xFFF) {                   /* saturated count = not a real reader frame */
                emu_hot_leave();
                continue;
            }
            uint32_t par = 0;
            int n = nfct_unwrap(s_nfc_rx, bits, rx, &par, cap);
            /* rx_par[i] = the parity bit RECEIVED for byte i (nfct_unwrap flags violations of odd parity;
             * an encrypted reader frame legitimately violates it, which is why the raw bit is what matters). */
            if (rx_par)
                for (int i = 0; i < n; i++)
                    rx_par[i] = (uint8_t)(((par >> i) & 1) ? !!__builtin_parity(rx[i]) : !__builtin_parity(rx[i]));
            if (n > 0) {
                emu_hot_enter();                   /* fallback when a short frame hid RXFRAMESTART from polling */
                return n;
            }
            emu_hot_leave();
            continue;
        }
        if (xTaskGetTickCount() > deadline) {
            emu_hot_leave();
            return 0;
        }
    }
}

int hal_rfid_hf_emu_send(const uint8_t *tosend, int len)
{
    if (s_mode != RFID_HF_EMU || !tosend || len <= 1) {
        emu_hot_leave();
        return RFID_ERR_UNSUPP;
    }
    int rc = emu_tx_bits(tosend, len) < 0 ? RFID_ERR_UNSUPP : 0;
    emu_hot_leave();
    return rc;
}

/* Keep the callback stream unsupported: NFCT EasyDMA may fetch ahead, so its
 * buffer must be complete before STARTTX. The module uses this buffer fallback. */
int hal_rfid_hf_emu_send_stream_buf(const uint8_t *tosend, int len)
{
    return hal_rfid_hf_emu_send(tosend, len);
}

/* ---- Passive dual-receiver HF sniff (see the rfid-chameleon-sniff design) ----
 * True simultaneous bidirectional eavesdrop of an external reader<->card exchange, TX never fired: the nRF
 * NFCT decodes the reader's modified-Miller (parity-preserving via nfct_unwrap), the Ci522 correlator decodes
 * the card's 847 kHz load-modulation - both on the shared coil at once. A free-running 1 MHz TIMER stamps every
 * frame (PPI HW-captures each RXFRAMEEND); the card side is coalesced (the Ci522 emits one byte at a time) and
 * gated to the between-reader-frames window (bursts during / just after the reader's TX are Miller artifacts).
 * Emits the host sniff wire-format to dump_path - a leading "L<couple> f o v m p1000" header (p1000 = the
 * timestamps are already us) then one "<R|C> <start_us> <end_us> <hex>[!].." line per frame - and returns the
 * frame count (RFID_ERR_TIMEOUT with the file cleared if the window was silent). Runs one window; the app loops. */
int hal_rfid_hf_sniff(const char *dump_path, uint32_t timeout_ms)
{
    if (!s_spi_up || s_mode != RFID_HF_READER || !dump_path) return RFID_ERR_UNSUPP;
    if (!timeout_ms) timeout_ms = 1500;

    const int cap = 4096;                         /* <= the app's read buffer; frames stop at cap-300 */
    char *txt = pvPortMalloc((size_t)cap);        /* heap only for the sniff's duration (idle-RAM discipline) */
    if (!txt) return RFID_ERR_UNSUPP;

    /* S140 owns CLOCK while BLE is enabled. A direct TASKS_HFCLKSTART write is
     * trapped as APP_MEMACC, so acquire the crystal through the SoftDevice. */
    if (!cu_hfclk_request()) {
        vPortFree(txt);
        return RFID_ERR_UNSUPP;
    }

    /* ---- both receivers on the coil, passive ---- */
    pin_out(PIN_HF_ANTSEL, 1);                    /* route the shared antenna to the NFCT tap (was reader-only) */
    pin_out(PIN_RDR_POWER, 1);                    /* Ci522 alive (already on in HF_READER) */
    rc522_reset();
    vTaskDelay(pdMS_TO_TICKS(60));
    /* Ci522 -> passive card RX. Operating point picked by an on-device sweep vs a pm3 reading a tag (bad-frame
     * rate ~17% -> ~4%): RxModeReg=0x0E is the big win - RxMultiple(bit3)=1 makes the receiver delimit frames
     * properly (free-running byte-drain otherwise glued leading Miller-pause noise onto each response); RxCRCEn
     * (bit7)=0 still keeps all bytes (anticoll has no CRC). RxThreshold=0x88 (MinLevel 8) is a noise gate that
     * rejects the weak leading-edge junk while the strong on-coil response still decodes. Gain stays max (0x70);
     * lowering it made framing worse. TxControlReg=0 keeps the field off (passive). */
    reg_write(TxControlReg, 0x00);
    reg_write(TModeReg, 0x00);
    reg_write(RxModeReg, 0x0E);
    reg_write(RFCfgReg, 0x70);
    reg_write(RxThresholdReg, 0x88);
    reg_write(DemodReg, 0x4D);
    reg_write(ModeReg, 0x3D);
    reg_set(FIFOLevelReg, 0x80);                  /* flush FIFO */
    reg_write(ComIrqReg, 0x7F);
    reg_write(CommandReg, PCD_RECEIVE);           /* free-running receive, no TX */

    /* NFCT -> passive reader RX. FRAMEDELAYMODE=3 (WindowGrid) is load-bearing: FreeRun never trips the field
     * detector in side-eavesdrop geometry. Auto-collision-resolution disabled -> deliver any demodulated frame. */
    NRF_NFCT->FRAMEDELAYMODE = 3;
    NRF_NFCT->SENSRES = (NRF_NFCT->SENSRES & ~NFCT_SENSRES_BITFRAMESDD_Msk)
                      | (NFCT_SENSRES_BITFRAMESDD_SDD00100 << NFCT_SENSRES_BITFRAMESDD_Pos);
    *(volatile uint32_t *)0x4000559C |= 0x1UL;    /* AUTOCOLRESCONFIG.MODE = Disabled */
    NRF_NFCT->EVENTS_FIELDDETECTED = 0;
    NRF_NFCT->EVENTS_FIELDLOST = 0;
    NRF_NFCT->EVENTS_RXFRAMESTART = 0;
    NRF_NFCT->EVENTS_RXFRAMEEND = 0;
    NRF_NFCT->EVENTS_RXERROR = 0;
    NRF_NFCT->ERRORSTATUS = 0xFFFFFFFFUL;
    NRF_NFCT->TASKS_SENSE = 1;

    /* us timebase: TIMER2 free-run @1 MHz; PPI CH[3] HW-stamps each RXFRAMEEND into CC[0] (no poll latency). */
    NRF_TIMER2->MODE      = TIMER_MODE_MODE_Timer;
    NRF_TIMER2->BITMODE   = TIMER_BITMODE_BITMODE_32Bit;
    NRF_TIMER2->PRESCALER = 4;                    /* 16 MHz / 16 = 1 MHz -> 1 tick = 1 us */
    NRF_TIMER2->TASKS_CLEAR = 1;
    NRF_TIMER2->TASKS_START = 1;
    NRF_PPI->CH[3].EEP = (uint32_t)(uintptr_t)&NRF_NFCT->EVENTS_RXFRAMEEND;
    NRF_PPI->CH[3].TEP = (uint32_t)(uintptr_t)&NRF_TIMER2->TASKS_CAPTURE[0];
    NRF_PPI->CHENSET = (1u << 3);

    const int FSTART = 40;                        /* reserve the front for the L-header, prepended at the end */
    int pos = FSTART, valid = 0, activated = 0, in_reader_tx = 0;
    uint32_t iters = 0;
    uint8_t  cbuf[32]; int cbn = 0;               /* card-byte coalescing buffer (one demod byte arrives at a time) */
    unsigned long cbt0 = 0, cbt_last = 0, rx_end_us = 0, r_start_us = 0;
    const unsigned long CARD_GUARD_US = 180;      /* drop Ci522 bytes <180 us after a reader frame (Miller edge) */

    /* Emit the coalesced card frame: start = first byte, end = last byte + ~1 byte-time. The Ci522's
     * Manchester decoder reads one extra empty byte-slot (0x00) after every response ends - drop that single
     * trailing 00 so the host sees the true frame length (ATQA 44 00, UID+BCC ..36, SAK ..17) and its
     * length-keyed annotation + BCC/CRC check land. A lone-00 frame collapses to nothing (pure artifact). */
#define FLUSH_CARD() do { \
        if (cbn > 0 && cbuf[cbn - 1] == 0x00) cbn--; \
        if (cbn > 0) { \
            if (pos < cap - 200) { \
                valid++; \
                pos += snprintf(txt + pos, (size_t)(cap - pos), "C %lu %lu", cbt0, cbt_last + 86); \
                for (int _i = 0; _i < cbn; _i++) pos += snprintf(txt + pos, (size_t)(cap - pos), " %02X", cbuf[_i]); \
                txt[pos++] = '\n'; \
            } \
            cbn = 0; \
        } \
    } while (0)

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (xTaskGetTickCount() < deadline) {
        /* ---- reader direction (NFCT) ---- */
        if (!activated && NRF_NFCT->EVENTS_FIELDDETECTED) {
            NRF_NFCT->EVENTS_FIELDDETECTED = 0;
            vTaskDelay(pdMS_TO_TICKS(1));
            NRF_NFCT->TASKS_ACTIVATE = 1;
            NRF_NFCT->RXD.FRAMECONFIG = 0x04;     /* NoParity: HW delivers raw bits -> nfct_unwrap keeps parity */
            NRF_NFCT->PACKETPTR = (uint32_t)(uintptr_t)s_nfc_rx;
            NRF_NFCT->MAXLEN = sizeof s_nfc_rx;
            NRF_NFCT->TASKS_ENABLERXDATA = 1;
            activated = 1;
        }
        if (NRF_NFCT->EVENTS_RXFRAMESTART) {
            NRF_NFCT->EVENTS_RXFRAMESTART = 0;
            in_reader_tx = 1;
            NRF_TIMER2->TASKS_CAPTURE[2] = 1;
            r_start_us = NRF_TIMER2->CC[2];
        }
        int rgot = NRF_NFCT->EVENTS_RXFRAMEEND ? 1 : (NRF_NFCT->EVENTS_RXERROR ? 2 : 0);
        if (rgot == 1) NRF_NFCT->EVENTS_RXFRAMEEND = 0;
        if (rgot == 2) NRF_NFCT->EVENTS_RXERROR = 0;
        if (rgot) {
            in_reader_tx = 0;                     /* reader frame done -> the card may respond now */
            rx_end_us = NRF_TIMER2->CC[0];        /* PPI-captured exact RXFRAMEEND time (us) */
            int total_bits = (int)(NRF_NFCT->RXD.AMOUNT & 0xFFF);
            uint8_t data[64]; uint32_t rpar;
            int n = nfct_unwrap(s_nfc_rx, total_bits, data, &rpar, (int)sizeof data);
            FLUSH_CARD();                         /* the card's response to the PREVIOUS reader frame goes first */
            if (n > 0 && pos < cap - 300) {
                valid++;
                pos += snprintf(txt + pos, (size_t)(cap - pos), "R %lu %lu", r_start_us, rx_end_us);
                for (int i = 0; i < n; i++)       /* '!' marks a byte whose parity disagreed with ISO14443-A odd */
                    pos += snprintf(txt + pos, (size_t)(cap - pos), " %02X%s", data[i], (rpar >> i) & 1 ? "!" : "");
                txt[pos++] = '\n';
            }
            NRF_NFCT->TASKS_ENABLERXDATA = 1;     /* re-arm for the next frame */
        }
        if (NRF_NFCT->EVENTS_FIELDLOST) { NRF_NFCT->EVENTS_FIELDLOST = 0; activated = 0; in_reader_tx = 0; NRF_NFCT->TASKS_SENSE = 1; }

        /* ---- card direction (Ci522), gated + coalesced ---- */
        uint8_t lvl = (uint8_t)(reg_read(FIFOLevelReg) & 0x7F);
        if (lvl > 0) {
            uint8_t buf[64];
            int n = lvl > 64 ? 64 : lvl;
            reg_read_fifo(buf, n);
            reg_write(ComIrqReg, 0x7F);
            NRF_TIMER2->TASKS_CAPTURE[1] = 1;
            unsigned long t = NRF_TIMER2->CC[1];
            if (!(in_reader_tx || (t - rx_end_us) < CARD_GUARD_US)) {   /* keep only real card responses */
                if (cbn > 0 && (t - cbt_last) > 300) FLUSH_CARD();      /* gap -> a distinct card frame */
                if (cbn == 0) cbt0 = t;
                for (int i = 0; i < n && cbn < (int)sizeof cbuf; i++) cbuf[cbn++] = buf[i];
                cbt_last = t;
            }
        }
        if ((++iters & 0xFF) == 0) taskYIELD();
    }
    FLUSH_CARD();
#undef FLUSH_CARD

    /* ---- restore the reader front-end (mirror set_mode(HF_READER) so a re-entrant call is clean) ---- */
    NRF_PPI->CHENCLR = (1u << 3);
    NRF_TIMER2->TASKS_STOP = 1;
    reg_write(CommandReg, PCD_IDLE);
    reg_clear(TxControlReg, 0x03);
    NRF_NFCT->TASKS_SENSE = 1;
    rc522_reset();
    pin_out(PIN_HF_ANTSEL, 0);

    if (valid == 0) {                             /* silent window: clear the file so the app doesn't reprint a stale trace */
        cu_hfclk_release();
        vPortFree(txt);
        vfs_write_file(dump_path, "", 0);
        return RFID_ERR_TIMEOUT;
    }

    /* Prepend the header into the reserved slot. Coupling / field strength is reported as unknown (couple
     * sentinel 9), not estimated: the CU has no way to measure it - NFCT FIELDPRESENT is binary (datasheet
     * 6.14.13.31) and there is no envelope ADC on the coil - and capture quality is not a proxy for it (a card
     * emitting bad frames under perfect coupling would read "weak"). Only the frame count is real. The host
     * renders couple>=3 as a plain "unknown" (vs the Flipper's measured 0/1/2 = strong/fair/weak). p1000 tells
     * the host the timestamps are already us. */
    char h[40];
    int hl = snprintf(h, sizeof h, "L9 v%d p1000\n", valid);
    memcpy(txt + (FSTART - hl), h, (size_t)hl);
    vfs_write_file(dump_path, txt + (FSTART - hl), (uint32_t)(pos - (FSTART - hl)));
    cu_hfclk_release();
    vPortFree(txt);
    return valid;
}

int hal_rfid_hf_transceive(const uint8_t *tx, int tx_bits, uint32_t flags,
                           uint8_t *rx, int rx_cap, uint32_t timeout_us)
{
    if (!s_spi_up || s_mode != RFID_HF_READER) return RFID_ERR_UNSUPP;
    if (tx_bits <= 0) return RFID_ERR_FRAMING;

    int tx_bytes = (tx_bits + 7) / 8;
    uint8_t tx_last_bits = (uint8_t)(tx_bits & 7);   /* 0 => full final byte */

    /* Parity: ISO14443-A hardware parity on by default; suppress for the short
     * anticollision/REQA frames (RFID_HF_NO_PARITY). */
    if (flags & RFID_HF_NO_PARITY) reg_set(MfRxReg, 0x10);
    else                           reg_clear(MfRxReg, 0x10);

    reg_write(CommandReg, PCD_IDLE);
    reg_clear(ComIrqReg, 0x80);       /* clear IRQ flags */
    reg_set(FIFOLevelReg, 0x80);      /* flush FIFO */
    reg_write_fifo(tx, tx_bytes);
    reg_write(CommandReg, PCD_TRANSCEIVE);
    reg_write(BitFramingReg, tx_last_bits);
    reg_set(BitFramingReg, 0x80);     /* StartSend */

    uint32_t ms = timeout_us ? (timeout_us / 1000) : 25;
    if (ms == 0) ms = 1;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(ms) + 1;

    uint8_t irq = 0;
    bool timed_out = true;
    /* waitFor = RxIRq | IdleIRq (0x20 | 0x10) */
    while (xTaskGetTickCount() <= deadline) {
        irq = reg_read(ComIrqReg);
        if (irq & 0x30) { timed_out = false; break; }
    }

    reg_clear(BitFramingReg, 0x80);   /* stop send */
    reg_write(BitFramingReg, 0x00);

    if (timed_out) return RFID_ERR_TIMEOUT;

    uint8_t err = reg_read(ErrorReg);
    if (err & 0x01) return RFID_ERR_FRAMING;    /* ProtocolErr */
    /* ParityErr (0x02) is not fatal: the anticoll/nonce data is still delivered to the FIFO, and every mfc
     * frame is validated downstream (BCC, then SELECT, then CRC on reads, then Crypto1 on auth). On a
     * marginal Ci522 signal the parity flag trips intermittently even though the UID is correct. Let the
     * caller validate. */
    if (err & 0x04) return RFID_ERR_CRC;        /* CRCErr     */
    if (err & 0x08) return RFID_ERR_COLLISION;  /* CollErr    */

    int n = reg_read(FIFOLevelReg);
    uint8_t last = reg_read(Control522Reg) & 0x07;
    int rx_bits = last ? ((n - 1) * 8 + last) : (n * 8);
    if (n > rx_cap) { return RFID_ERR_OVERFLOW; }
    reg_read_fifo(rx, n);

    if (rx_bits == 0) return RFID_ERR_TIMEOUT;   /* no usable response */
    return rx_bits;
}

/* Custom-parity transceive for Crypto1-encrypted 14443-A frames (nested-nonce attack, MIFARE auth). The
 * MFRC522's hardware ISO parity is wrong for encrypted frames, whose parity bits are keystream-encrypted, so
 * we run ParityDisable and interleave the caller's parity bits into the raw bit stream ourselves (8 data
 * LSB-first + 1 parity per byte), then split them back out of the response. Ported from the ChameleonUltra
 * reference pcd_14a_reader_bits_transfer (rc522.c). No CRC. Returns received data bits (parity stripped). */
int hal_rfid_hf_transceive_par(const uint8_t *tx, int nbytes, const uint8_t *par,
                               uint8_t *rx, uint8_t *rx_par, int rx_cap, uint32_t timeout_us)
{
    if (!s_spi_up || s_mode != RFID_HF_READER) return RFID_ERR_UNSUPP;
    if (nbytes <= 0 || nbytes > 40) return RFID_ERR_FRAMING;

    static uint8_t buf[64];
    int i, dataLen, modulus;
    buf[0] = tx[0];
    if (nbytes > 1) {                                  /* pack tx + par -> 9-bits-per-byte stream */
        modulus = dataLen = nbytes;
        buf[1] = (uint8_t)((par[0] & 1) | (tx[1] << 1));
        for (i = 2; i < dataLen; i++)
            buf[i] = (uint8_t)(((par[i - 1] & 1) << (i - 1)) | (tx[i - 1] >> (9 - i)) | (tx[i] << i));
        buf[dataLen] = (uint8_t)(((par[dataLen - 1] & 1) << (i - 1)) | (tx[dataLen - 1] >> (9 - i)));
        dataLen += 1;
    } else {                                           /* lone byte: 8 data bits, no parity (matches CU path) */
        modulus = 0; dataLen = 1;
    }

    reg_set(MfRxReg, 0x10);                            /* ParityDisable: parity rides as data */
    reg_write(CommandReg, PCD_IDLE);
    reg_clear(ComIrqReg, 0x80);
    reg_set(FIFOLevelReg, 0x80);
    reg_write_fifo(buf, dataLen);
    reg_write(CommandReg, PCD_TRANSCEIVE);
    reg_write(BitFramingReg, (uint8_t)(modulus & 0x07));   /* TxLastBits = (nbytes*9) mod 8 = nbytes mod 8 */
    reg_set(BitFramingReg, 0x80);                      /* StartSend */

    uint32_t ms = timeout_us ? (timeout_us / 1000) : 25;
    if (ms == 0) ms = 1;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(ms) + 1;
    bool timed_out = true;
    while (xTaskGetTickCount() <= deadline) { if (reg_read(ComIrqReg) & 0x30) { timed_out = false; break; } }

    reg_clear(BitFramingReg, 0x80);
    reg_write(BitFramingReg, 0x00);
    reg_clear(MfRxReg, 0x10);                          /* restore hardware parity for normal frames */

    if (timed_out) return RFID_ERR_TIMEOUT;
    /* Encrypted parity is non-ISO by design, so we do not fail on ErrorReg parity bits - just read the raw
     * stream; a genuine no-answer shows up as an empty FIFO below. */
    int n = reg_read(FIFOLevelReg);
    uint8_t last = reg_read(Control522Reg) & 0x07;
    int rx_bits = last ? ((n - 1) * 8 + last) : (n * 8);
    if (n <= 0 || rx_bits <= 0) return RFID_ERR_TIMEOUT;
    if (n > (int)sizeof buf) return RFID_ERR_OVERFLOW;
    reg_read_fifo(buf, n);

    /* Split the interleaved 9-bits-per-byte stream (8 data LSB-first + 1 parity) back into data + parity.
     * General bit-level extraction: data byte d occupies stream bits [9d, 9d+7], its parity bit 9d+8. This
     * works for long responses (an encrypted block read is 16+2 CRC = 18 bytes); the CU's shift-by-i trick
     * silently zeroes out at byte index >=8 and only ever handled <=8-byte nonce frames. A short frame
     * (<9 bits, e.g. a 4-bit NAK) carries no parity - pass it straight through. */
    if (rx_bits < 9) { rx[0] = buf[0]; return rx_bits; }
    int kbytes = rx_bits / 9;                          /* each data byte carries one interleaved parity bit */
    if (kbytes > rx_cap) return RFID_ERR_OVERFLOW;
    for (int d = 0; d < kbytes; d++) {
        uint8_t db = 0;
        for (int p = 0; p < 8; p++) { int pos = d * 9 + p; db |= (uint8_t)(((buf[pos >> 3] >> (pos & 7)) & 1) << p); }
        rx[d] = db;
        if (rx_par) { int pp = d * 9 + 8; rx_par[d] = (uint8_t)((buf[pp >> 3] >> (pp & 7)) & 1); }
    }
    return kbytes * 8;
}

/* ---- LF 125 kHz reader ---- */

/* ---- T5577 downlink (OOK gap modulation) + block read ----
 * T55 command bits are sent as brief field gaps: charge the tag, drop the carrier for a start gap, then per
 * bit re-raise the carrier for a long (1) or short (0) burst followed by a write gap, then leave it on. The
 * complete waveform is played by PWM EasyDMA so interrupts cannot stretch its carrier-counted timings. The
 * reply is demodulated into the anchored run stream consumed by the shared T5577 module. */
/* Downlink gap timing, in Tc (1 carrier cycle = 8 us at 125 kHz), taken from the stock ChameleonUltra
 * firmware (lf_t55xx_data.c), which is known to program T5577s on this exact frontend. */
#define T55_START_GAP_US  240   /* 30 Tc */
#define T55_WRITE_GAP_US   72   /*  9 Tc - field OFF between bits */
#define T55_WRITE_0_US    192   /* 24 Tc - field ON before the gap => bit 0 */
#define T55_WRITE_1_US    432   /* 54 Tc - field ON before the gap => bit 1 */
#define T55_POWERUP_MS      8
#define T55_PROGRAM_MS     10
#define T55_READ_RESET_MS  20
/* The Chameleon's AC-coupled receive chain needs several response repetitions to recover from the downlink
 * gaps. The tag repeats continuously, so wait with interrupts enabled and then arm on a hardware RF/64
 * phase below; this loses no information and avoids accepting the visibly distorted early copies. */
#define T55_REPLY_SETTLE_MS 80
#define LF_T55_WARM        256   /* carrier cycles discarded before eye calibration / output */
#define LF_T55_SKIP        150   /* EM4100 path: lead-in excluded from its band estimate + demod */
#define T55_HB             32     /* half-bit width in carrier cycles = samples (RF/64: 32/half-bit) */

/* Emit the complete downlink from PWM EasyDMA, one compare value per carrier period. CPU busy-delays and live
 * writes to a sequence that EasyDMA is reading are both vulnerable to SoftDevice timing, whereas this array
 * fixes every on/gap interval in hardware. The final ON value is held after SEQEND1; that same event clears
 * TIMER3 through PPI, establishing response-block phase zero at the actual field restore. */
static int lf_t55_cmd(const uint8_t *p, int nbits, uint16_t cmd_duty, uint16_t hold_duty,
                      bool cmd_inverted, bool hold_inverted)
{
    if (!p || nbits <= 0) return -1;
    int periods = T55_START_GAP_US / 8 + 1;
    for (int i = 0; i < nbits; i++) {
        int add = (p[i] ? T55_WRITE_1_US : T55_WRITE_0_US) / 8 + T55_WRITE_GAP_US / 8;
        if (periods > LF_ENV_SAMPLES - add) return -1;
        periods += add;
    }

    uint16_t cmd_polarity = cmd_inverted ? LF_PWM_INVERTED : 0;
    uint16_t hold_polarity = hold_inverted ? LF_PWM_INVERTED : 0;
    uint16_t cmd_word = cmd_polarity | cmd_duty;
    uint16_t hold_word = hold_polarity | hold_duty;
    uint16_t gap_word = cmd_polarity;              /* compare zero: constant low normally, high if inverted */

    lf_start();                                    /* field on: charge the tag */
    s_pwm_val = cmd_word;
    vTaskDelay(pdMS_TO_TICKS(T55_POWERUP_MS));

    uint16_t *seq = (uint16_t *)(void *)s_lf_env;
    int n = 0;
    for (int i = 0; i < T55_START_GAP_US / 8; i++) seq[n++] = gap_word;
    for (int i = 0; i < nbits; i++) {
        int on = (p[i] ? T55_WRITE_1_US : T55_WRITE_0_US) / 8;
        for (int j = 0; j < on; j++) seq[n++] = cmd_word;
        for (int j = 0; j < T55_WRITE_GAP_US / 8; j++) seq[n++] = gap_word;
    }
    seq[n++] = hold_word;                           /* restored field, held after playback */

    uint32_t primask = __get_PRIMASK();
    __disable_irq();                                /* only the stop/reconfigure/start handoff (~one carrier) */
    NRF_PWM0->SHORTS = 0;
    NRF_PWM0->EVENTS_STOPPED = 0;
    NRF_PWM0->TASKS_STOP = 1;
    for (uint32_t g = 0; !NRF_PWM0->EVENTS_STOPPED && g < 1000000; g++) { }
    NRF_PWM0->LOOP = 1;                             /* nrfx simple-playback count=1 starts sequence 1 */
    NRF_PWM0->SEQ[0].PTR = (uint32_t)(uintptr_t)seq;
    NRF_PWM0->SEQ[0].CNT = (uint32_t)n;
    NRF_PWM0->SEQ[0].REFRESH = 0;
    NRF_PWM0->SEQ[0].ENDDELAY = 0;
    NRF_PWM0->SEQ[1].PTR = (uint32_t)(uintptr_t)seq;
    NRF_PWM0->SEQ[1].CNT = (uint32_t)n;
    NRF_PWM0->SEQ[1].REFRESH = 0;
    NRF_PWM0->SEQ[1].ENDDELAY = 0;
    NRF_PWM0->EVENTS_SEQEND[1] = 0;
    NRF_PPI->CHENSET = (1u << LF_PPI_PHASE) | (1u << LF_PPI_REPHASE);
    NRF_PWM0->TASKS_SEQSTART[1] = 1;
    s_pwm_running = true;
    if (!primask) __enable_irq();

    for (uint32_t g = 0; !NRF_PWM0->EVENTS_SEQEND[1] && g < 10000000; g++) { }
    int complete = NRF_PWM0->EVENTS_SEQEND[1] != 0;
    NRF_PPI->CHENCLR = (1u << LF_PPI_REPHASE);      /* one SEQEND only, but leave no stale endpoint enabled */
    if (!complete) {
        lf_stop();
        return -1;
    }
    return 0;
}

int hal_rfid_lf_field(bool on, uint32_t divisor)
{
    (void)divisor;
    if (s_mode != RFID_LF_READER) return RFID_ERR_UNSUPP;
    if (on) { lf_start(); vTaskDelay(pdMS_TO_TICKS(150)); /* field + op-amp bias settle (AC-coupled amp needs >50 ms) */ }
    else lf_stop();
    return 0;
}

/* Capture the LF envelope with the SAADC and turn it into a stream of inter-edge
 * intervals (carrier-cycle counts) for the EM4100 decoder. `opts` is unused (the
 * capture length is fixed at ~2 frames). Field must be on.
 *
 * The op-amp output is a small analog envelope oscillating at the bit clock, with
 * the Manchester data as timing shifts of that oscillation. We DMA it at the
 * carrier rate (1 sample/cycle, so a sample count == a carrier-cycle count), then
 * recover edges by mean-crossing: a rising cross through mean+band marks a bit
 * edge; the interval to the previous edge is the inter-edge count the decoder
 * classifies as 1T/1.5T/2T. A hysteresis band (re-arm only below mean-band) stops
 * noise near the mid from double-triggering. This replaces the old GPIOTE digital
 * edge path, which saw nothing once the field was dropped to the low duty needed
 * to keep the amps linear (the linear envelope never reaches the logic level). */
static int lf_demod(uint8_t *buf, int max)   /* SAADC envelope -> inter-edge run-lengths (EM4100 + T55 reply) */
{
    if (lf_sample(s_lf_env, LF_ENV_SAMPLES) != 0) return RFID_ERR_TIMEOUT;

    /* Detrend with an exponential moving average (~128-sample / 2-bit time
     * constant) so the threshold tracks the op-amp's settling DC drift. A fixed
     * global-mean threshold misses most edges whenever that drift exceeds the
     * bit-swing amplitude. Overwrite the buffer in
     * place with the detrended signal and record its swing for the hysteresis
     * band. dc is Q8 fixed point. */
    int32_t dc = (int32_t)s_lf_env[0] << 8;
    int acmin = 0, acmax = 0;
    for (int i = 0; i < LF_ENV_SAMPLES; i++) {
        int32_t v = (int32_t)s_lf_env[i] << 8;
        dc += (v - dc) >> 7;
        int ac = (int)((v - dc) >> 8);
        s_lf_env[i] = (int16_t)ac;
        if (i >= LF_T55_SKIP) {                /* swing stats from the steady region: a lead-in transient is
                                                * many times the reply and would size the band for itself */
            if (ac < acmin) acmin = ac;
            if (ac > acmax) acmax = ac;
        }
    }
    if (acmax - acmin < 4) return 0;           /* flat: no field/tag, no edges */
    int band = (acmax - acmin) / 10;           /* hysteresis = 10% of pk-pk */
    if (band < 1) band = 1;

    /* Bit edges by both-edge hysteresis: cross +band to go high, -band to go low, so every transition is
     * caught and mid-level noise still can't double-trigger. Each crossing pair is one level run (~a
     * half-bit); the EM4100 decoder wants same-direction (1T/1.5T/2T) intervals, so emit the sum of each
     * non-overlapping pair of runs - one full period between like edges. */
    /* Timestamp one edge direction and emit the gap between consecutive such edges - the same quantity the
     * stock firmware gets from GPIOTE on one polarity plus a carrier-cycle counter, and what the decoder's
     * 1T/1.5T/2T classes mean. Pairing adjacent level runs instead cannot work: a pair is only the correct
     * like-edge period if it starts on the right run, and both pairings of this signal land in the same
     * polarity class anyway. Polarity is handled in the decoder, which tries both Manchester conventions.
     *
     * Alternate the direction per capture: rising- and falling-edge intervals are different sequences drawn
     * from the same signal, so a glitch that corrupts one often leaves the other intact, and
     * rfid_lf_em4100_read retries. Each gap is snapped to a whole number of half-bits - the envelope is
     * asymmetric (~29 low / ~35 high) and raw gaps landed outside the decoder's +-16 tolerance, where a
     * single invalid value resets its 64-bit frame accumulator. */
    static int s_edge_dir;
    int want_rise = (s_edge_dir++ & 1);
    int count = 0, state = 0, prev = -1;
    for (int i = LF_T55_SKIP; i < LF_ENV_SAMPLES && count < max; i++) {
        int rise;
        if      (state == 0 && s_lf_env[i] >  band) { state = 1; rise = 1; }
        else if (state == 1 && s_lf_env[i] < -band) { state = 0; rise = 0; }
        else continue;                             /* no crossing on this sample */
        if (rise != want_rise) continue;           /* consecutive LIKE edges bound one period */
        if (prev >= 0) {
            int q = ((i - prev) + T55_HB / 2) / T55_HB;   /* whole half-bits */
            if (q < 1) q = 1;
            int d = q * T55_HB;
            buf[count++] = (d > 0xFF) ? 0xFF : (uint8_t)d;
        }
        prev = i;
    }
    return count;
}

int hal_rfid_lf_acquire(uint8_t *buf, int max, uint32_t opts)
{
    (void)opts;
    if (s_mode != RFID_LF_READER || !buf || max <= 0) return RFID_ERR_UNSUPP;
    return lf_demod(buf, max);
}

/* T5577 WRITE: send the opcode+password+data+addr bits as field gaps, hold the field steady while the EEPROM
 * commits, then drop it. `opts` unused. */
int hal_rfid_lf_modulate(const uint8_t *p, int nbits, uint32_t o)
{
    (void)o;
    if (s_mode != RFID_LF_READER || !p || nbits <= 0) return RFID_ERR_UNSUPP;
    /* Match the stock Chameleon write field exactly: 50% PWM with inverted output polarity. Besides supplying
     * enough energy through a worst-case all-zero command, inversion makes a compare-zero gap park the driver
     * high, as nrfx_pwm_stop() does with NRFX_PWM_PIN_INVERTED in the proven upstream implementation. */
    if (lf_t55_cmd(p, nbits, LF_DUTY_WRITE, LF_DUTY_WRITE, true, true) != 0)
        return RFID_ERR_TIMEOUT;
    vTaskDelay(pdMS_TO_TICKS(T55_PROGRAM_MS));      /* EEPROM programming window, field steady */
    lf_stop();
    return 0;
}

/* Scratch used only while calibrating the T5577 envelope slicer. */
static uint8_t s_t55_hb[256];
static int s_t55_nhb;

#define T55_EYE_WIDTH 20
#define T55_EYE_CELLS 256
#define T55_BLOCK_CELLS 64
#define T55_EYE_PHASE_CUT 3    /* opposite the hardware-anchored transition (normally phase 17..20) */
#define T55_EYE_PHASE_GUARD 2
#define T55_EYE_MIN_QUALITY 192
static int32_t s_t55_eye[T55_EYE_CELLS];

static void lf_t55_measure_cells(int start, int sample)
{
    for (int h = 0; h < T55_EYE_CELLS; h++) {
        int cell = start + h * T55_HB + sample;
        int32_t sum = 0;
        for (int j = 0; j < T55_EYE_WIDTH; j++) sum += s_lf_env[cell + j];
        s_t55_eye[h] = sum;
    }

    /* Each 64-half-bit Manchester period has exactly 32 high and 32 low cells at every rotation. Its mean is
     * therefore a data-independent baseline measurement. Remove one baseline per physical repetition so the
     * frozen eye threshold does not follow the AC-coupled front end's slow drift through the capture. */
    for (int base = 0; base < T55_EYE_CELLS; base += T55_BLOCK_CELLS) {
        int32_t sum = 0;
        for (int h = 0; h < T55_BLOCK_CELLS; h++) sum += s_t55_eye[base + h];
        int32_t mean = sum / T55_BLOCK_CELLS;
        for (int h = 0; h < T55_BLOCK_CELLS; h++) s_t55_eye[base + h] -= mean;
    }
}

static void lf_t55_sort_cells(int first, int count)
{
    for (int i = first + 1; i < first + count; i++) {
        int32_t v = s_t55_eye[i]; int j = i;
        while (j > first && s_t55_eye[j - 1] > v) {
            s_t55_eye[j] = s_t55_eye[j - 1]; j--;
        }
        s_t55_eye[j] = v;
    }
}

/* Return the mean physical eye quality across the four complete response periods at one fine phase. The
 * score measures inner-cluster separation relative to within-level spread; unlike one adjacent-rank gap, it
 * follows the broad eye and cannot be won by one noisy order-statistic spacing. Averaging makes the phase
 * curve describe the common analog transition instead of allowing one noisy repetition to create its valley.
 * Each period also gets its own balanced-population midpoint: the AC-coupled frontend's level centre moves
 * through a capture even after mean removal, so pooling otherwise-open eyes under one threshold can close
 * the combined eye. These four thresholds are fixed analog calibration, not decoded-value selection. */
static int lf_t55_eye_at(int start, int sample, int32_t threshold[4], int period_quality[4])
{
    lf_t55_measure_cells(start, sample);
    int quality_sum = 0;
    for (int base = 0; base < T55_EYE_CELLS; base += T55_BLOCK_CELLS) {
        lf_t55_sort_cells(base, T55_BLOCK_CELLS);
        int32_t lo = s_t55_eye[base + T55_BLOCK_CELLS / 2 - 1];
        int32_t hi = s_t55_eye[base + T55_BLOCK_CELLS / 2];

        /* Ranks 28/35 are several cells inside the two 32-cell populations; ranks 3/60 estimate their
         * outer spread without letting extrema dominate. A tied central pair is a closed slicing gap,
         * regardless of the wider ranks. Q10 scaling keeps this integer-only. */
        int32_t inner = s_t55_eye[base + 35] - s_t55_eye[base + 28];
        int32_t spread = (s_t55_eye[base + 28] - s_t55_eye[base + 3]) +
                         (s_t55_eye[base + 60] - s_t55_eye[base + 35]);
        int quality = hi > lo ? (int)(((int64_t)inner << 10) / (spread + 1)) : 0;
        quality_sum += quality;
        if (threshold) threshold[base / T55_BLOCK_CELLS] = lo + (hi - lo) / 2;
        if (period_quality) period_quality[base / T55_BLOCK_CELLS] = quality;
    }
    return quality_sum / (T55_EYE_CELLS / T55_BLOCK_CELLS);
}

/* Calibrate the data eye without using decoded values. Every Manchester block has exactly 32 high and 32
 * low half-bits, so four complete periods contain an exactly balanced population at any block rotation.
 * The broad minimum of the normalized cluster-quality curve marks where the integration window straddles the
 * analog transition. Move its start back by half a window so the calibrated window ends just before that
 * edge, after the receive chain has had almost the complete cell to recover.
 * A guarded, unwrapped fine-phase branch preserves which hardware-anchored 32-cycle cell owns each sample
 * across phase 31<->0. Decoded bits never choose, retry, or repair either the phase or level calibration. */
static int lf_demod_t55(uint8_t *buf, int cap)
{
    s_t55_nhb = 0;
    const int start = LF_T55_WARM + 2 * T55_HB;
    int32_t threshold[T55_EYE_CELLS / T55_BLOCK_CELLS];
    int quality[T55_HB];
    for (int sample = 0; sample < T55_HB; sample++)
        quality[sample] = lf_t55_eye_at(start, sample, NULL, NULL);

    int edge_score = 0x7FFFFFFF, edge = -1;
    for (int p = 0; p < T55_HB; p++) {
        int score = quality[(p + T55_HB - 1) % T55_HB] + quality[p] +
                    quality[(p + 1) % T55_HB];
        if (score < edge_score) {
            edge_score = score;
            edge = p;
        }
    }
    if (edge < 0) return 0;

    /* Fine phase is circular, but the exported half-bit grid is not. Keep the phase branch cut well away
     * from the observed transition and reject its guard band: config and target can then jitter through
     * phase 31<->0 without silently changing which physical cell owns a sample. */
    int cut_distance = edge - T55_EYE_PHASE_CUT;
    if (cut_distance < 0) cut_distance = -cut_distance;
    if (cut_distance > T55_HB / 2) cut_distance = T55_HB - cut_distance;
    if (cut_distance <= T55_EYE_PHASE_GUARD) return 0;
    int unwrapped_edge = edge < T55_EYE_PHASE_CUT ? edge + T55_HB : edge;
    int sample = unwrapped_edge - T55_EYE_WIDTH / 2;

    int eye_quality = lf_t55_eye_at(start, sample, threshold, NULL);
    /* A Q10 score of 192 requires the mean inner-cluster separation to exceed 3/16 of the within-level
     * spread. Judge the calibrated capture as a whole: the exported stream deliberately includes an ignored
     * lead copy, so allowing that unused physical period to veto an otherwise open eye only compounds misses
     * across a dump's 21 acquisitions. Corrupted used periods still fail the shared decoder's zero-error,
     * exact-duplicate requirement; decoded values never influence this analog confidence gate. */
    if (eye_quality < T55_EYE_MIN_QUALITY) return 0;

    /* Leave two fixed half-bits ahead of the exported anchor. The shared stream cap is 256 bytes and an
     * anchored stream spends two of them on [0, initial_level]; an alternating block therefore carries at
     * most 254 half-bits. Starting two cells later moves this frontend's calibrated boundary from phase 63
     * to phase 61, leaving one cell of framing margin while two complete repetitions still fit even for
     * 00000000/FFFFFFFF (one transition per cell). */
    lf_t55_measure_cells(start, sample);
    for (int h = 0; h < T55_EYE_CELLS; h++)
        s_t55_hb[s_t55_nhb++] = (uint8_t)(s_t55_eye[h] > threshold[h / T55_BLOCK_CELLS]);

    if (s_t55_nhb < 3 || cap < 3) return 0;
    int count = 0, run = 1;
    uint8_t level = s_t55_hb[0];
    buf[count++] = 0;
    buf[count++] = level;
    for (int i = 1; i < s_t55_nhb; i++) {
        if (s_t55_hb[i] == level) { run++; continue; }
        if (count >= cap) break;
        int d = run * T55_HB;
        buf[count++] = d > 0xFF ? 0xFF : (uint8_t)d;
        level = s_t55_hb[i]; run = 1;
    }
    if (count < cap) {
        int d = run * T55_HB;
        buf[count++] = d > 0xFF ? 0xFF : (uint8_t)d;
    }
    return count;
}

/* T5577 READ: send the read downlink (field left on), settle past the peak-detector, then demodulate the
 * tag's continuously-repeated block into level-run durations for the shared t55 decoder (t55_extract). */
int hal_rfid_lf_transceive(const uint8_t *cmd, int nbits, uint8_t *buf, int cap)
{
    if (s_mode != RFID_LF_READER) return RFID_ERR_UNSUPP;
    if (!cmd || nbits <= 0) return RFID_ERR_UNSUPP;
    if (!buf || cap <= 0) return RFID_ERR_UNSUPP;

    /* Direct access needs the same strong, idle-high gap waveform as programming. The final EasyDMA word
     * switches straight back to the low-duty, non-inverted carrier so the receive amplifier remains linear. */
    if (lf_t55_cmd(cmd, nbits, LF_DUTY_WRITE, LF_DUTY_RX, true, false) != 0)
        return RFID_ERR_TIMEOUT;
    vTaskDelay(pdMS_TO_TICKS(T55_REPLY_SETTLE_MS)); /* let the AC-coupled receive path recover */

    /* Prepare only after recovery so the SAADC input does not load the analog path during the downlink.
     * Twelve-bit conversion resolves the T5577's load-modulation swing; EM4100 acquisition remains 8-bit. */
    if (lf_sample_prepare(s_lf_env, LF_ENV_SAMPLES, SAADC_RESOLUTION_VAL_12bit) != 0) {
        lf_stop();
        vTaskDelay(pdMS_TO_TICKS(T55_READ_RESET_MS));
        return RFID_ERR_TIMEOUT;
    }
    NRF_TIMER3->EVENTS_COMPARE[2] = 0;
    NRF_PPI->TASKS_CHG[LF_PPI_GRP_ARM].EN = 1;      /* hardware opens ADC at count 2047 for phase-0 sample */

    int rc = lf_sample_finish(LF_ENV_SAMPLES);
    if (rc != 0) {
        lf_stop();
        vTaskDelay(pdMS_TO_TICKS(T55_READ_RESET_MS));
        return RFID_ERR_TIMEOUT;
    }
    int nr = lf_demod_t55(buf, cap);
    /* A short carrier interruption does not reset a T5577 reliably. Fully stop it between independently
     * addressed captures and let its reservoir discharge, matching the proven Proxmark path. This also makes
     * the dump module's documented inter-block reset real on the Chameleon; the next command recharges it. */
    lf_stop();
    vTaskDelay(pdMS_TO_TICKS(T55_READ_RESET_MS));
    taskYIELD();                                    /* let the transport drain before the module emits this block */
    return nr;
}
