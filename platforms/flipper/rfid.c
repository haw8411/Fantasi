/* Fantasi / Flipper Zero - RFID HAL (HF reader via ST25R3916 on SPI1).
 *
 * Implements the hal_rfid.h contract for the Flipper's HF *read* path: a discrete
 * ST25R3916 NFC frontend on SPI1 (the Flipper's "bus R", shared with the sub-GHz
 * CC1101 - we own it while reading). This is a minimal direct-register driver, not
 * the STM RFAL library. Modelled on the stock Flipper low-level driver
 * (lib/drivers/st25r3916*.c + targets/f7/furi_hal/furi_hal_nfc*.c) but against
 * bare STM32 SPI/GPIO registers, mirroring the CU's MFRC522 driver
 * (platforms/chameleon/rfid.c). The portable 14443A/CRC/EM4100 logic in
 * core/rfid/ and the app + host layers are unchanged - only this HAL differs.
 *
 * Pins (STM32WB55, Flipper "bus R"):
 *   SCK = PA5 (AF5)   MISO = PB4 (AF5)   MOSI = PB5 (AF5)
 *   CS  = PE4 (GPIO, software, active low)      IRQ = PA2 (we poll IRQ regs over SPI)
 * SPI mode 1 (CPOL=0, CPHA=1), MSB-first, 8-bit, ~4 MHz (SPI1 on APB2 /8).
 * LF (125 kHz) is a separate STM32 comparator/timer subsystem - a later phase.
 */
#include "stm32wbxx.h"
#include "../../hal/hal_rfid.h"
#include "../../core/vfs.h"
#include "../../core/log.h"   /* TEMP (port bring-up): for debugging */
#include "ble.h"
#include "power.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

/* ---- ST25R3916 SPI operation bytes (st25r3916_reg.h) ---- */
#define ST_WRITE_MODE   0x00      /* reg write: (addr & 0x3F) | this */
#define ST_READ_MODE    0x40      /* reg read                       */
#define ST_FIFO_LOAD    0x80      /* FIFO write opcode              */
#define ST_FIFO_READ    0x9F      /* FIFO read opcode               */
#define ST_SPACE_B      0x40      /* address marker: register is in bank B */
#define ST_CMD_SPACE_B  0xFB      /* one-shot prefix to reach bank B */

/* ---- Registers (bank A; |ST_SPACE_B => bank B) ---- */
#define REG_IO_CONF2       0x01
#define REG_OP_CONTROL     0x02
#define REG_MODE           0x03
#define REG_BIT_RATE       0x04
#define REG_ISO14443A_NFC  0x05
#define REG_AUX            0x0A
#define REG_RX_CONF1       0x0B
#define REG_RX_CONF2       0x0C
#define REG_RX_CONF3       0x0D
#define REG_RX_CONF4       0x0E
#define REG_CORR_CONF1       (ST_SPACE_B | 0x0C)
#define REG_CORR_CONF2       (ST_SPACE_B | 0x0D)
#define REG_OVERSHOOT_CONF1  (ST_SPACE_B | 0x30)
#define REG_OVERSHOOT_CONF2  (ST_SPACE_B | 0x31)
#define REG_UNDERSHOOT_CONF1 (ST_SPACE_B | 0x32)
#define REG_UNDERSHOOT_CONF2 (ST_SPACE_B | 0x33)
#define REG_IRQ_MASK_MAIN  0x16    /* 4 consecutive IRQ mask regs (0x16-0x19); 1 = masked/disabled */
#define REG_IRQ_MAIN       0x1A    /* 4 consecutive status regs; reading clears */
#define REG_FIFO_STATUS1   0x1E
#define REG_FIFO_STATUS2   0x1F
#define REG_NUM_TX_BYTES1  0x22    /* tx bit-count high byte */
#define REG_NUM_TX_BYTES2  0x23    /* tx bit-count low byte  */
#define REG_AD_RESULT      0x25    /* A/D converter output (MEASURE_VDD result) */
#define REG_REGULATOR_CTRL 0x2C    /* regulated-voltage control (supply measure + adjust) */
#define REG_AUX_DISPLAY    0x31
#define REG_IC_IDENTITY    0x3F
#define REG_FIELD_ON_GT    (ST_SPACE_B | 0x15)

/* Register bits */
#define OP_CONTROL_en        0x80
#define OP_CONTROL_rx_en     0x40
#define OP_CONTROL_tx_en     0x08
#define MODE_om_mask         0x78
#define MODE_om_iso14443a    0x08
#define MODE_om_subc_stream  0x70   /* om=1110: sub-carrier stream mode (Table 23 - INITIATOR mode; using it as a
                                     * target is undocumented but makes the chip generate the reply subcarrier from
                                     * its own carrier-locked clock -> reliable reply, no CPU-async-phase drift) */
#define MODE_tr_am           0x04
#define REG_STREAM           0x09   /* Stream mode definition: scf(6:5) subcarrier freq, scp(4:3) pulses/report, stx(2:0) tx period */
#define STREAM_scf_fc16      0x40   /* scf=10: fc/16 = 848 kHz (14443-A subcarrier) */
#define STREAM_scp_8         0x18   /* scp=11: 8 sub-carrier pulses per report period (= 1 etu @ fc/16) */
#define STREAM_scp_4         0x10   /* scp=10: 4 pulses per report (= half-etu) */
#define STREAM_stx_fc16      0x03   /* stx=011: TX modulator time period fc/16 */
#define ISO14443A_no_tx_par  0x80
#define ISO14443A_no_rx_par  0x40
#define AUX_no_crc_rx        0x80
#define AUXDISP_osc_ok       0x10
#define IO_CONF2_drv_lvl     0x04
#define IO_CONF2_sup3V       0x80    /* supply mode: 1 = 3V, 0 = 5V (set from MEASURE_VDD) */
#define REGCTRL_mpsv_mask    0x07    /* measure-power-supply source select */
#define REGCTRL_mpsv_vdd     0x00    /* measure VDD */
#define REGCTRL_reg_s        0x80    /* regulator manual/reset strobe */
#define ICID_mask            0xF8
#define ICID_st25r3916       0x28
#define FIFO2_b_mask         0xC0    /* byte-count bits 9:8 */
#define FIFO2_lb_mask        0x0E    /* # valid bits in the last byte */

/* ---- Direct commands (full opcode bytes) ---- */
#define CMD_SET_DEFAULT       0xC1
#define CMD_STOP              0xC2
#define CMD_TRANSMIT_NO_CRC   0xC5
#define CMD_TRANSMIT_REQA     0xC6
#define CMD_TRANSMIT_WUPA     0xC7
#define CMD_ADJUST_REGULATORS 0xD6
#define CMD_MEASURE_VDD       0xDF   /* measure supply voltage into AD_RESULT */
#define CMD_CLEAR_FIFO        0xDB
#define CMD_TRANSPARENT_MODE  0xDC   /* enter transparent mode: demod -> MISO, CS held high */
#define CMD_UNMASK_RECEIVE    0xD1   /* start the RX decoders/digitizer (un-gate the receiver) */
#define CMD_RESET_RX_GAIN     0xD5   /* load RX_CONF4 / manual gain into the AGC/digitizer block */
#define REG_EFD_THRESH        0x2A   /* trg_l[6:4] peer-detect threshold (external field detector) */
#define REG_EFD_THRESH_D      0x2B   /* deactivation threshold; mirror 0x2A to disable hysteresis */
#define REG_AUX_DISP          0x31   /* efd_o (bit 6) = external carrier above the active threshold */

/* ---- Card-emulation (passive target) registers, per stock furi_hal_nfc_init ---- */
#define REG_IO_CONF1          0x00
#define REG_TX_DRIVER         0x28
#define REG_ANT_TUNE_A        0x26
#define REG_ANT_TUNE_B        0x27
#define REG_PT_MOD            0x29
#define REG_FIELD_THR_ACTV    0x2A   /* External Field Detector Activation Threshold */
#define REG_FIELD_THR_DEACTV  0x2B   /* External Field Detector Deactivation Threshold */
#define REG_RES_AM_MOD        (ST_SPACE_B | 0x2A)
#define REG_AUX_MOD           (ST_SPACE_B | 0x28)   /* lm_ext|lm_dri: enable load modulation */
#define REG_EMD_SUP_CONF      (ST_SPACE_B | 0x05)

#define REG_PT_STATUS         0x21   /* Passive target status: pta_state<3:0> in bits 0-3 */

/* ---- Sniffer front-end (passive envelope demod; HydraNFC recipe) ---- */
#define REG_PASSIVE_TARGET    0x08
#define AUX_dis_corr          0x04   /* coherent/envelope demod, not the 848 kHz correlator */
#define OP_CONTROL_en_fd_auto 0x03   /* automatic external-field detector (en_fd_c=11) */
#define OP_CONTROL_en_fd_peer 0x02   /* manual peer-detection field detector (en_fd_c=10) */
#define IO_CONF2_miso_pd      0x18   /* miso_pd1|miso_pd2: pull-downs on MISO (clear for sniff) */
#define MODE_targ             0x80   /* target/listen (expect an external field, no self-field) */

/* ---- IRQ status bits (32-bit little-endian over the 0x1A burst) ---- */
#define IRQ_OSC   0x00000080u
#define IRQ_RXE   0x00000010u    /* receive complete */
#define IRQ_TXE   0x00000008u    /* transmit complete */
#define IRQ_COL   0x00000004u    /* bit collision */
#define IRQ_NRE   0x00004000u    /* no-response timeout */
#define IRQ_CRC   0x00800000u
#define IRQ_PAR   0x00400000u
#define IRQ_ERR1  0x00100000u    /* hard framing error */
#define IRQ_ERR2  0x00200000u    /* soft framing error */

/* ---- Pins ---- */
#define CS_PORT  GPIOE
#define CS_PIN   4               /* PE4 */

static bool        s_spi_up;
static rfid_mode_t s_mode = RFID_OFF;
static volatile int s_hf_dirty;      /* a sniff reconfigured the HF front-end (demod/transparent mode) with
                                       * s_mode still HF_READER; set for the whole sniff and cleared only by a
                                       * real reader re-init, so the next set_mode(HF_READER) can't take its
                                       * idempotency shortcut and leave the reader running the sniff config
                                       * (deaf). Survives a ^C abort that skips the sniff's own cleanup. */

/* ---- Raw SPI1 - mode 1, ~4 MHz, MSB-first, software CS ---- */
static void cs_low(void)  { CS_PORT->BSRR = (1u << (CS_PIN + 16)); }
static void cs_high(void) { CS_PORT->BSRR = (1u << CS_PIN); }

static uint8_t spi_xfer(uint8_t v)
{
    while (!(SPI1->SR & SPI_SR_TXE)) { }
    *(volatile uint8_t *)&SPI1->DR = v;
    while (!(SPI1->SR & SPI_SR_RXNE)) { }
    return *(volatile uint8_t *)&SPI1->DR;
}

static void spi_init(void)
{
    if (s_spi_up) return;

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOEEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    (void)RCC->APB2ENR;

    /* PA2 is shared by ST25R3916 IRQ and the LF RFID_PULL control. Release the
     * LF output before the ST25 enables its push-pull IRQ driver. */
    GPIOA->MODER &= ~GPIO_MODER_MODE2;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD2;

    /* PA5 = SCK (AF5, very-high speed) */
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE5) | (2u << GPIO_MODER_MODE5_Pos);
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFu << (5 * 4))) | (5u << (5 * 4));
    GPIOA->OSPEEDR |= (3u << GPIO_OSPEEDR_OSPEED5_Pos);
    /* PB4 = MISO, PB5 = MOSI (AF5) */
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5))
                 | (2u << GPIO_MODER_MODE4_Pos) | (2u << GPIO_MODER_MODE5_Pos);
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~((0xFu << (4 * 4)) | (0xFu << (5 * 4))))
                  | (5u << (4 * 4)) | (5u << (5 * 4));
    GPIOB->OSPEEDR |= (3u << GPIO_OSPEEDR_OSPEED4_Pos) | (3u << GPIO_OSPEEDR_OSPEED5_Pos);
    /* PE4 = CS (output push-pull, idle HIGH) */
    CS_PORT->BSRR = (1u << CS_PIN);
    CS_PORT->MODER = (CS_PORT->MODER & ~GPIO_MODER_MODE4) | (1u << GPIO_MODER_MODE4_Pos);

    /* SPI1: master, mode 1 (CPOL=0 CPHA=1), MSB-first, 8-bit, /8, software NSS. */
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI
              | SPI_CR1_CPHA                        /* CPHA=1 => SPI mode 1 */
              | (1u << SPI_CR1_BR_Pos);             /* PCLK2 / 4 = 8 MHz: fast enough to issue the target
                                                    * TRANSMIT inside the ISO14443-A frame-delay window */
    SPI1->CR2 = SPI_CR2_FRXTH | (7u << SPI_CR2_DS_Pos);   /* 8-bit; RXNE at 8 bits */
    SPI1->CR1 |= SPI_CR1_SPE;
    s_spi_up = true;
}

static void spi_deinit(void)
{
    if (!s_spi_up) return;
    SPI1->CR1 &= ~SPI_CR1_SPE;
    s_spi_up = false;
}

/* ---- ST25R3916 com layer (bank-B aware) ---- */
static void reg_read_burst(uint8_t addr, uint8_t *buf, int n)
{
    cs_low();
    if (addr & ST_SPACE_B) spi_xfer(ST_CMD_SPACE_B);
    spi_xfer((uint8_t)((addr & 0x3F) | ST_READ_MODE));
    for (int i = 0; i < n; i++) buf[i] = spi_xfer(0xFF);
    cs_high();
}
static uint8_t reg_read(uint8_t addr) { uint8_t v; reg_read_burst(addr, &v, 1); return v; }

static void reg_write(uint8_t addr, uint8_t val)
{
    cs_low();
    if (addr & ST_SPACE_B) spi_xfer(ST_CMD_SPACE_B);
    spi_xfer((uint8_t)((addr & 0x3F) | ST_WRITE_MODE));
    spi_xfer(val);
    cs_high();
}
static void reg_set(uint8_t a, uint8_t m)   { reg_write(a, (uint8_t)(reg_read(a) | m)); }
static void reg_clear(uint8_t a, uint8_t m) { reg_write(a, (uint8_t)(reg_read(a) & ~m)); }

static void direct_cmd(uint8_t cmd) { cs_low(); spi_xfer(cmd); cs_high(); }

/* Read + clear the 4-byte IRQ status burst at 0x1A into a 32-bit word. */
static uint32_t get_irq(void)
{
    uint8_t b[4];
    reg_read_burst(REG_IRQ_MAIN, b, 4);
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

static void mask_all_irqs(void)
{
    for (int i = 0; i < 4; i++) reg_write((uint8_t)(REG_IRQ_MASK_MAIN + i), 0xFF);
    (void)get_irq();
}

/* Bring the ST25R3916 up as an ISO14443-A poller. Returns 0, or -1 if no chip. */
static int st25_init(void)
{
    direct_cmd(CMD_STOP);                         /* exit any lingering transparent mode first */
    direct_cmd(CMD_SET_DEFAULT);
    vTaskDelay(pdMS_TO_TICKS(1));
    reg_set(REG_IO_CONF2, IO_CONF2_drv_lvl);      /* boost driver strength */

    uint8_t id = reg_read(REG_IC_IDENTITY);
    if ((id & ICID_mask) != ICID_st25r3916) return -1;

    /* Oscillator + regulators. */
    reg_set(REG_OP_CONTROL, OP_CONTROL_en);
    for (int i = 0; i < 50; i++) {                /* wait for the crystal to settle */
        if (reg_read(REG_AUX_DISPLAY) & AUXDISP_osc_ok) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    direct_cmd(CMD_ADJUST_REGULATORS);
    vTaskDelay(pdMS_TO_TICKS(6));

    /* ISO14443-A initiator (poller), OOK modulation. */
    reg_clear(REG_MODE, MODE_om_mask | MODE_tr_am);
    reg_set(REG_MODE, MODE_om_iso14443a);

    /* Analog RX chain + overshoot/undershoot protection for 106 kbps NFC-A,
     * matching the stock furi_hal_nfc_iso14443a_common_init / _poller_init. The
     * CORR/OVERSHOOT/UNDERSHOOT registers live in bank B (reg_write prefixes 0xFB
     * for the ST_SPACE_B marker automatically). */
    reg_write(REG_OVERSHOOT_CONF1,  0x40);
    reg_write(REG_OVERSHOOT_CONF2,  0x03);
    reg_write(REG_UNDERSHOOT_CONF1, 0x40);
    reg_write(REG_UNDERSHOOT_CONF2, 0x03);
    reg_write(REG_RX_CONF1, 0x08);
    reg_write(REG_RX_CONF2, 0x2D);
    reg_write(REG_RX_CONF3, 0x00);
    reg_write(REG_RX_CONF4, 0x00);
    reg_write(REG_CORR_CONF1, 0x51);
    reg_write(REG_CORR_CONF2, 0x00);
    reg_write(REG_BIT_RATE,   0x00);   /* 106 kbps (reset default; set for parity) */
    return 0;
}

/* ================= LF 125 kHz reader (TIM1 carrier + COMP1 -> TIM2 capture) =========
 * The ST25R3916 is HF-only, so LF is a wholly separate STM32 subsystem, exactly as
 * on the stock Flipper (targets/f7/furi_hal/furi_hal_rfid.c): a 125 kHz carrier is
 * driven onto the reader coil by TIM1_CH1N (PB13); the tag's load modulation comes
 * back demodulated on PC5, which feeds analog comparator COMP1 (thresholded at
 * 1/2 VREFINT); COMP1's digital output is routed *internally* onto TIM2's TI4 input
 * (no pin), and a TIM2 input-capture timestamps each rising edge at 1 us. The
 * inter-edge intervals (÷8 -> 125 kHz carrier cycles) are what core/rfid/rfid_em4100
 * decodes. Timers run at 32 MHz (this firmware is HSE/32 MHz, not the stock 64 MHz).
 * Pins: PB13 carrier (AF1 TIM1_CH1N), PC5 COMP1 in (analog), PA2 RFID_PULL (low),
 * PB14 iButton (parked low - shares the coil node). */
#define LF_CARRIER_ARR   255   /* 32 MHz / (255+1) = 125 kHz */
#define LF_CARRIER_CCR   127   /* ~50% duty */
#define LF_TIM2_PSC      31    /* 32 MHz / 32 = 1 MHz -> 1 us per tick */
#define LF_US_PER_CYCLE  8     /* one 125 kHz period = 8 us; interval_us/8 = carrier cycles */

/* COMP1 CSR: medium power, INM = 1/2 VREFINT (INMSEL=001 + scaler bridge), INP = IO1
 * (PC5, INPSEL=00), hysteresis HIGH for the noisy envelope; EN added last. */
#define LF_COMP_CSR (COMP_CSR_PWRMODE_0 | COMP_CSR_INMSEL_0 | COMP_CSR_SCALEN | \
                     COMP_CSR_BRGEN | COMP_CSR_HYST)

#define LF_CAP  700
static volatile uint16_t *s_lf_iv;          /* raw inter-rising-edge intervals, us - heap scratch, live only during an acquire */
static volatile int      s_lf_n;
static volatile uint32_t s_lf_prev;
static volatile bool     s_lf_have_prev;
static volatile bool     s_lf_capturing;
static bool              s_lf_warm;         /* LF RX front-end has been settled since the last (re)init */

/* TIM2 CC4: each enabled COMP1 edge is captured (free-running 1 MHz counter); the
 * delta from the previous capture is the inter-edge interval. Reading CCR4 clears
 * the flag. No FreeRTOS calls here - just fills a buffer. */
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_CC4IF) {
        uint32_t cur = TIM2->CCR4;                 /* clears CC4IF */
        if (s_lf_capturing) {
            if (s_lf_have_prev && s_lf_n < LF_CAP) {
                uint32_t d = cur - s_lf_prev;
                s_lf_iv[s_lf_n++] = (d > 0xFFFF) ? 0xFFFF : (uint16_t)d;
            }
            s_lf_prev = cur;
            s_lf_have_prev = true;
        }
    }
}

static void lf_hw_init(void)
{
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
    RCC->APB2ENR  |= RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    (void)RCC->APB1ENR1;

    /* PA2 = RFID_PULL: output push-pull, driven LOW for the whole read. */
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE2) | (1u << GPIO_MODER_MODE2_Pos);
    GPIOA->BSRR = (1u << (2 + 16));
    /* PB13 = TIM1_CH1N carrier (AF1); PB14 = iButton parked (output low). */
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE13 | GPIO_MODER_MODE14))
                 | (2u << GPIO_MODER_MODE13_Pos) | (1u << GPIO_MODER_MODE14_Pos);
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(0xFu << ((13 - 8) * 4))) | (1u << ((13 - 8) * 4));
    GPIOB->BSRR = (1u << (14 + 16));
    /* PC5 = COMP1 input: analog. */
    GPIOC->MODER |= (3u << GPIO_MODER_MODE5_Pos);

    /* TIM1: 125 kHz PWM on CH1N. Advanced timer -> needs MOE (set in lf_field). */
    TIM1->CR1   = TIM_CR1_ARPE;
    TIM1->PSC   = 0;
    TIM1->ARR   = LF_CARRIER_ARR;
    TIM1->CCR1  = LF_CARRIER_CCR;
    TIM1->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;   /* PWM1 + preload */
    TIM1->CCER  = TIM_CCER_CC1NE;                                          /* enable CH1N */
    TIM1->BDTR  = TIM_BDTR_OSSI;   /* MOE=0 -> CH1N driven to idle (OIS1N=0, low), not hi-Z: damps a downlink gap */
    TIM1->EGR   = TIM_EGR_UG;                                              /* load registers */

    /* COMP1: 1/2 VREFINT threshold on PC5; enable + startup settle. */
    COMP1->CSR = LF_COMP_CSR;
    COMP1->CSR = LF_COMP_CSR | COMP_CSR_EN;
    vTaskDelay(pdMS_TO_TICKS(1));

    /* TIM2: free-running 1 us counter; CH4 input-capture on TI4 (= COMP1 out via
     * the internal remap), rising edge. Counter runs continuously; the ISR is
     * gated on/off per acquire via CC4IE. */
    TIM2->CR1  = 0;
    TIM2->PSC  = LF_TIM2_PSC;
    TIM2->ARR  = 0xFFFFFFFFu;
    TIM2->OR   = TIM2_OR_TI4_RMP_0;                 /* TI4 <- COMP1 output */
    TIM2->CCMR2 = TIM_CCMR2_CC4S_0;                 /* CC4 = input, IC4 on TI4 */
    TIM2->CCER  = TIM_CCER_CC4E;                    /* capture enable, rising edge */
    TIM2->EGR   = TIM_EGR_UG;
    TIM2->SR    = 0;
    TIM2->CR1  |= TIM_CR1_CEN;
    NVIC_SetPriority(TIM2_IRQn, 5);
    NVIC_EnableIRQ(TIM2_IRQn);
}

static void lf_stop(void)
{
    s_lf_warm = false;                             /* next set_mode(LF) re-inits the RX, so it needs the settle
                                                    * again - the module set_mode(OFF)s after every command */
    TIM2->DIER &= ~TIM_DIER_CC4IE;
    NVIC_DisableIRQ(TIM2_IRQn);
    TIM2->CR1  &= ~TIM_CR1_CEN;
    TIM1->CR1  &= ~TIM_CR1_CEN;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    COMP1->CSR &= ~COMP_CSR_EN;
    GPIOA->BSRR = (1u << (2 + 16));                 /* keep RFID_PULL LOW (its HIGH<->LOW toggle disturbs the
                                                    * RX; parking it low avoids the ~130 ms recovery hit) */
}

/* ================= HF sniffer (passive ISO14443-A, ST25R3916 transparent mode) =======
 * The Flipper wires only SPI + IRQ to the ST25R3916 - none of the HydraNFC's sniff pins
 * (MCU_CLK / EXT_LM / CSI / CSO). So we exploit *transparent mode* (cmd 0xDC): it bypasses
 * the FIFO/framing and drives the receiver's DIGITIZED envelope-demod output straight onto
 * the MISO pin (our PB4). Transparent mode is held while CS/BSS stays HIGH, so the SPI
 * peripheral is parked during capture and we read PB4 as a plain GPIO. rx_en=1 / tx_en=0
 * runs the demodulator on the EXTERNAL reader's field with our own carrier off (passive);
 * reusing st25_init's known-good ISO14443-A RX chain so its digitizer produces the demod on
 * MISO. We oversample PB4 with our own timebase (no field-locked MCU_CLK needed) via a
 * TIM2->DMA one-shot at 4 MHz - the DMA runs autonomously (no CPU, no interrupts-off) - then
 * decode both directions offline: the reader's 100% ASK Miller pauses and the card's 847 kHz
 * load-modulation subcarrier. */
#define MISO_PIN     4          /* PB4 */
/* Continuous streaming capture: a circular DMA ring the CPU decodes behind, per-frame (cut only at
 * edge-free between-frame gaps), so a transaction of any length streams without truncation and WUPA
 * is never missed to a trigger. Overrun handled Proxmark-style: if we fall behind, jump the read
 * pointer up to the write pointer and drop the in-flight frame rather than corrupt the stream. */
#define RING_SAMPLES  65500u    /* 25 ms circular ring @2.67 MHz - never stops. During a dense burst the
                                 * decoder falls behind (subcarrier defeats the word fast-skip) and catches
                                 * up in the quiet after, so the ring must hold a whole transaction's
                                 * backlog. Sized just under the DMA's 16-bit CNDTR ceiling (65535): a full
                                 * mfu dump (11 back-to-back reads) plus repeated failed-auth anticollision
                                 * peaks ~59000 samples, kept under the 90% overrun line (~58950). The 80 KB
                                 * this + decode scratch needs only fits once the app loader frees the driver's
                                 * /ramfs ELF post-load (core/app_run.c) - without that it OOMs on this heap. */
#define RING_SAMPLES_MIN 32768u  /* low-memory fallback ring; if even this won't fit we report OOM */
#define RING_MARGIN   1500u     /* keep the read ptr this far behind the DMA write ptr */
#define PROCESS_MIN   4000u     /* don't process until this many samples are queued - keeps rounds large
                                 * (low per-round overhead) instead of spinning ~100-sample rounds */
#define MAX_SCAN      8000u     /* max new ring samples scanned per round before we yield */
#define QUIET_MS      250u      /* end the session only after the field has been quiet this long - a
                                 * reader's transaction can span multiple bursts (e.g. a host round-trip
                                 * between select and a follow-up command), so a short gap must not cut it */
#define FRAME_GAP     81        /* >= 3.5 etu (81 samp @2.46 MHz) edge-free = a between-frame gap = cut */
#define CARD_MIN      1539      /* subcarrier samples marking a real card reply (gain check, phase 3) */
#define CAP_ARR      12u        /* TIM2 update = 32 MHz / (12+1) = 2.462 MHz. A long back-to-back read
                                 * burst (mfu dump, ~27 ms) delivers ~17% fewer samples than at 3.2 MHz,
                                 * so the scan+decode backlog per burst is ~half - the difference between
                                 * dropping the tail reads and capturing them - while 2.67 MHz still >3x
                                 * the 847 kHz subcarrier so it resolves cleanly. All sample-unit decode
                                 * constants below are scaled to match (x0.833 of the 3.2 MHz values). */

static void        miso_as_input(void){ GPIOB->MODER &= ~GPIO_MODER_MODE4; }
static void        miso_as_spi(void)  { GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE4)
                                                     | (2u << GPIO_MODER_MODE4_Pos); }

/* Park the ST25R3916 as a silent passive demodulator of an external field. Reuse the
 * known-good ISO14443-A RX chain st25_init already programmed (the one that decodes card
 * subcarriers when we read tags) - just run the receiver with the transmitter off so it
 * demodulates the external reader's field, and un-gate the receiver so its digitizer runs. */
/* AM-mixer coherent demod with a runtime first-stage gain. rg1_am gates the RF front end:
 * boost catches a weak far field but saturates a strong near one, and no fixed value spans
 * the range (the chip's AGC is on the wrong stage) - so hf_sniff picks rg1 from the capture itself. */
#define SNIFF_RG_MID   0x78   /* rg1_am = 3 (moderate): clean for a near/strong field */
#define SNIFF_RG_BOOST 0xF8   /* rg1_am = 7 (+5.5 dB boost): recovers a far/weak field, clips a near one */

static void sniff_config(uint8_t rg1)
{
    direct_cmd(CMD_STOP);
    reg_clear(REG_OP_CONTROL, OP_CONTROL_tx_en);          /* no field of our own */
    reg_set(REG_OP_CONTROL, OP_CONTROL_en | OP_CONTROL_rx_en | OP_CONTROL_en_fd_auto);
    /* Wideband: 1200 kHz low-pass keeps the 847 kHz card subcarrier; 12 kHz high-pass (vs the
     * 600 kHz reader default) keeps the reader's ~150-300 kHz baseband Miller pauses - so one
     * MISO stream carries both directions. Coherent AM mixer demod. */
    reg_write(REG_RX_CONF1, 0x03);                       /* wideband AM channel (ch_sel=0) */
    reg_write(REG_RX_CONF2, 0x40);                       /* amd_sel=1: mixer (coherent) demod, AGC off */
    reg_write(REG_RX_CONF3, rg1);                        /* adaptive first-stage gain */
    reg_write(REG_RX_CONF4, 0x00);                       /* smallest digitizer window (most sensitive) */
    reg_write(REG_AUX, AUX_dis_corr);                    /* coherent/envelope demod, not the 847kHz correlator */
    reg_clear(REG_IO_CONF2, IO_CONF2_miso_pd);            /* don't load the transparent-mode MISO output */
    direct_cmd(CMD_RESET_RX_GAIN);                       /* mandatory: load RX_CONF4/manual gain */
    direct_cmd(CMD_CLEAR_FIFO);
    direct_cmd(CMD_UNMASK_RECEIVE);                       /* un-gate the receiver so the digitizer runs */
}

/* ---- External Field Detector: field strength for adaptive gain + coupling colour (phase 3) ---- */
#define EFD_O  0x40   /* REG_AUX_DISP bit 6: external carrier is above the active peer threshold */

/* Set the peer-detect threshold (trg_l 0..7) and MIRROR the deactivation threshold to it, killing
 * hysteresis so efd_o tracks the instantaneous carrier against exactly this level. */
static void efd_set_thresh(int th)
{
    uint8_t v = (uint8_t)((th & 7) << 4);
    reg_write(REG_EFD_THRESH,   (uint8_t)((reg_read(REG_EFD_THRESH)   & ~0x70) | v));
    reg_write(REG_EFD_THRESH_D, (uint8_t)((reg_read(REG_EFD_THRESH_D) & ~0x70) | v));
}

/* Wait (up to timeout_ms) for an external reader field, then measure its strength 0..7. The EFD is a
 * comparator on RFI, independent of the demod/gain/AGC - so it reads field strength where RSSI is dead
 * in this dis_corr config and Measure-Amplitude (0xD3) is useless (it turns TX on). Sweeps the trigger
 * level high->low; the highest threshold the carrier still exceeds is the strength. Peak-holds a few
 * samples per threshold so a 100% ASK pause (carrier momentarily 0) doesn't read as no-field. Returns
 * 0..7, or -1 if no field arrived. Must run in normal (non-transparent) mode with SPI available. */
static int efd_wait_measure(uint32_t timeout_ms)
{
    reg_clear(REG_OP_CONTROL, 0x03);
    reg_set(REG_OP_CONTROL, OP_CONTROL_en_fd_peer);       /* en_fd_c = 10 manual peer-detect */
    efd_set_thresh(0);
    TickType_t t0 = xTaskGetTickCount();
    /* Wait for any lingering field to drop first, then for a fresh field-on - so each call catches a
     * new transaction instead of re-capturing the same one still radiating. */
    while (reg_read(REG_AUX_DISP) & EFD_O) {
        if ((xTaskGetTickCount() - t0) >= pdMS_TO_TICKS(timeout_ms)) return -1;
        vTaskDelay(2);
    }
    while (!(reg_read(REG_AUX_DISP) & EFD_O)) {
        if ((xTaskGetTickCount() - t0) >= pdMS_TO_TICKS(timeout_ms)) return -1;
        vTaskDelay(1);
    }
    /* Field on: sweep every threshold, the highest the carrier still exceeds is the strength. Kept fast
     * (~100 us total) - it runs inside the reader's power-up guard time, so capture can arm before the
     * first short frame (WUPA/REQA, ~5 ms in). One retry per threshold peak-holds across a brief pause. */
    int level = 0;
    for (int th = 0; th <= 7; th++) {
        efd_set_thresh(th);
        for (volatile int d = 0; d < 300; d++) { }         /* let the comparator settle */
        int on = (reg_read(REG_AUX_DISP) & EFD_O) ? 1 : 0;
        if (!on) { for (volatile int d = 0; d < 900; d++) { } on = (reg_read(REG_AUX_DISP) & EFD_O) ? 1 : 0; }
        if (on) level = th;
    }
    return level;
}

/* TIM2->DMA capture of PB4 (byte reads of GPIOB->IDR) at 4 MHz (RM0434: TIM2_UP DMAMUX request = 32
 * -> DMA1_Channel1; CPAR = &GPIOB->IDR byte, PB4 in the low byte). The DMA runs autonomously. */
static void dma_cap_go(void)   { TIM2->CNT = 0; TIM2->CR1 |= TIM_CR1_CEN; }
static void dma_cap_stop(void) { TIM2->CR1 &= ~TIM_CR1_CEN; TIM2->DIER = 0; DMA1_Channel1->CCR = 0; }

/* Capture source, so the same DMA + scan_span serve the transparent-mode demod on PB4/MISO for sniffing
 * and emulation. Each user sets these before arming. */
static volatile uint32_t *s_cap_idr   = &GPIOB->IDR;   /* low byte holds the capture pin */
static uint8_t            s_cap_shift  = MISO_PIN;      /* bit of the captured byte: PB4/MISO */
static uint32_t           s_cap_arr    = CAP_ARR;       /* TIM2 reload: 12 @32MHz (sniff) -> 2.462 MHz; the emu
                                                         * clock-boost raises it to 25 @64MHz to keep 2.462 MHz */

/* Circular arm: the DMA fills buf[n] and wraps forever. Write offset at any instant is n - CNDTR. */
static void dma_circ_arm(uint8_t *buf, uint32_t n)
{
    RCC->AHB1ENR  |= (1u << 0) | (1u << 2);
    RCC->APB1ENR1 |= (1u << 0);
    (void)RCC->AHB1ENR;

    DMA1_Channel1->CCR   = 0;
    DMA1_Channel1->CPAR  = (uint32_t)(uintptr_t)s_cap_idr;
    DMA1_Channel1->CMAR  = (uint32_t)(uintptr_t)buf;
    DMA1_Channel1->CNDTR = n;
    DMA1_Channel1->CCR   = (2u << 12) | (1u << 7) | (1u << 5) | (1u << 0);  /* PL hi, MINC, CIRC, EN */

    DMAMUX1_Channel0->CCR = 32;

    TIM2->CR1  = 0;
    TIM2->PSC  = 0;
    TIM2->ARR  = s_cap_arr;                             /* 12 @32MHz sniff / 25 @64MHz emu -> 2.462 MHz either way */
    TIM2->EGR  = TIM_EGR_UG;
    TIM2->SR   = 0;
    TIM2->DIER = TIM_DIER_UDE;
}

/* ---- Single-pass 14443-A decode of the captured demod-envelope stream ----
 * The wideband AM-envelope digitizer puts both directions on one MISO stream (§sniff_config):
 * the reader's 100% ASK Miller pauses appear as wide (~2-3 us) low pulses, the card's 847 kHz
 * subcarrier as narrow (~0.5 us) dense toggling. We separate by pulse width, then run a Miller
 * decoder (reader) and a Manchester decoder (card) - the same algorithms the Proxmark uses.
 * 2.462 MHz sampling:
 * 1 etu (9.44 us) = 23.24 samples (fixed-point x100 = 2324). */
#define ETU100   2324
#define ETUSAMP  23

static uint16_t *s_tr;        /* transition (edge) sample positions - heap scratch, live only during a sniff */
static uint8_t  *s_seq;       /* Miller per-etu symbol scratch (miller_decode) - heap, live only during a sniff */
static int      s_ntr;
static int      s_ongrid, s_offgrid;   /* reader pauses that fit / miss the etu grid (reader-dir confidence) */
static int      s_card_samp;           /* total samples of detected card 847 kHz subcarrier - PHYSICAL, so it
                                        * gauges the gain without reading any (possibly encrypted) data byte */
static int      s_tr_init;             /* level of the samples before s_tr[0] (reader run polarity) */

/* Persistent state for the incremental single-pass edge extractor (scan_span). s_tr positions are
 * relative to the start of the frame currently being accumulated; dead air before the first edge is
 * dropped so the positions stay small. The capture loop finds between-frame gaps (edge spacing >=
 * FRAME_GAP) over the edge list afterwards - far cheaper than tracking them per sample. */
static int      s_lvl;                  /* current level, after the last processed sample */

/* Miller-decode a reader frame (pause centers ps[0..np]) into bytes; returns byte count.
 * Modified-Miller pauses sit at 0.25 etu (Z, logic 0) or 0.75 etu (X, logic 1) of their bit;
 * a Y bit (0-after-1) has none. The first pause is the SOC (a Z). Because our 4 MHz clock is
 * async to the 13.56 MHz carrier, a fixed grid drifts over a long frame - so we anchor on the
 * SOC and re-sync the etu boundary on every pause, bounding error to one inter-pause gap.
 * Fixed-point x100 samples @2.667 MHz: ETU100=2517, quarter=629, three-quarter=1888, half=1259. */
__attribute__((optimize("O2")))
static int miller_decode(const uint16_t *ps, int np, uint8_t *out, uint32_t *par)
{
    *par = 0;
    uint8_t *seq = s_seq;                          /* per-etu: 0=Y(none), 1=Z(0), 2=X(1); heap scratch (alloc'd in hf_sniff) */
    __builtin_memset(seq, 0, 160);                 /* faster than a byte loop on the FDT reply path */
    long base = (long)ps[0]*100 - ETU100/4;        /* etu-0 boundary: SOC pause is 0.25 etu in */
    int maxi = 0;
    for (int i = 0; i < np; i++) {
        long rel = (long)ps[i]*100 - base; if (rel < 0) rel = 0;
        int idx = (int)(rel / ETU100);
        long within = rel - (long)idx*ETU100;      /* offset within the etu */
        long dZ = within - ETU100/4, dX = within - 3*ETU100/4;   /* distance to Z(0.25) / X(0.75) slot */
        if (dZ < 0) dZ = -dZ;
        if (dX < 0) dX = -dX;
        int half = (dX < dZ) ? 1 : 0;
        if ((half ? dX : dZ) > ETU100*16/100) { s_offgrid++; continue; }   /* off-grid (>0.16 etu): jitter/clip */
        s_ongrid++;
        if (idx >= 0 && idx < 160) { if (seq[idx] == 0) seq[idx] = half ? 2 : 1; if (idx > maxi) maxi = idx; }
        base = (long)ps[i]*100 - (long)idx*ETU100 - (half ? 3*ETU100/4 : ETU100/4);   /* re-sync on trusted pauses */
    }
    int sr = 0, nb = 0, n = 0;
    for (int k = 1; k <= maxi && n < 32; k++) {    /* skip SOC (k=0) */
        int bit = (seq[k] == 2) ? 1 : 0;
        sr = (sr >> 1) | (bit << 8); nb++;
        if (nb == 9) {                             /* full byte: bits 0-7 data, bit 8 the parity bit */
            int v = sr & 0xFF;
            if ((sr >> 8 & 1) == __builtin_parity(v)) *par |= (1u << n);   /* == parity(v) = wrong (odd) */
            out[n++] = (uint8_t)v; sr = 0; nb = 0;
        }
    }
    /* A 7-bit short frame (REQA 26h / WUPA 52h) never fills a 9-bit byte group; emit the bits as one
     * byte (a trailing 0/Y bit just isn't on the grid, so nb is 6-7). Only when no full byte formed, so
     * a truncated standard frame isn't mistaken for a short one. */
    if (n == 0 && nb >= 6 && nb <= 7)
        out[n++] = (uint8_t)((sr >> (9 - nb)) & ((1 << nb) - 1));
    return n;
}

/* ---- Incremental (pipelined) Miller decoder for the FDT-critical emu reply path ----
 * miller_decode does all its per-pause etu-grid placement (loop 1, ~2400 cyc for a 2-byte frame) after the
 * frame ends, which pushes a multi-byte reply's turnaround past the 87us anticollision FDT window. Here that
 * loop-1 work is done incrementally as each pause arrives during reception (miller_inc_feed, called after the
 * active-window scans), so at frame-end only the cheap byte-grouping (miller_inc_finish, ~200 cyc) remains.
 * Same math as miller_decode - anchor etu 0 on the SOC pause, re-sync the boundary on every trusted pause,
 * Z=0.25 etu (logic 0), X=0.75 etu (logic 1). s_seq is the shared per-etu symbol grid. */
static int   s_mp_proc;      /* next s_tr interval index still to fold into the grid */
static long  s_mp_base;      /* current etu-0 boundary (x100 samples), from the last trusted pause */
static int   s_mp_has;       /* base anchored (SOC pause seen) */
static int   s_mp_maxi;      /* highest etu slot filled so far */
static int   s_mp_prev;      /* previous pause centre (sample), for the FRAME_GAP new-frame trim */
static int   s_mp_np;        /* pauses folded in the current frame */
/* Incremental byte-grouping (loop-2), also pipelined: group filled grid slots into bytes DURING reception so
 * a long command's frame-end decode is cheap. Slots are final once set (first pause wins), so grouping up to
 * maxi-2 (a margin for pause re-sync) is safe; finish groups the rest. */
static int   s_grp_k, s_grp_sr, s_grp_nb, s_grp_n; static uint32_t s_grp_par; static uint8_t s_grp_out[32];
static void (*s_grp_byte_ready)(void *ctx, int index, uint8_t raw, int bits);
static void *s_grp_byte_ctx;
static int s_grp_callback_active;
static int s_grp_prefix_ready;
static void emu_early_stream_cancel(void);

static void miller_inc_reset(void)
{
    emu_early_stream_cancel();
    __builtin_memset(s_seq, 0, 160);
    s_mp_proc = 1; s_mp_has = 0; s_mp_maxi = 0; s_mp_prev = 0; s_mp_np = 0;
    s_grp_k = 1; s_grp_sr = 0; s_grp_nb = 0; s_grp_n = 0; s_grp_par = 0;
    s_grp_prefix_ready = 0;
}

/* Group grid slots s_grp_k..upto into 9-bit (8 data + parity) bytes, appending to s_grp_out. */
__attribute__((optimize("O2")))
static void miller_inc_group(int upto)
{
    while (s_grp_k <= upto && s_grp_n < 32) {
        int bit = (s_seq[s_grp_k] == 2) ? 1 : 0;
        s_grp_sr = (s_grp_sr >> 1) | (bit << 8); s_grp_nb++;
        if (s_grp_n == 0 && s_grp_nb == 7 && s_grp_byte_ready && !s_grp_prefix_ready) {
            s_grp_prefix_ready = 1;
            s_grp_callback_active = 1;
            s_grp_byte_ready(s_grp_byte_ctx, s_grp_n, (uint8_t)(s_grp_sr >> 2), 7);
            s_grp_callback_active = 0;
        }
        if (s_grp_nb == 8 && s_grp_byte_ready) {
            uint8_t raw = (uint8_t)(s_grp_sr >> 1);
            if (s_grp_n == 0 && s_grp_prefix_ready) emu_early_stream_cancel();
            s_grp_callback_active = 1;
            s_grp_byte_ready(s_grp_byte_ctx, s_grp_n, raw, 8);
            s_grp_callback_active = 0;
        }
        if (s_grp_nb == 9) {
            int v = s_grp_sr & 0xFF;
            if ((s_grp_sr >> 8 & 1) == __builtin_parity(v)) s_grp_par |= (1u << s_grp_n);
            s_grp_out[s_grp_n++] = (uint8_t)v; s_grp_sr = 0; s_grp_nb = 0;
            s_grp_prefix_ready = 0;
        }
        s_grp_k++;
    }
}

/* Place one pause centre `c` (sample pos) onto the etu grid (loop-1 body of miller_decode). */
__attribute__((optimize("O2")))
static inline void miller_inc_place(int c)
{
    if (!s_mp_has) { s_mp_base = (long)c * 100 - ETU100 / 4; s_mp_has = 1; }   /* SOC anchors etu 0 */
    long rel = (long)c * 100 - s_mp_base; if (rel < 0) rel = 0;
    int idx = (int)(rel / ETU100);
    long within = rel - (long)idx * ETU100;
    long dZ = within - ETU100 / 4, dX = within - 3 * ETU100 / 4;
    if (dZ < 0) dZ = -dZ;
    if (dX < 0) dX = -dX;
    int half = (dX < dZ) ? 1 : 0;
    if ((half ? dX : dZ) > ETU100 * 16 / 100) return;                          /* off-grid: jitter/clip */
    if (idx >= 0 && idx < 160) { if (s_seq[idx] == 0) s_seq[idx] = half ? 2 : 1; if (idx > s_mp_maxi) s_mp_maxi = idx; }
    s_mp_base = (long)c * 100 - (long)idx * ETU100 - (half ? 3 * ETU100 / 4 : ETU100 / 4);   /* re-sync */
}

/* Fold any newly-recorded s_tr intervals into the grid: a wide LOW run at reader polarity is a pause.
 * A pause >= FRAME_GAP after the previous one starts a new frame, so the grid resets to it. */
__attribute__((optimize("O2")))
static void miller_inc_feed(void)
{
    int init = s_tr_init; (void)init;
    for (int i = s_mp_proc; i < s_ntr; i++) {
        /* A reader modified-Miller pause is a short run (~2.5 us ~ 5-8 samples); the carrier-on gaps between
         * pauses are always long (>= ~16 samples, one full etu minus a pause). Detect pauses by width, not by
         * level parity: after our own load-mod TX the demod baseline inverts for the following frame, so a
         * polarity test would pick the long carrier runs as pauses. Width is polarity-proof and both
         * directions are cleanly separated (<=12 vs >=16). */
        int w = s_tr[i] - s_tr[i - 1];
        if (w >= 4 && w <= 13) {
            int c = (s_tr[i - 1] + s_tr[i]) / 2;
            if (s_mp_has && c - s_mp_prev >= FRAME_GAP) {          /* between-frame gap: restart on this pause */
                emu_early_stream_cancel();
                __builtin_memset(s_seq, 0, 160);
                s_mp_has = 0; s_mp_maxi = 0; s_mp_np = 0;
                s_grp_k = 1; s_grp_sr = 0; s_grp_nb = 0; s_grp_n = 0; s_grp_par = 0;
                s_grp_prefix_ready = 0;
            }
            miller_inc_place(c);
            s_mp_prev = c; s_mp_np++;
        }
    }
    s_mp_proc = s_ntr;
    if (!s_grp_prefix_ready && s_grp_n == 0 && s_mp_maxi >= 7 && s_grp_byte_ready) {
        uint8_t raw = 0;
        for (int b = 0; b < 7; b++)
            if (s_seq[b + 1] == 2) raw |= (uint8_t)(1u << b);
        s_grp_prefix_ready = 1;
        s_grp_callback_active = 1;
        s_grp_byte_ready(s_grp_byte_ctx, 0, raw, 7);
        s_grp_callback_active = 0;
    }
    if (s_mp_maxi > 2) miller_inc_group(s_mp_maxi - 2);
}

/* Frame end: fold any tail edges, then group the etu grid into bytes (loop-2 of miller_decode). */
__attribute__((optimize("O2")))
static int miller_inc_finish(uint8_t *out, uint32_t *par)
{
    miller_inc_feed();
    miller_inc_group(s_mp_maxi);                                  /* finalize the last (ungrouped) slots */
    int n = s_grp_n;
    if (n == 0 && s_grp_nb >= 6 && s_grp_nb <= 7) {               /* 7-bit short frame (REQA/WUPA) */
        uint8_t raw = (uint8_t)((s_grp_sr >> (9 - s_grp_nb)) & ((1 << s_grp_nb) - 1));
        if (!s_grp_prefix_ready && s_grp_byte_ready) {
            s_grp_prefix_ready = 1;
            s_grp_callback_active = 1;
            s_grp_byte_ready(s_grp_byte_ctx, 0, raw, 7);
            s_grp_callback_active = 0;
        }
        s_grp_out[n++] = raw;
    }
    else if (n >= 1 && s_grp_nb == 8 && n < 32) {
        /* A final sequence-Y parity bit has no pause, so the grid can end one
         * bit short. Do not pad shorter fragments caused by post-frame noise. */
        uint32_t sr = s_grp_sr;
        for (int b = s_grp_nb; b < 9; b++) sr >>= 1;             /* pad the lost trailing 0-bits (Y) */
        int v = sr & 0xFF;
        if ((sr >> 8 & 1) == __builtin_parity(v)) s_grp_par |= (1u << n);
        s_grp_out[n++] = (uint8_t)v;
    }
    for (int i = 0; i < n; i++) out[i] = s_grp_out[i];
    *par = s_grp_par;
    return n;
}

/* Manchester-decode a card frame in [s,e) into bytes; returns byte count. *par gets a bad-parity bitmask
 * (bit i = byte i's 9th bit disagreed with ISO14443-A odd parity). */
__attribute__((optimize("O2")))          /* hot: 200 half-etu windows x O(edges) per card reply */
static int manch_decode(int s, int e, uint8_t *out, uint32_t *par)
{
    *par = 0;
    const uint16_t *tr = s_tr; int nt = s_ntr;   /* hoist the globals: the compiler can't prove s_tr/s_ntr
                                                  * don't alias, so left as globals it reloads both every
                                                  * inner-loop test - a big cost over 200 windows x O(edges). */
    int st = 0, hi = nt;                          /* first edge >= s */
    while (st < hi) { int m = (st+hi)/2; if (tr[m] < s) st = m+1; else hi = m; }
    if (st >= nt) return 0;
    int t0 = tr[st];
    uint8_t bits[200]; int nbits = 0;
    int p = st;                                  /* running edge pointer that only advances: the half-etu
                                                  * windows tile [t0, ...) contiguously (each end == next
                                                  * h1), so one monotonic walk counts every window - the
                                                  * edge list is touched once total (O(edges)), not a
                                                  * fresh sub-walk per bit. This is what lets a big
                                                  * multi-reply force-flush decode in ms, not tens. */
    int h1 = t0, mid = t0 + ETUSAMP/2, end = t0 + ETUSAMP, frac = 0;
    for (int k = 0; k < 200; k++) {
        if (h1 > e + ETUSAMP) break;
        int c1 = 0; while (p < nt && tr[p] < mid) { p++; c1++; }   /* edges in 1st half [h1,mid) */
        int c2 = 0; while (p < nt && tr[p] < end) { p++; c2++; }   /* edges in 2nd half [mid,end) */
        if (c1 < 3 && c2 < 3) break;             /* subcarrier gone in both halves -> reply ended */
        bits[nbits++] = (c1 >= 3) ? 1 : 0;       /* subcarrier 1st half = logic 1 */
        int adv = ETU100 / 100; frac += ETU100 % 100;   /* advance one etu = 25.17 samp, no per-k divide */
        if (frac >= 100) { adv++; frac -= 100; }
        h1 += adv; mid += adv; end += adv;
    }
    int n = 0, i = 1;                            /* skip start bit */
    while (i + 8 <= nbits && n < 32) {
        int v = 0;
        for (int j = 0; j < 8; j++) v |= (bits[i+j] & 1) << j;   /* LSB first */
        if (i + 8 < nbits && (bits[i+8] & 1) == __builtin_parity(v))   /* 9th bit present and == parity(v):
                                                       * ISO14443-A parity is odd, so a correct bit is
                                                       * !parity(v); equal means wrong (encrypted/garbled) */
            *par |= (1u << n);
        out[n++] = (uint8_t)v; i += 9;            /* +9: 8 data + parity */
    }
    /* A 4-bit ACK/NAK reply (start bit + 4 bits, no parity) never fills a 9-bit byte; emit the nibble. */
    if (n == 0 && nbits >= 5 && nbits <= 6) {
        int v = 0;
        for (int j = 1; j <= 4; j++) v |= (bits[j] & 1) << (j - 1);   /* LSB first */
        out[n++] = (uint8_t)(v & 0x0F);
    }
    return n;
}

/* Decoded-frame accumulator (reader + card), sorted by capture time for a sniff trace. */
/* One decoded frame: t/tend are the modulation start/end (origin-relative samples; the caller adds the
 * absolute base and converts to us). par is a bad-parity bitmask - bit i set = byte i's received parity
 * bit disagreed with ISO14443-A odd parity (only the device sees the raw parity bit, so the mark must be
 * made here; the host can't recompute it). CRC and protocol naming are left to the host. */
struct sfr { uint16_t t, tend; char dir; uint8_t n; uint32_t par; uint8_t b[24]; };
static struct sfr *s_fr;      /* decoded frames - heap scratch, live only during a sniff */
static int s_nfr;

static void add_frame(int t, int tend, char dir, const uint8_t *b, int n, uint32_t par)
{
    if (s_nfr >= 48 || n <= 0) return;
    int nz = 0; for (int i = 0; i < n; i++) if (b[i]) nz = 1;
    if (!nz) return;                             /* drop all-zero noise (spurious inter-frame pauses) */
    if (n > 24) n = 24;
    s_fr[s_nfr].t = (uint16_t)t; s_fr[s_nfr].tend = (uint16_t)tend;
    s_fr[s_nfr].dir = dir; s_fr[s_nfr].n = (uint8_t)n; s_fr[s_nfr].par = par;
    for (int i = 0; i < n; i++) s_fr[s_nfr].b[i] = b[i];
    s_nfr++;
}

/* Decode the currently-held edge list s_tr[0..s_ntr) (positions relative to the accumulation origin,
 * spanning `got` samples) into reader/card frames (s_fr[0..s_nfr)); returns the frame count. The edge
 * list is built incrementally by scan_span() and flushed here at each between-frame gap, so no sample
 * is ever scanned twice. No text: the caller formats and accumulates. */
#define S_TXT_CAP 4096
static char *s_txt;        /* formatted trace text - heap scratch, live only during a sniff. A long
                            * transaction (e.g. an mfu dump: ~40 frames) needs room; the app must read at
                            * least this many bytes back or the tail frames are lost */
static uint16_t *s_paus;      /* reader-pause centers for one flush - heap scratch, live only during a sniff */

__attribute__((optimize("O2")))          /* hot: single-pass classify over the whole edge list per flush */
static int sniff_decode(uint32_t got)
{
    s_nfr = 0; s_ongrid = 0; s_offgrid = 0; s_card_samp = 0;
    if (s_ntr < 4 || got < 4) return 0;
    int init = s_tr_init;
    (void)got;

    /* Single pass over the edge list, classifying each interval (the card decode was ~75% of decode
     * time). A short interval is an 847 kHz subcarrier tick; 5 in a row starts a card burst, which
     * runs until a real gap (> 10 etu, not a Manchester half-etu dropout). A wide LOW run outside a
     * burst is a reader 100% ASK pause - so in-burst pauses are excluded for free (no cspan filter),
     * and the two O(n) passes (pos-window card scan + reader scan) become one. */
    uint16_t *paus = s_paus; int np = 0;          /* file-scope heap scratch, aliased local for the hot loop */
    int dense = 0, in_burst = 0, bs = 0, be = 0;
    for (int i = 1; i < s_ntr; i++) {
        int w = s_tr[i] - s_tr[i-1];
        if (w <= 4) {                                 /* subcarrier tick */
            if (++dense == 5 && !in_burst) { in_burst = 1; bs = s_tr[i-5]; }
            if (in_burst) be = s_tr[i];
        } else {                                      /* wide interval */
            if (in_burst) {
                if (s_tr[i] - be > 4 * ETUSAMP) {     /* subcarrier stopped -> card reply ends. Only ~4
                                                       * etu (not 10): a Manchester absent-half is ~0.5
                                                       * etu so this never fragments a reply, but ending
                                                       * promptly stops the burst from swallowing the
                                                       * first pauses of a short following reader command
                                                       * (e.g. 95 20). ACK/NAK are 4 bits - never gated. */
                    uint8_t by[32]; uint32_t par; int nn = manch_decode(bs, be, by, &par);
                    add_frame(bs, be, 'C', by, nn, par);
                    s_card_samp += be - bs; in_burst = 0;
                }
            } else if (w >= 5 && (init ^ (i & 1)) == 0 && np < 600) {
                paus[np++] = (uint16_t)((s_tr[i-1] + s_tr[i]) / 2);   /* reader 100% ASK pause */
            }
            dense = 0;
        }
    }
    if (in_burst) { uint8_t by[32]; uint32_t par; int nn = manch_decode(bs, be, by, &par);
                    add_frame(bs, be, 'C', by, nn, par); s_card_samp += be - bs; }

    for (int fs = 0, i = 1; i <= np; i++)            /* group pauses <3.5 etu apart -> frames */
        if (i == np || paus[i] - paus[i-1] >= FRAME_GAP) {
            if (i - fs >= 4) { uint8_t by[32]; uint32_t par; int nn = miller_decode(paus + fs, i - fs, by, &par);
                               add_frame(paus[fs], paus[i-1], 'R', by, nn, par); }
            fs = i;
        }

    for (int i = 0; i < s_nfr; i++)                  /* time-order the interleaved trace */
        for (int j = i + 1; j < s_nfr; j++)
            if (s_fr[j].t < s_fr[i].t) { struct sfr t = s_fr[i]; s_fr[i] = s_fr[j]; s_fr[j] = t; }

    return s_nfr;
}

/* Append an unsigned decimal (no libc). One divide per digit (derive the remainder from the quotient),
 * since this runs per frame inside the capture loop where the margin is thin. */
static int put_u32(char *out, uint32_t v)
{
    char t[10]; int k = 0;
    do { uint32_t q = v / 10; t[k++] = (char)('0' + (v - q * 10)); v = q; } while (v);
    for (int j = 0; j < k; j++) out[j] = t[k-1-j];
    return k;
}

/* Format each decoded frame (s_fr) as one compact, serial-safe text line:
 *     <R|C> <start_samp> <end_samp> <hex>[!] <hex>[!] ...
 * `base` is the absolute sample offset of this region's origin (added to the frame's origin-relative
 * t/tend). Timestamps are raw sample counts - the host multiplies by the sample period (carried in the
 * L-header) to get us; keeping the divide off the device's hot flush path is what preserves the no-
 * overrun margin. The `!` after a byte flags bad parity. CRC and protocol naming are the host's job too -
 * keep the wire text minimal (serial is a valid transport; no binary, no heavy formatting on the device).
 * Direct writers, not snprintf ("%02X"/"%u" ~800 cyc each = ~40 K in a dense round). Returns bytes. */
static int fmt_frames(char *out, int cap, uint32_t base)
{
    static const char HEX[16] = "0123456789ABCDEF";
    int p = 0;
    for (int i = 0; i < s_nfr && p < cap - 128; i++) {
        struct sfr *f = &s_fr[i];
        out[p++] = f->dir; out[p++] = ' ';
        p += put_u32(out + p, base + f->t); out[p++] = ' ';
        p += put_u32(out + p, base + f->tend);
        for (int j = 0; j < f->n && p < cap - 8; j++) {
            uint8_t v = f->b[j];
            out[p++] = ' '; out[p++] = HEX[v >> 4]; out[p++] = HEX[v & 0xF];
            if (f->par & (1u << j)) out[p++] = '!';   /* parity disagreed with ISO14443-A odd parity */
        }
        out[p++] = '\n';
    }
    return p;
}

/* Extract edges from `len` fresh ring samples (one linear span) into the persistent edge list,
 * single pass - each sample is examined exactly once, ever. Leading dead air (before the first edge of
 * a frame) is dropped so positions stay small; *pos_io tracks the sample offset from the accumulation
 * origin. The capture loop finds frame boundaries over the resulting edge list.
 * -O2 (not the build's -Os): this is the hot path - every captured sample flows through it - and -Os
 * spills seg/len to the stack and reloads them each iteration. At -O2 the whole loop stays in registers
 * and unrolls, which is the difference between keeping pace with the 4 MHz stream and not. The body is
 * kept minimal (pure edge detection); gap/cut detection runs afterwards over the far shorter edge list. */
#define MISO_MASK   (1u << MISO_PIN)
#define MISO_WORD   (MISO_MASK * 0x01010101u)   /* MISO bit replicated into all 4 bytes of a word */
__attribute__((optimize("O2")))
static void scan_span(const uint8_t *seg, int len, int *pos_io)
{
    int lvl = s_lvl, ntr = s_ntr, tr_init = s_tr_init, pos = *pos_io;
    uint16_t *tr = s_tr;
    if (ntr == 0) pos = 0;                              /* drop dead air before a frame's first edge */
    int i = 0;

    /* Record an edge if sample level `b` differs from the running level. `b` is the MISO bit already
     * extracted (0/1) - callers never re-read memory. */
    #define STEPB(b, idx) do {                                          \
        int _b = (int)(b);                                              \
        if (_b != lvl) {                                                \
            if (ntr == 0) tr_init = lvl;                                \
            if (ntr < 6000) tr[ntr++] = (uint16_t)(pos + (idx));        \
            lvl = _b;                                                   \
        }                                                               \
    } while (0)
    #define STEP(idx)  STEPB((seg[idx] >> s_cap_shift) & 1, idx)        /* head/tail: one byte load */

    while (i < len && ((uintptr_t)(seg + i) & 3u)) { STEP(i); i++; }     /* align to a word boundary */
    /* Word SIMD: extract all 4 MISO bits, then XOR each sample against its predecessor (the prior
     * sample fed in at byte 0) - one operation flags every level crossing in the 4 samples. Walk only
     * the set bits (ctz), so a dense subcarrier word costs one XOR plus one store per real edge instead
     * of four extract-compare-branch steps; lvl advances once per word (last sample), not per edge.
     * Records exactly what STEPB would - this is the hot path that must outrun the 3.2 MHz stream. */
    for (; i + 4 <= len; i += 4) {
        uint32_t raw  = *(const uint32_t *)(seg + i);
        uint32_t bits = (raw >> s_cap_shift) & 0x01010101u;              /* capture bit of each of the 4 samples */
        uint32_t edg  = (bits ^ ((bits << 8) | (uint32_t)lvl)) & 0x01010101u;   /* set byte = sample crossed */
        if (edg) {
            do {
                int idx = __builtin_ctz(edg) >> 3;                       /* which sample (0..3) crossed */
                if (ntr == 0) tr_init = lvl;
                if (ntr < 6000) tr[ntr++] = (uint16_t)(pos + i + idx);
                edg &= edg - 1;                                          /* clear that edge, take the next */
            } while (edg);
            lvl = (int)((bits >> 24) & 1u);                              /* running level = last sample */
        }                                                               /* edg==0: all 4 == lvl, nothing to do */
    }
    for (; i < len; i++) STEP(i);                                        /* tail */
    #undef STEP
    #undef STEPB

    s_lvl = lvl; s_ntr = ntr; s_tr_init = tr_init; *pos_io = pos + len;
}

/* A decoded region is real 14443-A activity (vs field-off noise) if its reader pauses fit the etu
 * grid or it carries a real card subcarrier. Noise fits neither, so it's dropped - kept out of the
 * trace and not counted as activity (so the session ends after the real exchange, not on noise). */
static bool region_valid(void)
{
    return (s_ongrid >= 4 && s_ongrid >= s_offgrid) || (s_card_samp >= 246);
}

/* Scan new edges [start,s_ntr) for the last between-frame gap (>= FRAME_GAP) and track the largest gap
 * seen (mg*, the force-flush fallback). Returns cut_ne (0 = none); cut_pos via *cp. -O2 + a cached prev +
 * hoisted globals: this runs every round over ~4000 fresh edges and at -Os was ~30 K cyc/round. */
__attribute__((optimize("O2")))
static int find_cut(int start, int *cp, int *mgv, int *mgn, int *mgp)
{
    const uint16_t *tr = s_tr; int nt = s_ntr;
    int cut_ne = 0, cut_pos = 0, mv = *mgv, mn = *mgn, mp = *mgp, prev = tr[start - 1];
    for (int i = start; i < nt; i++) {
        int cur = tr[i], gap = cur - prev;
        if (gap >= FRAME_GAP) { cut_ne = i; cut_pos = cur; }
        if (gap > mv)         { mv = gap; mn = i; mp = cur; }
        prev = cur;
    }
    *cp = cut_pos; *mgv = mv; *mgn = mn; *mgp = mp;
    return cut_ne;
}

/* Passively sniff a live reader<->card exchange to `dump_path`. Waits (EFD) for the reader field, then
 * measures its strength to set the front-end gain and the coupling colour (green/yellow/red) - so a
 * hand-held Flipper stays correctly gained as the distance drifts, and the trace is coloured by how
 * well it is coupled. Then a circular DMA ring streams the demod and the CPU decodes behind it, cutting
 * only at edge-free between-frame gaps so a transaction of any length streams intact. Gating capture on
 * a detected field means field-off noise is never captured or decoded. Runs until the field has been
 * quiet a while or timeout. [INSTRUMENTED for bring-up: L-header carries rounds/overruns/... after the
 * coupling level; the app colours the frames by that level.] */
static int s_ring_low_warned;   /* latched on ring fallback, cleared when the full ring fits - warn once per episode */
int hal_rfid_hf_sniff(const char *dump_path, uint32_t timeout_ms)
{
    if (!s_spi_up || s_mode != RFID_HF_READER || !dump_path) return RFID_ERR_UNSUPP;
    s_hf_dirty = 1;                                /* from here the HF config is the sniff's, not the reader's,
                                                    * until a real set_mode re-init restores it (see set_mode) */

    /* Field detection + strength first, in normal (SPI) mode - the demod's transparent mode parks the
     * SPI pins, so this cannot be done during capture. Measured in the reader's power-up guard time. */
    int fld = efd_wait_measure(timeout_ms ? timeout_ms : 1500);
    if (fld < 0) { vfs_write_file(dump_path, "", 0); return RFID_ERR_TIMEOUT; }  /* no field: clear the
                                                       * file so the app doesn't re-print the last trace */
    uint8_t gain = (fld <= 2) ? SNIFF_RG_BOOST : SNIFF_RG_MID;  /* far/weak -> boost, near/strong -> mid */
    int couple = (fld >= 6) ? 0 : (fld >= 4) ? 1 : 2;    /* 0 green (strong/good), 1 yellow (fair), 2 red (weak) */

    /* The capture ring AND all decode scratch (s_tr/s_fr/s_txt, ~17 KB) live on the heap only for the
     * duration of a sniff. As static BSS they'd pin that memory below the app heap forever for a feature
     * that is rarely the one running - free it back when idle. */
    /* Reserve the small fixed scratch (~18 KB) first, then the ring last so the ring flexes to the
     * largest block that remains. Allocating the big ring first would grab the one large free block and
     * starve the scratch - OOM despite plenty of total free heap (just fragmented), and the ring's own
     * fallback never triggers because the ring itself "fit". */
    s_tr   = pvPortMalloc(6000 * sizeof *s_tr);
    s_fr   = pvPortMalloc(48   * sizeof *s_fr);
    s_txt  = pvPortMalloc(S_TXT_CAP);
    s_paus = pvPortMalloc(600  * sizeof *s_paus);
    s_seq  = pvPortMalloc(160);
    uint32_t ring_samples = RING_SAMPLES;
    uint8_t *ring = pvPortMalloc(RING_SAMPLES);
    if (!ring) {                                  /* full ring won't fit - fall back to a smaller one */
        ring_samples = RING_SAMPLES_MIN;
        ring = pvPortMalloc(RING_SAMPLES_MIN);
    }
    if (!ring || !s_tr || !s_fr || !s_txt || !s_paus || !s_seq) {
        vPortFree(ring); vPortFree(s_tr); vPortFree(s_fr); vPortFree(s_txt); vPortFree(s_paus); vPortFree(s_seq);
        s_tr = 0; s_fr = 0; s_txt = 0; s_paus = 0; s_seq = 0;
        /* Not even the reduced ring fit: report OOM (need = the minimum, the bar we failed). */
        unsigned freeb = (unsigned)xPortGetFreeHeapSize();
        static TickType_t last_oom;
        TickType_t now = xTaskGetTickCount();
        if (now - last_oom > pdMS_TO_TICKS(2000)) {
            last_oom = now;
            fantasi_log(LOG_WARN, "sniff: out of memory (need %u B ring, %u B free)", (unsigned)RING_SAMPLES_MIN, freeb);
        }
        char msg[80];
        int n = snprintf(msg, sizeof msg, "sniff: out of memory - need %u B, %u B free (see `free`)\r\n",
                         (unsigned)RING_SAMPLES_MIN, freeb);
        vfs_write_file(dump_path, msg, (uint32_t)n);
        return n;   /* >0 so the module surfaces the reason to the user */
    }

    /* Reduced ring: warn once per episode (log breadcrumb + inline trace notice, prepended below). */
    int reduced = (ring_samples != RING_SAMPLES), warn_now = 0;
    if (reduced) {
        if (!s_ring_low_warned) {
            s_ring_low_warned = 1; warn_now = 1;
            fantasi_log(LOG_WARN, "sniff: low memory - ring reduced to %u B (full %u B)",
                        (unsigned)RING_SAMPLES_MIN, (unsigned)RING_SAMPLES);
        }
    } else {
        s_ring_low_warned = 0;
    }

    s_cap_idr = &GPIOB->IDR; s_cap_shift = MISO_PIN;   /* sniff reads the demod on PB4/MISO (reader mode) */
    sniff_config(gain);
    direct_cmd(CMD_TRANSPARENT_MODE);
    miso_as_input();
    vTaskDelay(pdMS_TO_TICKS(1));
    dma_circ_arm(ring, ring_samples);
    dma_cap_go();

    /* Reset the incremental extractor. s_lvl seeds from the first ring sample; the first real edge
     * anchors the frame. Everything decode-side (s_tr/s_fr) is static, so no scratch buffer. */
    s_ntr = 0;
    s_lvl = (ring[0] >> MISO_PIN) & 1; s_tr_init = s_lvl;
    int pos = 0;

    const int FSTART = 40;                        /* room for the leading L-header line */
    int plen = FSTART, saw_activity = 0, rounds = 0, overrun = 0, valid = 0;
    if (warn_now)                                 /* inline notice; host beautifier passes non-frame lines through */
        plen += snprintf(s_txt + plen, (size_t)(S_TXT_CAP - plen),
                         "sniff: reduced ring to %u B - low memory (see `free`)\n", (unsigned)RING_SAMPLES_MIN);
    uint32_t read_abs = 0, wr_hi = 0, last_wpos = 0, max_bl = 0;   /* max_bl: peak backlog, health metric */
    uint32_t abs_base = 0;                        /* absolute sample offset of the current edge-list origin,
                                                   * so frame timestamps become absolute-in-capture (for us) */
    int mg_val = 0, mg_ne = 0, mg_pos = 0;        /* largest edge gap held (fallback flush boundary) */
    UBaseType_t base_prio = uxTaskPriorityGet(NULL);
    int boosted = 0;                              /* raised above the USB task while a burst drains */
    TickType_t t0 = xTaskGetTickCount(), last_active = t0;

    for (;;) {
        /* ABSOLUTE backlog. The naive (write - read) % RING WRAPS once we fall a whole ring behind, so a
         * deep backlog looks small and the overrun check is evaded while the DMA silently overwrites the
         * data we're about to read (corrupting frames mid-transaction). Track write position across ring
         * wraps and measure the true, unwrapped backlog. Rounds/waits are << the ~16 ms ring period, so
         * no wrap is missed. */
        uint32_t write_pos = ring_samples - DMA1_Channel1->CNDTR;
        if (write_pos < last_wpos) wr_hi += ring_samples;   /* DMA wrapped the ring */
        last_wpos = write_pos;
        uint32_t backlog = (wr_hi + write_pos) - read_abs;
        if (backlog > max_bl) max_bl = backlog;

        /* Keep decoding fast enough and the backlog stays bounded - but a higher-priority USB task
         * flushing the PREVIOUS trace can park us for ms, and a read arriving in that window spikes the
         * backlog. Since the EFD gates capture to an ACTIVE field, stay boosted above USB the whole time
         * a transaction is live (recent valid frame); the USB IRQ keeps the device alive and the flush
         * runs between captures (during the EFD field-wait, at base prio). Drop back once field-quiet. */
        int active = saw_activity && (xTaskGetTickCount() - last_active) < pdMS_TO_TICKS(40);
        if (active && !boosted)      { vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1); boosted = 1; }
        else if (!active && boosted) { vTaskPrioritySet(NULL, base_prio); boosted = 0; }

        if (backlog > 9 * ring_samples / 10) {        /* fell behind (or lapped) -> catch up to the head */
            read_abs = wr_hi + write_pos; overrun++;
            s_ntr = 0; pos = 0;
            s_lvl = (ring[read_abs % ring_samples] >> MISO_PIN) & 1; s_tr_init = s_lvl;
            continue;
        }
        if (backlog < PROCESS_MIN) {                  /* caught up / data still trickling in: wait for a
                                                       * worthwhile batch instead of spinning tiny rounds
                                                       * (per-round overhead + a context switch each time
                                                       * would otherwise burn most of the CPU). During a
                                                       * real burst backlog races past this, so we never
                                                       * stall mid-transaction. */
            TickType_t now = xTaskGetTickCount();
            if ((now - t0) >= pdMS_TO_TICKS(timeout_ms ? timeout_ms : 1500)) break;
            if (saw_activity && s_ntr == 0 && (now - last_active) >= pdMS_TO_TICKS(QUIET_MS)) break;
            vTaskDelay(1);
            continue;
        }
        uint32_t take = backlog - RING_MARGIN;
        if (take > MAX_SCAN) take = MAX_SCAN;
        rounds++;

        /* Single pass: scan the new ring samples in place (two linear spans across the wrap), building
         * edges. No copy, no re-scan of carried data - the whole reason we keep pace with the stream. */
        int ntr0 = s_ntr;                             /* edges already held (the carried in-flight frame) */
        if (ntr0 == 0) abs_base = read_abs;           /* scan_span drops dead air (pos=0) with no carried
                                                       * edges, so the origin re-bases to this segment start */
        uint32_t read_pos = read_abs % ring_samples;
        uint32_t first = ring_samples - read_pos;
        if (first > take) first = take;
        scan_span(ring + read_pos, (int)first, &pos);
        if (take > first) scan_span(ring, (int)(take - first), &pos);
        read_abs += take;

        /* Find the last between-frame gap (edge spacing >= FRAME_GAP). The carried edges [0,ntr0) are one
         * in-flight frame with no internal gap, so only NEW edges can introduce one - scan from ntr0, not
         * from 0 (incremental; avoids re-scanning a big carried reply every round). A trailing edge-free
         * run >= FRAME_GAP means even the last frame is done; a full edge list is force-flushed. */
        int cut_pos = 0;
        int cut_ne = find_cut((ntr0 > 1 ? ntr0 : 1), &cut_pos, &mg_val, &mg_ne, &mg_pos);
        if (s_ntr > 0 && pos - s_tr[s_ntr-1] >= FRAME_GAP) { cut_ne = s_ntr; cut_pos = pos; }
        /* No clean between-frame gap but the edge list is getting big (a noisy continuous stretch, e.g.
         * weak inter-reply gaps): flush at the LARGEST gap seen - the likeliest real boundary - so the
         * region stays small and its decode cheap. A huge force-flush is a multi-ms stall that spikes the
         * backlog; keeping regions bounded is what lets a long dense transaction not fall behind. */
        if (cut_ne == 0 && s_ntr >= 3000 && mg_ne > 0)  { cut_ne = mg_ne; cut_pos = mg_pos; }
        if (cut_ne == 0 && s_ntr >= 5990)               { cut_ne = s_ntr; cut_pos = pos; }

        /* Flush the complete frames; carry the in-flight frame's edges rebased to a fresh origin. */
        if (cut_ne > 0) {
            int full = s_ntr;
            s_ntr = cut_ne;
            sniff_decode((uint32_t)cut_pos);
            if (region_valid()) {
                valid++; saw_activity = 1; last_active = xTaskGetTickCount();
                if (s_nfr && plen < (int)S_TXT_CAP - 160)
                    plen += fmt_frames(s_txt + plen, (int)S_TXT_CAP - plen, abs_base);
            }
            s_ntr = full - cut_ne;                    /* carry the trailing (in-flight) edges */
            for (int i = 0; i < s_ntr; i++) s_tr[i] = s_tr[i + cut_ne] - (uint16_t)cut_pos;
            s_tr_init ^= (cut_ne & 1);
            pos -= cut_pos;
            abs_base += cut_pos;                      /* origin advanced past the flushed region */
            mg_val = 0; mg_ne = 0; mg_pos = 0;        /* carried tail is one frame, no big gap: rescan fresh */
        }

        /* Yield to the equal-priority app I/O tasks (USB flush of the previous trace) only when we're
         * near caught-up. Processing far outruns the stream, so the only way the backlog grows is us being
         * parked: a taskYIELD mid-burst lets USB/apppump run a whole tick while the DMA fills the ring. So
         * while a burst is draining (deep backlog) we run flat out; the higher-priority USB IRQ still
         * preempts us, and once drained the wait path below yields plenty. */
        if (backlog < 3u * PROCESS_MIN && (rounds & 3) == 0) taskYIELD();

        TickType_t now = xTaskGetTickCount();
        if ((now - t0) >= pdMS_TO_TICKS(timeout_ms ? timeout_ms : 1500)
            || plen >= (int)S_TXT_CAP - 96) break;
        /* End promptly once the reader's field has gone (no valid frame for QUIET_MS) - even if field-off
         * noise keeps the ring busy so the caught-up break below never sees s_ntr == 0. */
        if (saw_activity && (now - last_active) >= pdMS_TO_TICKS(QUIET_MS)) break;
    }
    if (boosted) vTaskPrioritySet(NULL, base_prio);   /* never leave the task boosted */

    dma_cap_stop();
    if (s_ntr > 0) {                                  /* decode the last held (possibly partial) frame */
        sniff_decode((uint32_t)pos);
        if (region_valid() && s_nfr && plen < (int)S_TXT_CAP - 160)
            plen += fmt_frames(s_txt + plen, (int)S_TXT_CAP - plen, abs_base);
    }
    miso_as_spi();
    direct_cmd(CMD_STOP);
    vPortFree(ring);

    int ret;
    if (plen <= FSTART) {
        vfs_write_file(dump_path, s_txt, 0);
        ret = RFID_ERR_TIMEOUT;
    } else {
        /* Leading header: L<couple> drives the host's per-capture colour; then a quick health readout (field
         * level, overrun count - should be 0, valid frames, peak ring backlog in samples) and p<ns> = the
         * sample period so the host turns the frames' raw sample timestamps into us without the device ever
         * dividing. Period = (CAP_ARR+1)/32 us = (CAP_ARR+1)*1000/32 ns (375 ns at 2.667 MHz). */
        char h[48]; int hl = snprintf(h, sizeof h, "L%d f%d o%d v%d m%lu p%u\n",
                                      couple, fld, overrun, valid, (unsigned long)max_bl,
                                      (unsigned)((CAP_ARR + 1) * 1000u / 32u));
        memcpy(s_txt + (FSTART - hl), h, (size_t)hl);
        vfs_write_file(dump_path, s_txt + (FSTART - hl), (uint32_t)(plen - (FSTART - hl)));
        ret = valid;
    }
    vPortFree(s_tr); vPortFree(s_fr); vPortFree(s_txt); vPortFree(s_paus); vPortFree(s_seq);
    s_tr = 0; s_fr = 0; s_txt = 0; s_paus = 0; s_seq = 0;
    return ret;
}

/* ============ HF ISO14443-A READER (transparent-mode software reader) ============
 * The ST25R3916 framing engine does not read a card in this HydraNFC config, so - like the sniff and the emu -
 * the reader runs in transparent mode: generate the carrier (tx_en on), modified-Miller ASK-modulate it by
 * bit-banging MOSI (the emu's load-mod gate, here cutting our own field), then reuse the sniff's capture +
 * demod (scan_span + sniff_decode) to read the card's 847 kHz subcarrier reply. The RX half is already proven
 * (the sniff decodes both directions from the exact same demod); the new part is the reader Miller TX. */
#define RDR_MOSI_HI  (GPIOB->BSRR = (1u << 5))            /* MOSI/PB5 HI  = carrier pause (AM off)   */
#define RDR_MOSI_LO  (GPIOB->BSRR = (1u << (5 + 16)))     /* MOSI/PB5 LO  = carrier on                */
/* modified-Miller symbols, one per air-bit; each is 8 sub-periods of 16 fc (MSB first), a set bit = a carrier
 * pause during that sub-period. Same encoding the PM3 clocks out its SSC. */
#define RDR_SEC_X 0x0c   /* pause mid-bit  -> logic 1                */
#define RDR_SEC_Y 0x00   /* no pause       -> 0 after 1 / idle / EOC */
#define RDR_SEC_Z 0xc0   /* pause at start -> 0 after 0 / SOC        */
#define RDR_RING  8192u  /* ~3.3 ms MISO capture @2.46 MHz - TX + FDT + card reply fit */

static uint8_t *s_rdr_ring;
static bool     s_rdr_up;
static uint8_t  s_rdr_gain = 0x08;   /* RX_CONF3 first-stage gain (tunable; our own field, so the reply is strong) */

static inline uint8_t rdr_oddpar(uint8_t x) { return (uint8_t)(!__builtin_parity(x)); }

/* Encode a reader frame into modified-Miller SEC bytes (one per air-bit + SOC/EOC). Ported from the PM3's
 * code_14a_reader_ex; `par` supplies per-byte parity (Crypto1 frames) or NULL for the byte's own odd parity. */
static int rdr_miller_encode(const uint8_t *cmd, int bits, const uint8_t *par, uint8_t *ts, int cap)
{
    int last = 0, n = 0;
#define RPUSH(b) do { if (n >= cap) return -1; ts[n++] = (uint8_t)(b); } while (0)
    RPUSH(RDR_SEC_Z);                               /* start of communication */
    int bytecount = (bits + 7) / 8;
    for (int i = 0; i < bytecount; i++) {
        uint8_t b = cmd[i];
        int bitsleft = bits - i * 8; if (bitsleft > 8) bitsleft = 8;
        int j;
        for (j = 0; j < bitsleft; j++) {            /* LSB first within the byte */
            if (b & 1)          { RPUSH(RDR_SEC_X); last = 1; }
            else if (last == 0) { RPUSH(RDR_SEC_Z); }
            else                { RPUSH(RDR_SEC_Y); last = 0; }
            b >>= 1;
        }
        if (j == 8) {                               /* parity only on a whole byte */
            uint8_t p = par ? (par[i] & 1) : rdr_oddpar(cmd[i]);
            if (p)              { RPUSH(RDR_SEC_X); last = 1; }
            else if (last == 0) { RPUSH(RDR_SEC_Z); }
            else                { RPUSH(RDR_SEC_Y); last = 0; }
        }
    }
    if (last == 0) RPUSH(RDR_SEC_Z); else RPUSH(RDR_SEC_Y);   /* end of communication */
    RPUSH(RDR_SEC_Y);
#undef RPUSH
    return n;
}

/* MOSI (PB5) output-low (modulation) / SPI-AF (reg ops) helpers are shared with the emu TX code below. */
static void mosi_as_output_low(void);
static void mosi_as_spi(void);

/* Bit-bang the modified-Miller SEC stream on MOSI, DWT-timed, IRQs off. Each SEC byte = one 106 kbit/s bit
 * period = 8 sub-periods of 16 fc (302 cyc @32 MHz); a set bit cuts the carrier for that sub-period. */
static void reader_tx(const uint8_t *sec, int n)
{
    static const uint8_t dur[8] = { 38, 38, 37, 38, 38, 38, 37, 38 };   /* sum 302 = 128 fc @32 MHz */
    uint32_t pm = __get_PRIMASK(); __disable_irq();
    uint32_t t = DWT->CYCCNT + 64;
    for (int i = 0; i < n; i++) {
        uint8_t s = sec[i];
        for (int e = 0; e < 8; e++) {
            if ((s >> (7 - e)) & 1) RDR_MOSI_HI; else RDR_MOSI_LO;
            t += dur[e];
            int g = 0;
            while ((int32_t)(DWT->CYCCNT - t) < 0 && ++g < 20000) { }   /* fail-safe: never hang if DWT is dead */
        }
    }
    RDR_MOSI_LO;                                     /* carrier back on */
    if (!pm) __enable_irq();
}

static void reader_config(void)
{
    direct_cmd(CMD_STOP);
    reg_write(REG_FIELD_ON_GT, 0);
    reg_set(REG_OP_CONTROL, OP_CONTROL_en | OP_CONTROL_rx_en | OP_CONTROL_tx_en);   /* FIELD ON (reader) */
    reg_write(REG_RX_CONF1, 0x03);                  /* wideband AM (captures the 847 kHz card subcarrier) */
    reg_write(REG_RX_CONF2, 0x40);                  /* coherent mixer demod, AGC off */
    reg_write(REG_RX_CONF3, s_rdr_gain);
    reg_write(REG_RX_CONF4, 0x00);
    reg_write(REG_AUX, AUX_dis_corr);
    reg_clear(REG_IO_CONF2, IO_CONF2_miso_pd);
    direct_cmd(CMD_RESET_RX_GAIN);
    direct_cmd(CMD_CLEAR_FIFO);
    direct_cmd(CMD_UNMASK_RECEIVE);
}

static int reader_setup(void)
{
    s_rdr_ring = pvPortMalloc(RDR_RING);
    s_tr   = pvPortMalloc(6000 * sizeof *s_tr);
    s_fr   = pvPortMalloc(48   * sizeof *s_fr);
    s_paus = pvPortMalloc(600  * sizeof *s_paus);
    s_seq  = pvPortMalloc(160);
    if (!s_rdr_ring || !s_tr || !s_fr || !s_paus || !s_seq) {
        vPortFree(s_rdr_ring); vPortFree(s_tr); vPortFree(s_fr); vPortFree(s_paus); vPortFree(s_seq);
        s_rdr_ring = 0; s_tr = 0; s_fr = 0; s_paus = 0; s_seq = 0;
        return -1;
    }
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  /* DWT cycle counter for the Miller bit-bang timing */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_cap_idr = &GPIOB->IDR; s_cap_shift = MISO_PIN; s_cap_arr = CAP_ARR;   /* capture MISO/PB4 like the sniff */
    reader_config();
    direct_cmd(CMD_TRANSPARENT_MODE);
    mosi_as_output_low();                            /* MOSI drives the AM gate (idle low = carrier on) */
    miso_as_input();                                 /* MISO carries the demod for capture */
    vTaskDelay(pdMS_TO_TICKS(1));
    s_rdr_up = true;
    return 0;
}

static void reader_teardown(void)
{
    if (!s_rdr_up) return;
    miso_as_spi(); mosi_as_spi();                    /* pins back to SPI so reg ops work */
    direct_cmd(CMD_STOP);
    reg_clear(REG_OP_CONTROL, OP_CONTROL_tx_en | OP_CONTROL_rx_en);   /* field off */
    vPortFree(s_rdr_ring); vPortFree(s_tr); vPortFree(s_fr); vPortFree(s_paus); vPortFree(s_seq);
    s_rdr_ring = 0; s_tr = 0; s_fr = 0; s_paus = 0; s_seq = 0;
    s_rdr_up = false;
}

/* Core reader transceive in transparent mode: Miller-TX the command (with per-byte parity `par`, or NULL for
 * the byte's own odd parity), capture the MISO demod across the FDT + card reply, then reuse the sniff decoder
 * to pull out the card ('C') frame. If `rx_par` is non-NULL it receives the tag's raw per-byte parity bit (for
 * the nested-nonce attack). Returns rx bit count, or <0. */
static int reader_xcv(const uint8_t *tx, int tx_bits, const uint8_t *par, uint8_t *rx, int rx_cap, uint8_t *rx_par)
{
    if (!s_rdr_up && reader_setup() != 0) return RFID_ERR_UNSUPP;   /* lazy: enter transparent+field on first
                                                                     * transceive, so set_mode(HF_READER) alone
                                                                     * leaves the frontend plain for the sniff */
    uint8_t sec[200];
    int nsec = rdr_miller_encode(tx, tx_bits, par, sec, sizeof sec);
    if (nsec < 0) return RFID_ERR_FRAMING;

    s_ntr = 0;
    /* Shield the whole exchange from preemption. reader_tx already masks IRQs for
     * the TX bit-bang; extend that over the FDT capture-wait too. Bounded by a
     * DWT deadline so a dead DMA can't mask for long; the DMA captures in HW. */
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    dma_circ_arm(s_rdr_ring, RDR_RING);
    dma_cap_go();
    reader_tx(sec, nsec);                            /* DMA (TIM2, HW) keeps capturing through the bit-bang */

    /* Wait for FDT + reply after the TX ends (not a fixed count from start - a long frame's own TX would
     * otherwise satisfy it before the card answers). ~4200 samples @2.46 MHz = ~1.7 ms covers FDT + an
     * 18-byte encrypted READ reply (16 data + 2 CRC = ~1.5 ms at 106 kbit/s). */
    uint32_t need = (RDR_RING - DMA1_Channel1->CNDTR) + 4200;
    if (need > RDR_RING) need = RDR_RING;
    uint32_t deadline = DWT->CYCCNT + 32000u * 3;    /* 3 ms hard cap @32 MHz (wait is normally <=1.7 ms) */
    while ((RDR_RING - DMA1_Channel1->CNDTR) < need && (int32_t)(DWT->CYCCNT - deadline) < 0) { }
    dma_cap_stop();
    uint32_t got = RDR_RING - DMA1_Channel1->CNDTR;
    if (!pm) __enable_irq();
    if (got < 8) return RFID_ERR_TIMEOUT;

    s_ntr = 0; int pos = 0;
    s_lvl = (s_rdr_ring[0] >> MISO_PIN) & 1; s_tr_init = s_lvl;
    scan_span(s_rdr_ring, (int)got, &pos);
    sniff_decode(got);
    for (int i = 0; i < s_nfr; i++)                  /* first card ('C') frame after our TX = the reply */
        if (s_fr[i].dir == 'C') {
            int nn = s_fr[i].n;
            if (nn > rx_cap) return RFID_ERR_OVERFLOW;
            for (int j = 0; j < nn; j++) rx[j] = s_fr[i].b[j];
            if (rx_par)                             /* raw received parity: sniff stores (par != odd-parity) */
                for (int j = 0; j < nn; j++)
                    rx_par[j] = (uint8_t)(rdr_oddpar(rx[j]) ^ ((s_fr[i].par >> j) & 1));
            return nn * 8;
        }
    return RFID_ERR_TIMEOUT;
}

/* ============ HF ISO14443-A / MIFARE Classic TAG emulation (passive target) ============
 * The ST25R3916 remains in transparent mode for the complete exchange. TIM2 and DMA capture the reader's
 * envelope from PB4/MISO for software Miller decoding. Replies load-modulate the field through PB5/MOSI;
 * TIM17 schedules the frame delay and TIM1 with two DMA channels generates the Manchester waveform. */

/* Air-symbol values the module hands us (mirror iso14a_emu.h; the HAL can't include the app header). */
#define EMU_SEC_D  0xF0     /* logic 1 - subcarrier in the first half-bit */
#define EMU_SEC_F  0x00     /* stop / idle */
#define EMU_FIFO   64       /* a reader command / tag reply is small (<= 18-byte read + parity) */
/* NOTE: emulation runs the CPU at 64 MHz (emu_clk_boost, see emu_setup) so decode+dispatch fit the 87 us FDT.
 * All DWT-cycle constants below are therefore 64 MHz (2x the 32 MHz values). The sample rate stays 2.462 MHz
 * (s_cap_arr=25 at 64 MHz), so the sample-domain constants (ETU100, gap-52) are unchanged. */
#define EMU_CHIP_SUBC 0
#define EMU_PHASE_OFF 66u    /* fixed reply-start offset (cyc). Sweeping this was proven to have no effect on decode
                             * (PM3 Mod_Manchester_LUT accepts >=3 of 4 sub-samples, so fine phase is not the lever). */
/* Reply FDT (frame delay time), in 64 MHz CPU cycles from the carrier-locked final reader edge. TIM17 fires
 * shortly before the calibrated transmit point; the raw and DMA paths then align the first modulation edge. */
#define EMU_FDT_1     5834u
#define EMU_FDT_0     5532u
#define EMU_FDT_TICKS EMU_FDT_1
#define EMU_FDT_ARR   (EMU_FDT_0 - 600u)  /* enter early; the TX path still phase-pins the first edge */
/* BASEPRI level that masks the USB ISR (pri 5) but not the reply path (EXTI4/TIM17/DMA1_Ch2 all at pri 4). Raised
 * across the FDT-critical reply-staging window (recv-return -> module -> send -> arm) so a USB interrupt can't
 * preempt it and blow the turnaround past the FDT. FreeRTOS-safe (= configMAX_SYSCALL level), unlike __disable_irq. */
#define EMU_MASK_USB  (5u << (8u - 4u))   /* configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5) << (8 - configPRIO_BITS(4)) = 0x50 */
#define EMU_SAMP_CYC  26u   /* one 2.462 MHz capture sample = 64e6/2.462e6 = 26 CPU cycles @64 MHz */
#define CMD_GOTO_SENSE 0xCD /* put the passive target into the sense (idle) state, waiting for REQA */

/* Subcarrier timing in 64 MHz CPU cycles: fc/16 = 847.5 kHz. Fractional accumulation keeps both the raw and
 * DMA paths at 302 + 22/339 cycles per half-bit without allowing the rounded intervals to drift. */
#define EMU_SUBC_HALF 38u
#define EMU_HALF_ETU  302u                               /* integer part of one modulated/idle half-bit @64 MHz */
#define EMU_ETU       (2u * EMU_HALF_ETU)                /* legal fc/128 FDT step */
#define EMU_HALF_FRAC_NUM 22u                            /* exact half-bit: 102400/339 = 302 + 22/339 cycles */
#define EMU_HALF_FRAC_DEN 339u
#define EMU_MOSI_HI  (GPIOB->BSRR = (1u << 5))           /* PB5 = MOSI = load-modulation gate */
#define EMU_MOSI_LO  (GPIOB->BSRR = (1u << (5 + 16)))
#define EMU_RX_SETTLE 2600u                              /* 40.6 us, before the reader's next-frame FDT */
#define EMU_ACTIVE_US 1000u                              /* spans a poller's roughly 520 us WUPA retry interval */
#define EMU_NOISE_US 20000u                              /* longer than a complete poller's WUPA retry train */
#define EMU_WAKE_LOOKBACK 128u                           /* 52 us of margin before the edge-woken task runs */
#define EMU_WAKE_GRACE_US 1000u                          /* finish an edge-woken frame before recv times out */

static bool s_emu_up;                 /* HF_EMU configured (recv/send guard) */
static bool s_emu_starting;           /* setup owns resources but is not ready to receive yet */
static bool s_emu_clock_owned;        /* setup configured or selected the emulation PLL */
static void mosi_as_output_low(void);   /* fwd decl (defined with the TX code) */
static void emu_rx_disable(void);       /* SCLK low disables transparent-mode reception during our reply */
static void emu_rx_enable(void);        /* SCLK high re-enables transparent-mode reception */
static int emu_clk_boost(void);         /* fwd decl: 32->64 MHz for the FDT-critical emu path */
static int emu_clk_restore(void);       /* fwd decl: back to 32 MHz on teardown */
static uint32_t carrier_lock(uint32_t last_edge);
static void mosi_as_spi(void);          /* fwd decl (defined with the TX code) */
static void emu_tx_pat_init(void);      /* fwd decl: DMA-TX edge-pattern precompute */
static void emu_tx_dma_setup(void);     /* fwd decl: DMA-TX timer/DMA config */
static uint32_t s_emu_rx_cyc;         /* DWT cycles at RXE - the reply is timed to the FDT from here */
static volatile uint32_t s_fdt_ticks = EMU_FDT_TICKS;   /* reply FDT (CPU cyc from the last reader edge) */
static volatile uint32_t s_fdt_arr = EMU_FDT_ARR;       /* TIM17 fire point before the calibrated transmit time */
static UBaseType_t s_emu_base_prio;   /* task priority while idle-polling */
static volatile TickType_t s_last_frame_tick;   /* tick of the last real decoded reader frame; the reader-gone
                                                 * stop-key yield only engages once this is >200 ms stale */
static TaskHandle_t s_emu_task;       /* task running hal_rfid_hf_emu_recv (notified from the EXTI wake ISR) */
static volatile int s_emu_waiting;    /* only the first edge of an idle wait needs a deferred task notification */
static volatile uint32_t s_last_edge_cyc;   /* DWT time of the most recent MISO edge (real-time gap detect) */
/* Reply staged for the HW-timer FDT: the recv/module path STAGES here; TIM17 (armed per-edge in the EXTI ISR)
 * fires the bit-bang at last_edge + FDT, independent of task scheduling, so TX has no jitter. */
static uint8_t          s_tx_bits[EMU_FIFO];
static volatile int     s_tx_nbits;
static volatile int     s_tx_armed;      /* 1 = a reply is staged and waiting for the FDT timer to fire it */
static volatile int     s_tx_force_raw;  /* first use: transmit now, then populate the DMA edge cache */
static volatile int     s_tx_dma_pending;
static const uint8_t   *s_tx_dma_key;
/* Crypto replies (auth at / nested nt / encrypted READ) are pre-encoded in module context. These flags select the
 * one-shot DMA scratch rather than the static reply cache and are cleared after the reply fires. */
static volatile int      s_tx_crypto;
static volatile int      s_tx_crypto_dma;
static volatile int     s_fdt_locked;    /* 1 = frame ended + reply staged: freeze the FDT timer (ignore edges) */
enum { EMU_SPEC_NONE, EMU_SPEC_ARMED, EMU_SPEC_FIRED };
static volatile int     s_tx_spec_state;
static volatile uint32_t s_tx_end_cyc;   /* DWT time RX may resume after a reply and demodulator settling */
/* Small reply cache: the select sequence alternates ts_atqa/ts_uid/ts_sak/ts_nt (a different pointer each
 * frame), so a single-entry cache thrashed. Cache each by pointer -> pre-decoded air-bits, so a repeat is a
 * short memcpy into s_tx_bits instead of the symbol->bit loop on the FDT-critical path. */
#define SEND_CACHE_N 6
static struct { const uint8_t *ptr; int n; uint8_t bits[EMU_FIFO]; } s_send_cache[SEND_CACHE_N];
static uint8_t          s_send_cache_rr;  /* round-robin evict index */
#define EMU_RING   8192u      /* ~3.3 ms transparent-demod capture ring @2.46 MHz */
#define EMU_MAXTR  2048       /* edges held while framing one reader command */
static uint8_t *s_emu_ring;   /* transparent-demod capture ring - heap, live for the HF_EMU session */
volatile uint32_t s_edge_dwt[24];   /* reader modulation-edge DWT timestamps used for carrier phase-lock */
volatile int s_edge_n;
volatile int s_phase_off = 66;      /* runtime reply-phase offset (poke via SWD for interleaved A-B sweeps) */
volatile int s_carrier_en = 1;
volatile int s_use_dma_tx = 1;   /* 0=busy-wait TX, 1=DMA TX (poke via SWD to A-B) */
volatile int s_txskip = 600;    /* self-TX MISO-skip margin in cycles (theoretically sound; no measured effect on anticoll) */
volatile int s_ptmod = 0x0F;    /* REG_PT_MOD load-mod depth: sweep 0x0F..0x08 all keep ATQA 8/8, none fix anticoll */
volatile int s_rxc1 = 0x12;     /* TEMP DIAG: REG_RX_CONF1 (HPF zero for post-TX baseline recovery). 0x12=h80, 0x14=h200, 0x1A=z600k */
volatile int s_rxc4 = 0x00;     /* TEMP DIAG: REG_RX_CONF4 (2nd/3rd-stage gain / digitizer) */
volatile int s_rxc2 = 0x2D;     /* TEMP DIAG: REG_RX_CONF2. bit5 sqm_dyn (squelch, fires 18.88us after our TX!), bit3 agc_en. 0x0D=no squelch, 0x25=no AGC, 0x05=neither */
volatile int s_emu_env = 1;     /* TEMP DIAG: 0=424kHz correlator (baseline), 1=sniff-style envelope demod (no phase-sign inversion) */
volatile int s_rxc3 = 0x78;     /* TEMP DIAG: REG_RX_CONF3 first-stage gain for the envelope variant */
volatile int s_rxc3b = 0xD8;    /* TEMP DIAG: REG_RX_CONF3 for the correlator (baseline) path - sweep low to keep demod linear during TX */
static void emu_edge_cache_free(void);    /* fwd decls: DMA edge cache (defined with the TX code) */
static void emu_edge_cache_alloc(void);
static int emu_early_stream_commit(int frame_len);
static int emu_early_stream_pending(void);
static void emu_free(void)
{
    vPortFree(s_emu_ring); vPortFree(s_tr); vPortFree(s_seq);
    s_emu_ring = 0; s_tr = 0; s_seq = 0;
    emu_edge_cache_free();
}

/* Register-only emu shutdown: masks EXTI, disables every emu NVIC vector, stops the FDT/TX timers, the TX DMA
 * and the capture DMA, and NULLS s_emu_task. No FreeRTOS calls, no SPI, no delays - so it is safe to invoke
 * from a trace/ISR context. This is the load-bearing half of the teardown: it must run the instant the emu
 * task goes away (kill / USB disconnect) or the EXTI4 ISR keeps firing vTaskNotifyGiveFromISR() on the freed
 * task handle -> corrupts the reused TCB/heap -> wild execution.
 * s_hf_dirty forces the next set_mode(HF_EMU) to do a full teardown+re-init (freeing the leaked buffers). */
static void emu_isr_kill(void)
{
    EXTI->IMR1 &= ~(1u << 4);                           /* disable the MISO edge wake interrupt */
    NVIC_DisableIRQ(EXTI4_IRQn);
    EXTI->RTSR1 &= ~(1u << 4); EXTI->FTSR1 &= ~(1u << 4);
    NVIC_DisableIRQ(EXTI0_IRQn);                         /* deferred recv-wake vector */
    TIM17->CR1 &= ~TIM_CR1_CEN; TIM17->DIER = 0;         /* stop the FDT timer */
    NVIC_DisableIRQ(TIM1_TRG_COM_TIM17_IRQn);
    TIM1->CR1 = 0; TIM1->DIER = 0;                       /* stop the DMA TX timer */
    DMA1_Channel2->CCR = 0; DMA1_Channel3->CCR = 0;
    NVIC_DisableIRQ(DMA1_Channel2_IRQn);
    EXTI->PR1 = (1u << 4);
    TIM17->SR = 0; TIM1->SR = 0;
    DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3;
    NVIC_ClearPendingIRQ(EXTI4_IRQn);
    NVIC_ClearPendingIRQ(EXTI0_IRQn);
    NVIC_ClearPendingIRQ(TIM1_TRG_COM_TIM17_IRQn);
    NVIC_ClearPendingIRQ(DMA1_Channel2_IRQn);
    s_tx_armed = 0; s_tx_force_raw = 0; s_tx_dma_pending = 0; s_tx_dma_key = 0;
    s_tx_crypto = 0; s_tx_crypto_dma = 0; s_fdt_locked = 0; s_tx_spec_state = EMU_SPEC_NONE;
    s_grp_byte_ready = 0; s_grp_byte_ctx = 0; s_grp_callback_active = 0;
    emu_early_stream_cancel();
    s_emu_waiting = 0;
    s_emu_task = 0;                                      /* the EXTI/EXTI0 ISRs gate every notify on this */
    dma_cap_stop();
    EMU_MOSI_LO;
    s_hf_dirty = 1;                                      /* force a full re-init on the next set_mode(HF_EMU) */
}

/* FreeRTOS task-delete hook (wired via traceTASK_DELETE): fires for every vTaskDelete, in the deleter's
 * context with the scheduler locked. If the task being deleted is the emu recv task (abrupt kill / USB
 * disconnect - the normal exit already ran emu_teardown and NULLed s_emu_task), stop its ISRs now so no
 * dangling-handle notify can corrupt memory. Register-only work only (see emu_isr_kill). */
void rfid_emu_task_deleted(void *tcb)
{
    if (tcb && tcb == (void *)s_emu_task) emu_isr_kill();
}

static void emu_teardown(void)
{
    if (!s_emu_up && !s_emu_starting) return;
    if (xTaskGetCurrentTaskHandle() == s_emu_task)
        vTaskPrioritySet(NULL, s_emu_base_prio);        /* restore only the app task's saved priority */
    emu_isr_kill();                                     /* stop all emu ISRs/DMA + NULL s_emu_task (register-only) */
    __set_BASEPRI(0);                                   /* normal task context: release the reply-path USB mask */
    if (s_emu_clock_owned) {
        configASSERT(emu_clk_restore() == 0);           /* back to 32 MHz before SPI/reg ops resume */
        s_emu_clock_owned = false;
    }
    fz_power_tickless_block(false);
    if (s_spi_up) {
        SPI1->CR1 |= SPI_CR1_SPE;                       /* ensure SPI enabled before the teardown cmd */
        GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE5) | (2u << GPIO_MODER_MODE5_Pos);  /* SCK PA5 -> AF */
        miso_as_spi();
        mosi_as_spi();                                  /* restore MOSI to SPI AF (send leaves it a GPIO) */
        CS_PORT->BSRR = (1u << CS_PIN);
        CS_PORT->MODER = (CS_PORT->MODER & ~GPIO_MODER_MODE4) | (1u << GPIO_MODER_MODE4_Pos);
        direct_cmd(CMD_STOP);                           /* leave transparent mode */
    }
    emu_free();
    s_emu_up = false;
    s_emu_starting = false;
}

/* Target-mode NFC-A front-end for transparent emulation. It uses the envelope demodulator on PB4/MISO for
 * reader frames and PB5/MOSI for load modulation, and stays in transparent mode until teardown. */
static void emu_config(void)
{
    direct_cmd(CMD_STOP);
    direct_cmd(CMD_SET_DEFAULT);                        /* clean RESET: clear all reader-mode state from st25_init,
                                                         * so this is a from-scratch furi_hal_nfc_init, not a
                                                         * reconfigure on top of the reader chain */
    mask_all_irqs();                                    /* SET_DEFAULT restored the unmasked reset state */
    vTaskDelay(pdMS_TO_TICKS(1));
    reg_set(REG_IO_CONF2, IO_CONF2_drv_lvl);            /* io_drv_lvl (MISO/IRQ drive strength) */
    reg_set(REG_OP_CONTROL, OP_CONTROL_en);             /* turn the oscillator back on (SET_DEFAULT cleared it) */
    for (int i = 0; i < 50; i++) {
        if (reg_read(REG_AUX_DISPLAY) & AUXDISP_osc_ok) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    /* --- MEASURE_VDD -> sup3V (furi_hal_nfc_init). The oscillator is already up (st25_init). Set the supply
     * mode so the internal regulators bias the AFE for the real 3.3V rail - the working listener does this and
     * the demod does not reach PB4/MISO without a correctly biased front-end. Emu-only: the shared reader init is
     * untouched. --- */
    reg_write(REG_REGULATOR_CTRL,
              (uint8_t)((reg_read(REG_REGULATOR_CTRL) & ~REGCTRL_mpsv_mask) | REGCTRL_mpsv_vdd));
    direct_cmd(CMD_MEASURE_VDD);
    vTaskDelay(pdMS_TO_TICKS(1));
    {
        uint8_t ad = reg_read(REG_AD_RESULT);
        uint16_t mv = (uint16_t)(ad * 23U) + (uint16_t)(((ad * 4U) + 5U) / 10U);   /* furi mV formula */
        if (mv < 3600) reg_set(REG_IO_CONF2, IO_CONF2_sup3V);   /* 3V supply (Flipper) */
        else           reg_clear(REG_IO_CONF2, IO_CONF2_sup3V);
    }

    /* --- Card-emulation analog base, verbatim from stock furi_hal_nfc_init (the part my reader-only
     * st25_init omits). Without these the ST25R3916 emits no demod in target mode. --- */
    reg_write(REG_IO_CONF1, 0x07);                      /* out_c=11 + lf_clk_off: MCU_CLK off */
    reg_clear(REG_IO_CONF2, IO_CONF2_miso_pd);          /* miso_pd1|pd2 off */
    reg_set(REG_IO_CONF2, IO_CONF2_drv_lvl);            /* io_drv_lvl (MISO/IRQ drive strength) */
    reg_clear(REG_TX_DRIVER, 0x0F);                     /* d_res = 1 ohm */
    reg_set(REG_RES_AM_MOD, 0x80);                      /* fa3_f: minimum non-overlap */
    reg_write(REG_FIELD_THR_ACTV, 0x11);               /* trg_105mV | rfe_105mV (furi) - a real hysteresis band
                                                        * above DEACTV so auto-EFD has a stable arm point */
    reg_write(REG_FIELD_THR_DEACTV, 0x00);             /* trg_75mV | rfe_75mV (deactivation, furi) */
    reg_set(REG_AUX_MOD, 0x30);                         /* lm_ext | lm_dri: ENABLE load modulation */
    reg_write(REG_PASSIVE_TARGET, 0x50);               /* fdel = 5 (matches the working listener's PT=0x50 dumped
                                                        * from official fw); framing auto-anticoll is bypassed
                                                        * in transparent mode so d_106_ac_a is moot here. */
    reg_write(REG_PT_MOD, (uint8_t)s_ptmod);           /* ptm_res=0, pt_res: RFO strength when modulated (TEMP tunable) */
    reg_set(REG_EMD_SUP_CONF, 0x40);                   /* rx_start_emv: start RX on the first 4 bits */
    reg_write(REG_ANT_TUNE_A, 0x82);
    reg_write(REG_ANT_TUNE_B, 0x82);

    /* --- RX front-end = NFC-A correlator (AUX=00), the working transparent-target structure confirmed on
     * official firmware: instrumenting the stock ISO15693 listener (which emulated a card the PM3 read back)
     * dumped MODE=88, OP=C3, AUX=00 (correlator).
     * Values here are the 14443-A listener's (furi_hal_nfc_iso14443a common_init) since we receive a 14443-A
     * reader command: RX_CONF1=0x08 z600k, RX_CONF2=0x2D AGC on, CORR_CONF1=0x51, CORR_CONF2=0x00. --- */
    if (s_emu_env) {
        /* Envelope demod (sniff-proven, decodes reader Miller cleanly): the 424 kHz correlator's output sign
         * depends on its phase alignment, which our own load-mod TX kicks to a random phase -> the following
         * reader frame is received with inverted polarity. The peak/mixer AM
         * envelope has no phase-sign ambiguity (carrier-on high, pause low, always). */
        reg_write(REG_RX_CONF1, 0x03);                   /* wideband AM channel */
        reg_write(REG_RX_CONF2, 0x40);                   /* mixer AM demod, AGC off (sniff-proven) */
        reg_write(REG_RX_CONF3, (uint8_t)s_rxc3);        /* first-stage gain */
        reg_write(REG_RX_CONF4, (uint8_t)s_rxc4);
        reg_set(REG_AUX, AUX_dis_corr);                  /* envelope, bypass the correlator */
    } else {
        reg_write(REG_RX_CONF1, (uint8_t)s_rxc1);        /* HPF zero (TEMP tunable): h80/h200/z600k */
        reg_write(REG_RX_CONF2, (uint8_t)s_rxc2);        /* squelch/AGC (TEMP tunable) */
        reg_write(REG_RX_CONF3, (uint8_t)s_rxc3b);       /* first-stage gain (TEMP tunable): lower = demod stays linear during our TX */
        reg_write(REG_RX_CONF4, (uint8_t)s_rxc4);        /* 2nd/3rd-stage gain / digitizer (TEMP tunable) */
        reg_write(REG_CORR_CONF1, 0x13);                 /* corr_s0|s1|s4 */
        reg_write(REG_CORR_CONF2, 0x01);                 /* corr_s8: 424 kHz subcarrier STREAM mode ON */
        reg_clear(REG_AUX, AUX_dis_corr);                /* use the correlator (confirmed reaches the pin) */
    }
    reg_clear(REG_IO_CONF2, IO_CONF2_miso_pd);

    /* --- Transparent target, MODE = om_iso14443a | targ = 0x88 (confirmed: the working listener dumped
     * MODE=88 on official fw). tr_am_ook = tr_am cleared. OP_CONTROL = en|rx_en|en_fd_auto_efd (0xC3). The
     * merged furi code + the live register dump both use om_targ_nfca(=om_iso14443a, 0x08)|targ. --- */
    reg_clear(REG_MODE, MODE_om_mask | MODE_tr_am);
    reg_set(REG_MODE, MODE_om_iso14443a | MODE_targ);   /* 0x88 transparent target */
    reg_write(REG_STREAM, STREAM_scf_fc16 | STREAM_scp_8 | STREAM_stx_fc16);
    reg_clear(REG_OP_CONTROL, OP_CONTROL_tx_en);        /* target: no field of our own */
    reg_set(REG_OP_CONTROL, OP_CONTROL_en | OP_CONTROL_rx_en | OP_CONTROL_en_fd_auto);   /* 0xC3 */
    reg_set(REG_AUX_MOD, 0x30);                         /* lm_ext | lm_dri: keep the load-mod driver enabled */

    /* Chip IRQs were masked immediately after SET_DEFAULT so the shared PA2
     * IRQ/RFID_PULL pin remains deasserted. Transparent RX arrives on PB4/MISO. */
    /* NOTE: the working ISO15693 listener does not issue RESET_RX_GAIN / CLEAR_FIFO / UNMASK_RECEIVE before
     * transparent - RESET_RX_GAIN loads manual gain, which fights the AGC (agc_en in RXC2). Match the listener:
     * config, then CMD_TRANSPARENT_MODE + bus release (in emu_setup), nothing else. */

    /* Re-calibrate the regulators for the sup3V + full analog config now in place (furi does the reg_s
     * set/clear reset before ADJUST_REGULATORS, at the end of init). st25_init's earlier ADJUST ran with the
     * default supply mode; redo it here so the regulated rails match this front-end. */
    reg_set(REG_REGULATOR_CTRL, REGCTRL_reg_s);
    reg_clear(REG_REGULATOR_CTRL, REGCTRL_reg_s);
    direct_cmd(CMD_ADJUST_REGULATORS);
    vTaskDelay(pdMS_TO_TICKS(6));

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;     /* DWT cycle counter for the FDT delay */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* MISO-edge WAKE for recv. The demod is still decoded from the jitter-free level-DMA (unchanged) and the FDT
 * reference is still the accurate atomic DMA back-date - EXTI line 4 (PB4/MISO) is used only to wake recv from
 * its idle sleep on a frame's first edge, so it looks within ~1 us instead of up to 1 ms late (vTaskDelay(1)).
 * A 1 ms idle-sleep lag would make the reply land ~850 us after the frame end, far past any
 * FDT. Wake-only: no edge timestamps, no resync. */
void EXTI4_IRQHandler(void)
{
    s_last_edge_cyc = DWT->CYCCNT;                  /* first, before anything: the carrier-referenced phase anchor.
                                                    * At priority 4 (above USB's 5) nothing preempts this read, so
                                                    * the timestamp carries only the fixed HW entry latency and the
                                                    * deterministic reply start (emu_tx_raw) tracks the reader grid. */
    EXTI->PR1 = (1u << 4);                          /* clear the line-4 pending flag */
    if (s_edge_n < 24) s_edge_dwt[s_edge_n++] = s_last_edge_cyc;
    if (!s_fdt_locked && s_tx_spec_state == EMU_SPEC_ARMED) {
        s_tx_armed = 0;
        s_tx_spec_state = EMU_SPEC_NONE;
    }
    /* HW-timer FDT: (re)arm TIM17 to fire ~FDT after this edge. Intra-frame edges keep resetting it (all < FDT
     * apart), so it only fires once the frame has ended. OPM auto-stops after firing; CNT=0 + CEN restarts it.
     * But once a frame has ended and a reply is staged (s_fdt_locked=1), do not reset it here: post-frame demod
     * ringing / spurious MISO edges would otherwise keep restarting the FDT count, so the reply (especially the
     * re-triggered crypto at/READ reply) would fire ~FDT after the last noise edge instead of after the real
     * frame. recv clears s_fdt_locked per new frame. */
    if (!s_fdt_locked) {
        TIM17->CR1 &= ~TIM_CR1_CEN;
        TIM17->SR = ~TIM_SR_UIF;
        NVIC_ClearPendingIRQ(TIM1_TRG_COM_TIM17_IRQn);
        TIM17->ARR = s_fdt_arr;
        TIM17->CNT = 0;
        TIM17->CR1 |= TIM_CR1_CEN;
    }
    /* Defer the recv WAKE to EXTI0 (pri 5, syscall-safe): this handler runs above the FreeRTOS ceiling so it must
     * not call vTaskNotifyGiveFromISR here. NVIC_SetPendingIRQ runs EXTI0 the moment this returns. */
    if (s_emu_task && s_emu_waiting) NVIC_SetPendingIRQ(EXTI0_IRQn);
}

/* Deferred recv-wake, pended by EXTI4. Priority 5 (== FreeRTOS syscall ceiling), so the notify is legal here.
 * Not wired to any real EXTI line - only ever run via NVIC_SetPendingIRQ from EXTI4. */
void EXTI0_IRQHandler(void)
{
    if (s_emu_task && s_emu_waiting) {
        s_emu_waiting = 0;
        BaseType_t hp = pdFALSE;
        vTaskNotifyGiveFromISR(s_emu_task, &hp);
        portYIELD_FROM_ISR(hp);
    }
}

/* Release the SPI bus so the ST25R3916 drives its transparent-mode demod onto PB4/MISO without the STM32
 * peripheral contending. SCK(PA5) and MISO(PB4) become inputs and SPE is disabled; MOSI(PB5) remains the
 * load-modulation output. No SPI operation may run until emu_teardown re-acquires the bus. */
static void emu_bus_release(void)
{
    SPI1->CR1 &= ~SPI_CR1_SPE;                         /* disable the SPI peripheral */
    /* Exact furi transparent-mode pin config (furi_hal_spi_bus_nfc_handle_event_callback Deinit):
     * SCK, MISO, and CS all -> input + pull-up. MOSI stays a GPIO output low (caller). */
    GPIOA->MODER &= ~GPIO_MODER_MODE5;                 /* SCK PA5 -> input */
    GPIOA->PUPDR = (GPIOA->PUPDR & ~GPIO_PUPDR_PUPD5) | (1u << GPIO_PUPDR_PUPD5_Pos);   /* pull-up */
    GPIOB->MODER &= ~GPIO_MODER_MODE4;                 /* MISO PB4 -> input */
    GPIOB->PUPDR = (GPIOB->PUPDR & ~GPIO_PUPDR_PUPD4) | (1u << GPIO_PUPDR_PUPD4_Pos);   /* pull-up */
    GPIOE->MODER &= ~GPIO_MODER_MODE4;                 /* CS PE4 -> input (released, not driven) */
    GPIOE->PUPDR = (GPIOE->PUPDR & ~GPIO_PUPDR_PUPD4) | (1u << GPIO_PUPDR_PUPD4_Pos);   /* pull-up */
}

/* Boost SYSCLK from the 32 MHz HSE to 64 MHz (HSE /2 *8 /2 through the main PLL) for the FDT-critical emu path.
 * CPU2 shares SYSCLK on STM32WB, so when it is running it performs both clock transitions through SHCI. */
#define EMU_HSEM_COREID_CPU1 0x04u
#define EMU_HSEM_RCC_SEM     3

static void emu_rcc_lock(void)
{
    RCC->AHB3ENR |= RCC_AHB3ENR_HSEMEN;
    (void)RCC->AHB3ENR;
    while (HSEM->RLR[EMU_HSEM_RCC_SEM] !=
           (HSEM_RLR_LOCK_Msk | (EMU_HSEM_COREID_CPU1 << HSEM_RLR_COREID_Pos))) { }
}

static void emu_rcc_unlock(void)
{
    HSEM->R[EMU_HSEM_RCC_SEM] = EMU_HSEM_COREID_CPU1 << HSEM_R_COREID_Pos;
}

static int emu_clk_boost(void)
{
    fz_power_tickless_block(true);
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_3WS;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_3WS) { }

    emu_rcc_lock();
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) { }
    RCC->PLLCFGR = (3U << RCC_PLLCFGR_PLLSRC_Pos)      /* src = HSE 32 MHz */
                 | (1U << RCC_PLLCFGR_PLLM_Pos)        /* M = /2  -> 16 MHz PLL input */
                 | (8U << RCC_PLLCFGR_PLLN_Pos)        /* N = x8  -> 128 MHz VCO */
                 | (1U << RCC_PLLCFGR_PLLR_Pos)        /* R = /2  -> 64 MHz SYSCLK */
                 | RCC_PLLCFGR_PLLREN;
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0) { }
    emu_rcc_unlock();

    if (ble_cpu2_running()) {
        /* Wireless stacks before 1.20 can return an invalid status for this
         * command. The RCC status below is authoritative. */
        (void)ble_cpu2_set_system_clock(BLE_SYSCLK_HSE_TO_PLL);
    } else {
        emu_rcc_lock();
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | (3U << RCC_CFGR_SW_Pos);       /* SYSCLK <- PLL */
        emu_rcc_unlock();
    }
    if (((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos) != 3U) return -1;
    SystemCoreClock = 64000000u;
    SysTick->LOAD = (64000000u / configTICK_RATE_HZ) - 1u;   /* keep the 1 kHz tick (tickless off) */
    SysTick->VAL  = 0;
    s_cap_arr = 25u;                                  /* 64e6/(25+1) = 2.462 MHz sample rate (same as 32 MHz path) */
    return 0;
}

/* Restore SYSCLK to the 32 MHz HSE (undo emu_clk_boost) when emulation ends. */
static int emu_clk_restore(void)
{
    if (((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos) == 3U) {
        if (ble_cpu2_running()) {
            /* Keep PLL running until CPU1 has verified that CPU2 selected HSE. */
            (void)ble_cpu2_set_system_clock(BLE_SYSCLK_PLL_ON_TO_HSE);
        } else {
            emu_rcc_lock();
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | (2U << RCC_CFGR_SW_Pos);   /* SYSCLK <- HSE */
            emu_rcc_unlock();
        }
    }
    if (((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos) != 2U) return -1;
    if (RCC->CR & RCC_CR_PLLON) {
        emu_rcc_lock();
        RCC->CR &= ~RCC_CR_PLLON;
        while (RCC->CR & RCC_CR_PLLRDY) { }
        emu_rcc_unlock();
    }
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_1WS;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_1WS) { }
    SystemCoreClock = 32000000u;
    SysTick->LOAD = (32000000u / configTICK_RATE_HZ) - 1u;
    SysTick->VAL  = 0;
    s_cap_arr = CAP_ARR;
    fz_power_tickless_block(false);
    return 0;
}

static int emu_setup(void)
{
    s_emu_base_prio = uxTaskPriorityGet(NULL);
    s_emu_task = xTaskGetCurrentTaskHandle();
    s_emu_waiting = 0;
    s_emu_starting = true;
    s_emu_clock_owned = false;
    s_edge_n = 0; s_tx_end_cyc = 0; s_tx_crypto = 0; s_tx_crypto_dma = 0;
    s_tx_spec_state = EMU_SPEC_NONE;
    s_tx_force_raw = 0; s_tx_dma_pending = 0; s_tx_dma_key = 0;
    for (int e = 0; e < SEND_CACHE_N; e++) s_send_cache[e].ptr = 0;
    s_emu_ring = pvPortMalloc(EMU_RING);
    s_tr       = pvPortMalloc(EMU_MAXTR * sizeof *s_tr);
    s_seq      = pvPortMalloc(160);
    if (!s_emu_ring || !s_tr || !s_seq) {
        emu_free(); s_emu_task = 0; s_emu_starting = false;
        return -1;
    }

    emu_config();                                       /* transparent target using the PB4 envelope demodulator */
    direct_cmd(CMD_TRANSPARENT_MODE);                   /* enter transparent: chip's raw AFE drives the pins */
    mosi_as_output_low();                               /* PB5 = load-mod gate, idle low (for TX) */
    emu_bus_release();                                  /* furi transparent pin config: SCK/MISO/CS input+pullup */

    /* Capture the 14443-A OOK demod on MISO (PB4) with the free-running 2.462 MHz level-DMA - the same
     * jitter-free path the reader-passive sniff uses to decode WUPA. (The demod is on MISO only after the clean
     * full init in emu_config; before that it never reached any pin.) MISO is input+pullup from emu_bus_release. */
    s_cap_idr = &GPIOB->IDR; s_cap_shift = MISO_PIN;
    vTaskDelay(pdMS_TO_TICKS(1));                       /* settle after transparent entry (as the sniff does) */
    /* Boost to 64 MHz now - after all the SPI/reg config (which ran at 32 MHz) and before the capture timer is
     * armed, so TIM2 (s_cap_arr=25) and TIM17 come up on the 64 MHz clock. All emu DWT constants are 64 MHz. */
    s_emu_clock_owned = true;
    if (emu_clk_boost() != 0) return -1;
    dma_circ_arm(s_emu_ring, EMU_RING);
    dma_cap_go();                                       /* free-running capture across recv/send */

    /* EXTI line 4 <- PB4 (MISO), both edges: the recv-idle WAKE (see EXTI4_IRQHandler). Armed continuously;
     * recv blocks on the task-notify instead of vTaskDelay(1) so it looks within ~1 us of a frame's first edge.
     * SYSCFG is already clocked (furi uses GPIO interrupts); the level-DMA reads the same pin independently. */
    s_emu_task = xTaskGetCurrentTaskHandle();
    SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] & ~(0xFu)) | (0x1u);   /* EXTI4 source = port B */
    EXTI->RTSR1 |= (1u << 4);
    EXTI->FTSR1 |= (1u << 4);
    EXTI->PR1   =  (1u << 4);
    EXTI->IMR1 |=  (1u << 4);
    NVIC_SetPriority(EXTI4_IRQn, 3);               /* a new reader edge cancels/restarts a pending FDT before TIM17 */
    NVIC_ClearPendingIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_SetPriority(EXTI0_IRQn, 5);               /* deferred recv-wake (syscall-safe); pended by EXTI4 only */
    NVIC_ClearPendingIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI0_IRQn);

    /* HW-timer FDT (TIM17): one-pulse at 64 MHz, fires shortly before the reply time after the last MISO edge.
     * URS=1 so the per-edge software re-arm (CNT=0) never spuriously raises the update IRQ -
     * only a real overflow (frame end + FDT) does. Free during HF emu: TIM1 is LF-only, TIM2 drives the capture. */
    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
    (void)RCC->APB2ENR;
    TIM17->CR1  = TIM_CR1_OPM | TIM_CR1_URS;            /* one-pulse; only overflow generates update event */
    TIM17->PSC  = 0;                                    /* 64 MHz tick */
    TIM17->ARR  = s_fdt_arr;                             /* fire shortly before the calibrated transmit point */
    TIM17->EGR  = TIM_EGR_UG;                           /* load PSC/ARR (URS -> no IRQ) */
    TIM17->SR   = 0;
    TIM17->DIER = TIM_DIER_UIE;                         /* update interrupt */
    NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, 4);       /* above USB (5): the FDT fire (reply TX) preempts a USB
                                                         * ISR in flight instead of waiting for it, so the reply's
                                                         * subcarrier start doesn't jitter against the reader's grid.
                                                         * Safe at pri 4 - this ISR makes no FreeRTOS syscall. */
    NVIC_ClearPendingIRQ(TIM1_TRG_COM_TIM17_IRQn);
    NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
    s_tx_armed = 0; s_fdt_locked = 0;

    emu_tx_pat_init();        /* precompute the per-half-bit edge patterns */
    emu_tx_dma_setup();
    s_fdt_arr = EMU_FDT_ARR;   /* enter early enough to program DMA, then phase-pin the first edge exactly */
    if (s_use_dma_tx) emu_edge_cache_alloc();   /* build DMA edges once/reply, pointer-swap on repeats (build-free hot path) */

    s_emu_up = true;
    s_emu_starting = false;
    return 0;
}

/* Software carrier recovery (workaround for no MCU_CLK). The reply's subcarrier is CPU-timed and must land in the
 * PM3 FPGA's narrow carrier-locked detection window; a single last-edge timestamp carries ~+-10 cyc of demod+ISR
 * jitter, wider than the window, so the reply phase scatters and only ~60% of frames decode (bimodal 2368/192).
 * The reader's modulation edges are all carrier-referenced, and same-type edges (pause ends) sit at INTEGER
 * subcarrier-period (fc/16 = 75.5 cyc) spacing. So the sub-period deviation of each same-type edge from the last
 * edge is that edge's jitter minus the last edge's jitter; averaging them yields -jitter(last_edge), i.e. it
 * cancels the last edge's individual jitter and returns the true carrier-referenced instant. */
__attribute__((optimize("O2")))
static uint32_t carrier_lock(uint32_t last_edge)
{
    if (!s_carrier_en) return last_edge;   /* runtime toggle (poke via SWD) for interleaved A-B-A */
    if (s_edge_n < 6) return last_edge;
    /* Integer-only (FDT-critical path): work in HALF-cycles so the fc/16 period 75.5 -> 151 is an integer modulus. */
    int sum = 0, cnt = 0;
    for (int i = 0; i < s_edge_n && i < 24; i++) {
        int m = (int)(2 * (int32_t)(s_edge_dwt[i] - last_edge)) % 151;   /* 2*deviation, mod 2*period */
        if (m < 0) m += 151;
        if (m > 75) m -= 151;                          /* center: -75..75 half-cyc */
        if (m > -36 && m < 36) { sum += m; cnt++; }    /* same-type (pause-end) edges cluster near 0 (+-18 cyc) */
    }
    if (cnt < 4) return last_edge;
    return last_edge + (int32_t)((sum / cnt) / 2);     /* half-cyc -> cyc, jitter-averaged */
}

static int emu_fdt_anchor_lock(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    if (EXTI->PR1 & (1u << 4)) {
        if (!pm) __enable_irq();
        return 0;
    }
    uint32_t last_edge = s_last_edge_cyc;
    uint32_t rx_cyc = carrier_lock(last_edge);
    if (EXTI->PR1 & (1u << 4)) {
        if (!pm) __enable_irq();
        return 0;
    }
    s_emu_rx_cyc = rx_cyc;
    s_fdt_locked = 1;
    s_edge_n = 0;
    if (!pm) __enable_irq();
    return 1;
}

/* Receive one reader command by capturing the transparent-mode demod off MISO and Miller-decoding it
 * (reusing the sniff DSP). Scans the free-running ring for a command framed by >= FRAME_GAP of trailing
 * silence, records the frame-end DWT (the FDT reference), decodes into rx[]/rx_par[]. rx_par[i] is the
 * raw received parity bit (a Crypto1 frame legitimately violates odd parity). Returns byte count, 0 idle. */
static int emu_recv(uint8_t *rx, uint8_t *rx_par, int cap, uint32_t timeout_ms)
{
    if (s_mode != RFID_HF_EMU || !rx || cap <= 0 || !s_emu_up) return RFID_ERR_UNSUPP;
    emu_early_stream_cancel();
    __set_BASEPRI(0);   /* clear any reply-staging USB mask left by a prior command that didn't reply (see emu_tx_arm) */
    if (!s_tx_armed)
        s_fdt_locked = 0;   /* a new receive call confirms the prior command will not stage a reply */
    /* Run + block at high priority so the EXTI notify schedules us the instant a frame's first edge arrives
     * (at base priority the USB task, idle+2, runs first and the frame buffers, adding detection lag that
     * blows the FDT). The idle block still yields the CPU (USB runs while we're blocked), and recv returns
     * at the gap, so the high-prio window is one frame (~170 us), not a continuous spin - USB keeps ~97%. */
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
    uint32_t wp0 = EMU_RING - DMA1_Channel1->CNDTR;
    s_ntr = 0;
    miller_inc_reset();                                 /* pipelined decoder: fresh grid for this frame */
    s_lvl = (s_emu_ring[wp0 % EMU_RING] >> s_cap_shift) & 1; s_tr_init = s_lvl;
    int pos = 0;
    uint32_t read_abs = wp0, wr_hi = 0, last_wpos = wp0;
    TickType_t t0 = xTaskGetTickCount();
    uint32_t active_until = DWT->CYCCNT;    /* busy-poll (no block) until this DWT time; 0-length => block now */
    uint32_t active_open_since = DWT->CYCCNT;   /* DWT when the active window last OPENED; a real frame decodes and
                                                * lapses within a few ms, so "open >5 ms" == noise (not a live scan) */
    uint32_t wake_grace_until = DWT->CYCCNT;

    for (;;) {
        /* Reader-gone stop-key yield (HAL-side, so it can manage BASEPRI - the module can't). While a reader is
         * live this loop runs at pri-4 and masks USB during frames (FDT-critical), starving usbproto (the host
         * app_stop), the app pump (kill_req) and tud_task -> a `-c`/CDC/^C stop looks dead and the device stalls.
         * Once no real frame has decoded for >200 ms, wait here at base on the EXTI notify: usbproto/pump/tud run
         * (stop serviced) yet a real edge still wakes us (unlike a time-based delay, which would miss the WUPA);
         * we boost back and resync the scan window so the ensuing detect finds the reader's frame. The active-window
         * guard means we never do this mid-scan (would corrupt a frame in progress). A decoded frame refreshes
         * s_last_frame_tick and closes the gate -> live reads run entirely at pri-4, FDT untouched. */
        if ((int32_t)(DWT->CYCCNT - active_until) >= 0) active_open_since = DWT->CYCCNT;   /* window closed: reset */
        /* Two independent triggers for the stop-key yield:
         *  - Noise-stuck: the active window has been continuously open >8 ms. A real reader command scan is always
         *    <~4 ms (longest is an 18-byte frame), so >8 ms means the demod is free-running on noise, not a live
         *    frame. This fires regardless of s_last_frame_tick because, after the reader's field has been on, the
         *    demod decodes noise as bogus frames that keep refreshing that tick - so the reader-gone gate below
         *    stays falsely closed and cannot be trusted.
         *  - Reader-gone + idle: truly quiet (window lapsed) and no real frame for >200 ms (covers a clean idle
         *    with no noise). During a live exchange this is false (tick fresh) and the window lapses <8 ms between
         *    commands, so neither trigger fires -> the exchange (and the READ FDT) is untouched. */
        int emu_idle       = (int32_t)(DWT->CYCCNT - active_until) >= 0;
        int emu_noisestuck = (int32_t)(DWT->CYCCNT - active_open_since) > (int32_t)(EMU_NOISE_US * 64u);
        int emu_rdrgone    = (xTaskGetTickCount() - s_last_frame_tick) > pdMS_TO_TICKS(200);
        if (emu_noisestuck || (emu_rdrgone && emu_idle)) {
            s_edge_n = 0;                              /* discard phase samples from the abandoned window */
            __set_BASEPRI(0);
            if (emu_noisestuck) vTaskPrioritySet(NULL, s_emu_base_prio);
            s_emu_waiting = 1;
            __DMB();
            uint32_t woke = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
            s_emu_waiting = 0;
            if (emu_noisestuck) vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
            if (woke) {
                uint32_t wpr = EMU_RING - DMA1_Channel1->CNDTR;
                wr_hi = (wpr < EMU_WAKE_LOOKBACK) ? EMU_RING : 0;
                last_wpos = wpr;
                read_abs = wr_hi + wpr - EMU_WAKE_LOOKBACK;
                pos = 0; s_ntr = 0;
                miller_inc_reset();
                s_lvl = (s_emu_ring[read_abs % EMU_RING] >> s_cap_shift) & 1; s_tr_init = s_lvl;
                active_until = DWT->CYCCNT + EMU_ACTIVE_US * 64u;
                wake_grace_until = DWT->CYCCNT + EMU_WAKE_GRACE_US * 64u;
            } else {
                active_until = DWT->CYCCNT;
            }
        }
        /* USB-preemption guard for the active-frame window: while we're busy-polling a frame in (edges flowing,
         * active_until still in the future) mask the USB ISR (pri 5) so neither the gap detection drain nor the
         * reply dispatch is preempted (that preemption jitter would push the UID/SAK turnaround past the FDT).
         * The capture DMA and the reply ISRs (EXTI4/TIM17/DMA @ pri 4) stay
         * live. Cleared the instant the active window lapses, which is before the idle ulTaskNotifyTake block below:
         * we must never hold BASEPRI across a FreeRTOS block (the ARM_CM4F port doesn't save/restore BASEPRI per
         * task, so a held mask would stall USB for the entire idle wait). arm clears it on the reply path. */
        if (s_use_dma_tx)
            __set_BASEPRI((int32_t)(DWT->CYCCNT - active_until) < 0 ? (uint32_t)EMU_MASK_USB : 0u);
        if ((xTaskGetTickCount() - t0) >= pdMS_TO_TICKS(timeout_ms ? timeout_ms : 100) &&
            (int32_t)(DWT->CYCCNT - wake_grace_until) >= 0) {
            __set_BASEPRI(0);
            vTaskPrioritySet(NULL, s_emu_base_prio);    /* must drop to base: the app polls its stop key at our
                                                         * priority, and the lower-priority CLI task can't deliver
                                                         * the keystroke if we return still boosted -> the app
                                                         * never stops -> clean exit fails -> USB stranded. */
            emu_early_stream_cancel();
            return 0;                                   /* timeout: return so the app can poll its stop key */
        }

        uint32_t wp = EMU_RING - DMA1_Channel1->CNDTR;
        if (wp < last_wpos) wr_hi += EMU_RING;
        last_wpos = wp;
        /* While our own DMA reply is on the field, the demod feeds it back onto MISO - skip those samples so we
         * don't decode our reply as a bogus reader command. Keep read_abs at the head and reset the frame state. */
        int32_t txrem = (int32_t)(DWT->CYCCNT - s_tx_end_cyc);
        if (s_txskip && ((TIM1->CR1 & TIM_CR1_CEN) ||
                         (txrem < 0 && txrem > -20000))) {
            s_edge_n = 0;                                /* discard self-TX/ringing edges before carrier_lock */
            read_abs = wr_hi + wp; s_ntr = 0; pos = 0; miller_inc_reset();
            s_lvl = (s_emu_ring[(wr_hi + wp) % EMU_RING] >> s_cap_shift) & 1; s_tr_init = s_lvl;
            active_until = DWT->CYCCNT; continue;
        }
        uint32_t backlog = (wr_hi + wp) - read_abs;

        /* Hybrid real-time frame-end: the EXTI ISR timestamps every MISO edge into s_last_edge_cyc, so when no
         * edge has arrived for FRAME_GAP the frame ended ~33 us ago - known immediately, not after the level-DMA
         * scan slowly reaches the gap. On this trigger, scan all backlog
         * in one pass to catch pos up through the trailing silence so the decode below fires this iteration. */
        /* Gap threshold in DWT cycles. EMU_GAP=52 samples (2.24 etu) is just above the 2-etu max spacing between
         * Miller pauses within a frame, so it won't false-fire mid-frame yet cuts the wait vs FRAME_GAP=81. */
        int gap_by_time = (s_ntr >= 4) &&
                          ((int32_t)(DWT->CYCCNT - s_last_edge_cyc) >= (int32_t)(52u * EMU_SAMP_CYC));

        /* The edge timer can report frame end before the polling loop has
         * consumed the capture DMA's final samples. Drain that stable tail
         * before grouping bytes, or the last encrypted AUTH byte is lost. */
        if (gap_by_time && backlog) {
            uint32_t take = backlog > 4096u ? 4096u : backlog;
            uint32_t rp = read_abs % EMU_RING, first = EMU_RING - rp;
            if (first > take) first = take;
            scan_span(s_emu_ring + rp, (int)first, &pos);
            if (take > first) scan_span(s_emu_ring, (int)(take - first), &pos);
            read_abs += take;
            miller_inc_feed();
        }

        /* Detect + decode first, before the idle/scan below. s_tr already holds the whole frame (the during-frame
         * scans filled it in the ~52 samples before the gap), so on gap_by_time we go straight to decode+reply -
         * not scanning the trailing silence (that would delay gap detection and inflate `ta`). Reference is the ISR
         * last-edge time = deterministic frame end. */
        if (gap_by_time) {
            /* The per-pause grid placement was already done incrementally during reception (miller_inc_feed
             * after each scan), so all that remains on the FDT-critical path is the cheap byte-grouping -
             * no fast-path needed and multi-byte replies now fit the FDT window. */
            if (s_mp_np >= 4) {
                /* Reference = the deterministic hardware last-edge time from the EXTI ISR (no scan-lag jitter). */
                if (!emu_fdt_anchor_lock()) continue;
                uint32_t par = 0;
                int m = miller_inc_finish(rx, &par);
                int early_committed = 0;
                if (m > 0) {
                    int short_frame = s_grp_n == 0 && s_grp_nb >= 6 && s_grp_nb <= 7;
                    int last_bit = short_frame ? ((rx[0] >> 6) & 1)
                                               : (((par >> (m - 1)) & 1) ? __builtin_parity(rx[m - 1])
                                                                         : !__builtin_parity(rx[m - 1]));
                    s_fdt_ticks = last_bit ? EMU_FDT_1 : EMU_FDT_0;
                    if (emu_early_stream_pending()) {
                        if (s_use_dma_tx) __set_BASEPRI(EMU_MASK_USB);
                        early_committed = emu_early_stream_commit(m);
                    }
                }
                if (m > 0) {
                    s_last_frame_tick = xTaskGetTickCount();   /* reader live: keep the stop-key yield disengaged */
                    if (rx_par)
                        for (int i = 0; i < m && i < cap; i++)
                            rx_par[i] = (uint8_t)(((par >> i) & 1) ? __builtin_parity(rx[i])
                                                                   : !__builtin_parity(rx[i]));
                    if (!early_committed && s_use_dma_tx)
                        __set_BASEPRI(EMU_MASK_USB);   /* keep ordinary module dispatch through reply staging */
                    s_ntr = 0; pos = 0; miller_inc_reset();  /* ready for the next frame */
                    return m;
                }
            }
            s_fdt_locked = 0;
            s_edge_n = 0;
            s_ntr = 0; pos = 0; miller_inc_reset();          /* gap fired, no valid frame: reset, keep listening */
            read_abs = wr_hi + wp;
            continue;
        }

        if (backlog < 8 && !gap_by_time) {
            /* Within an active window (a frame is arriving): busy-poll, don't block - blocking per edge thrashes
             * ~50 context switches/frame and puts the reply ~300 us late. Otherwise idle: drop to base so USB/
             * watchdog run, block until the frame's first edge wakes us via EXTI (~1 us) or a 2 ms stop-key cap,
             * then boost + open an active window so the rest of the frame is scanned uninterrupted. */
            if ((int32_t)(DWT->CYCCNT - active_until) < 0) { continue; }   /* active: tight spin, no yield -
                                                         * taskYIELD would hand off to furi timer/USB and add
                                                         * 100s-of-us to `notice`; bounded by active_until */
            /* Idle: block at high priority (no drop-to-base) so the EXTI notify preempts USB and schedules us
             * within ~1 us of the frame's first edge. The block yields the CPU while we wait, so USB still runs.
             * (The stop key is serviced by the reader-gone edge-woken base wait at the top of this loop.) */
            s_edge_n = 0;                                 /* next EXTI edge starts a fresh carrier phase sample */
            __set_BASEPRI(0);
            s_emu_waiting = 1;
            __DMB();
            uint32_t woke = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));
            s_emu_waiting = 0;
            if (!woke) {
                active_until = DWT->CYCCNT;
                continue;
            }
            active_until = DWT->CYCCNT + EMU_ACTIVE_US * 64u;
            wake_grace_until = DWT->CYCCNT + EMU_WAKE_GRACE_US * 64u;
            /* Resync the scan window to a small lookback before now: while we slept the DMA filled the ring, so
             * scanning from the stale read_abs would chew through 100s of samples of pre-frame silence before
             * reaching the frame. Start ~200 samples before the
             * current write pos (covers the wake latency to catch the frame's first edge) and reset the decoder.
             * The atomic FDT reference stays valid because pos and s_tr[] are relative to read_abs. */
            uint32_t wpr = EMU_RING - DMA1_Channel1->CNDTR;
            wr_hi = (wpr < EMU_WAKE_LOOKBACK) ? EMU_RING : 0;
            last_wpos = wpr;
            read_abs = wr_hi + wpr - EMU_WAKE_LOOKBACK;
            /* Skip any leading low baseline (the demod droop our own load-mod leaves behind): anchor the frame on
             * the carrier-ON level so s_tr_init = 1. A stray low first sample here flips the pause-parity of the
             * whole following frame - the reader's short (~6-sample) Miller pauses get read as the long carrier-on
             * gaps between them. Cap the skip so a fully
             * quiet window can't run us into the frame body. */
            pos = 0; s_ntr = 0;
            miller_inc_reset();                          /* re-scanning from the lookback: rebuild the grid too */
            s_lvl = (s_emu_ring[read_abs % EMU_RING] >> s_cap_shift) & 1; s_tr_init = s_lvl;
            continue;
        }
        active_until = DWT->CYCCNT + EMU_ACTIVE_US * 64u;   /* samples flowing: stay in the active window */
        if (backlog > 9 * EMU_RING / 10) {              /* fell behind: skip to head, drop the frame */
            s_edge_n = 0;
            read_abs = wr_hi + wp; s_ntr = 0; pos = 0; miller_inc_reset();
            s_lvl = (s_emu_ring[read_abs % EMU_RING] >> s_cap_shift) & 1; s_tr_init = s_lvl;
            continue;
        }
        /* Hold back a short DMA tail while the frame is active. Refresh the
         * producer after each bounded quantum so samples captured during
         * scan/feed are consumed without another full outer-loop pass. */
        uint32_t budget = gap_by_time ? 0u : 2048u;
        while (budget) {
            uint32_t wp_scan = EMU_RING - DMA1_Channel1->CNDTR;
            if (wp_scan < last_wpos) wr_hi += EMU_RING;
            last_wpos = wp_scan;
            uint32_t live = (wr_hi + wp_scan) - read_abs;
            if (live <= 16u) break;
            uint32_t take = live - 16u;
            if (take > 128u) take = 128u;
            if (take > budget) take = budget;
            int full_quantum = take == 128u;
            uint32_t rp = read_abs % EMU_RING, first = EMU_RING - rp;
            if (first > take) first = take;
            scan_span(s_emu_ring + rp, (int)first, &pos);
            if (take > first) scan_span(s_emu_ring, (int)(take - first), &pos);
            read_abs += take;
            miller_inc_feed();                           /* PIPELINE: fold the new pauses into the grid now, so
                                                          * frame-end only needs the byte-grouping (fits the FDT) */
            budget -= take;
            if (s_mp_np >= 4 &&
                (int32_t)(DWT->CYCCNT - s_last_edge_cyc) >= (int32_t)(52u * EMU_SAMP_CYC))
                break;
            if (!full_quantum) break;
        }

        /* Re-check the gap right here (not only at the loop top): a frame-processing iteration is ~1700 cyc, so
         * deferring the gap-fire to the next top-of-loop adds a whole iteration to `det`.
         * With the pipeline the grid is already built, so this is just the cheap byte-grouping + reply. */
        if (s_mp_np >= 4 && (int32_t)(DWT->CYCCNT - s_last_edge_cyc) >= (int32_t)(52u * EMU_SAMP_CYC)) {
            /* scan/feed may span the final reader edges. Refresh the DMA head
             * before the final drain; the loop-top snapshot can be stale. */
            uint32_t wp_tail = EMU_RING - DMA1_Channel1->CNDTR;
            if (wp_tail < last_wpos) wr_hi += EMU_RING;
            last_wpos = wp_tail;
            uint32_t tail = (wr_hi + wp_tail) - read_abs;
            if (tail) {
                uint32_t rp = read_abs % EMU_RING, first = EMU_RING - rp;
                if (first > tail) first = tail;
                scan_span(s_emu_ring + rp, (int)first, &pos);
                if (tail > first) scan_span(s_emu_ring, (int)(tail - first), &pos);
                read_abs += tail;
                miller_inc_feed();
            }
            if (!emu_fdt_anchor_lock()) continue;
            uint32_t par = 0;
            int m = miller_inc_finish(rx, &par);
            int early_committed = 0;
            if (m > 0) {
                int short_frame = s_grp_n == 0 && s_grp_nb >= 6 && s_grp_nb <= 7;
                int last_bit = short_frame ? ((rx[0] >> 6) & 1)
                                           : (((par >> (m - 1)) & 1) ? __builtin_parity(rx[m - 1])
                                                                     : !__builtin_parity(rx[m - 1]));
                s_fdt_ticks = last_bit ? EMU_FDT_1 : EMU_FDT_0;
                if (emu_early_stream_pending()) {
                    if (s_use_dma_tx) __set_BASEPRI(EMU_MASK_USB);
                    early_committed = emu_early_stream_commit(m);
                }
            }
            if (m > 0) {
                s_last_frame_tick = xTaskGetTickCount();   /* reader live: keep the stop-key yield disengaged */
                if (rx_par)
                    for (int i = 0; i < m && i < cap; i++)
                        rx_par[i] = (uint8_t)(((par >> i) & 1) ? __builtin_parity(rx[i]) : !__builtin_parity(rx[i]));
                if (!early_committed && s_use_dma_tx)
                    __set_BASEPRI(EMU_MASK_USB);   /* keep ordinary module dispatch through reply staging */
                s_ntr = 0; pos = 0; miller_inc_reset();
                return m;
            }
            s_fdt_locked = 0;
            s_edge_n = 0;
            s_ntr = 0; pos = 0; miller_inc_reset();
            read_abs = wr_hi + wp_tail;
        }

    }
}

int hal_rfid_hf_emu_recv(uint8_t *rx, uint8_t *rx_par, int cap, uint32_t timeout_ms)
{
    s_grp_byte_ready = 0;
    s_grp_byte_ctx = 0;
    return emu_recv(rx, rx_par, cap, timeout_ms);
}

int hal_rfid_hf_emu_recv_progress(uint8_t *rx, uint8_t *rx_par, int cap, uint32_t timeout_ms,
                                  void (*data_ready)(void *ctx, int index, uint8_t raw, int bits), void *ctx)
{
    s_grp_byte_ready = data_ready;
    s_grp_byte_ctx = ctx;
    int rc = emu_recv(rx, rx_par, cap, timeout_ms);
    s_grp_byte_ready = 0;
    s_grp_byte_ctx = 0;
    return rc;
}

/* In transparent mode PB5/MOSI is a GPIO load-modulation gate. It returns to the SPI alternate function only
 * during teardown. */
static void mosi_as_output_low(void)
{ EMU_MOSI_LO; GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE5) | (1u << GPIO_MODER_MODE5_Pos); }
static void mosi_as_spi(void)
{ GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE5) | (2u << GPIO_MODER_MODE5_Pos); }

/* In transparent mode SCLK is the ST25R3916 receiver enable. CS remains high,
 * so changing SCLK here cannot clock an SPI command or leave transparent mode. */
static void emu_rx_disable(void)
{
    GPIOA->BSRR = (1u << (5 + 16));
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE5) | (1u << GPIO_MODER_MODE5_Pos);
}

static void emu_rx_enable(void)
{
    GPIOA->BSRR = (1u << 5);
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE5) | (1u << GPIO_MODER_MODE5_Pos);
}

/* Emit one half-bit on MOSI: `on` = load-modulate an 847 kHz subcarrier (4 periods), else idle low. `*t`
 * is the running DWT deadline - each edge busy-waits to it so the timing self-corrects. */
static void emu_tx_half(uint32_t *t, int on, unsigned *frac)
{
    *frac += EMU_HALF_FRAC_NUM;
    int extra = 0;
    if (*frac >= EMU_HALF_FRAC_DEN) { *frac -= EMU_HALF_FRAC_DEN; extra = 1; }
    if (on) {
        /* 8 half-periods on the 18.875-cyc grid, summing to exactly 151 (= 4 periods of fc/16). Keeps every
         * subcarrier edge aligned to the FPGA's 16-tick window instead of drifting 0.6%/period. */
        static const uint8_t s_subc_dur[8] = { 38, 38, 37, 38, 38, 38, 37, 38 };   /* 64 MHz: sum 302 = 4*fc/16 */
#if EMU_CHIP_SUBC
        /* TEST: hold MOSI HIGH for the whole modulated half-bit and let the ST25R3916's own (carrier-locked)
         * subcarrier generator make the 847 kHz - phase-locked to the reader's field, unlike my CPU-timed
         * toggle which drifts against the reader's crystal. */
        (void)s_subc_dur;
        EMU_MOSI_HI;
        *t += EMU_HALF_ETU + (uint32_t)extra;
        while ((int32_t)(DWT->CYCCNT - *t) < 0) { }
        EMU_MOSI_LO;
#else
        for (int e = 0; e < 8; e++) {
            if (e & 1) EMU_MOSI_LO; else EMU_MOSI_HI;   /* set at the start of the sub-period so the subcarrier
                                                         * edge is not delayed into the FPGA correlation window */
            *t += s_subc_dur[e] + (e == 2 ? extra : 0);
            while ((int32_t)(DWT->CYCCNT - *t) < 0) { }
        }
        EMU_MOSI_LO;                                     /* end the half-bit low */
#endif
    } else {
        *t += EMU_HALF_ETU + (uint32_t)extra;
        while ((int32_t)(DWT->CYCCNT - *t) < 0) { }
        EMU_MOSI_LO;
    }
}

/* Jitter-free DMA subcarrier TX. TIM1 (free during HF emu) generates the
 * fc/16 load-mod edges on MOSI/PB5 via two DMA channels triggered by TIM1_UP: DMA1_Ch2 -> TIM1->ARR (next edge
 * duration), DMA1_Ch3 -> GPIOB->BSRR (next MOSI level). No CPU per edge => zero inter-edge jitter and the CPU is
 * free during the reply. The raw DWT-timed path handles a static reply's first use while its DMA cache is built. */
/* Longest reply is an 18-byte encrypted READ: 1 start + 18*(8+1) = 163 air bits; the stop symbol is not sent.
 * Each air bit expands to 9 DMA segments, for 1468 entries including the sentinel. Sized to hold it whole.
 * ATQA/UID/SAK/auth replies are far shorter. */
#define TXD_MAX   1536
#define MOSI_SET  (1u << 5)
#define MOSI_RST  (1u << (5 + 16))
/* Heap-allocated (emu_edge_cache_alloc) - not static .bss: at 1536 words each these two are 12 KB, which as
 * static .bss would permanently reduce the FreeRTOS heap and starve hf_sniff's 64 KB contiguous capture ring.
 * Emu and sniff never run at once (set_mode arbitrates),
 * so heap-backing them costs nothing: they exist only while emulating and free back on teardown. */
static uint32_t *s_txd_arr;    /* per-segment TIM1 period (ticks-1) */
static uint32_t *s_txd_bsrr;   /* per-segment MOSI BSRR (set/reset) */
static volatile int s_txd_n;
static struct {
    uint8_t (*next)(void *ctx);
    void *ctx;
    uint8_t prefix_raw;
    uint8_t request[32];
    int *result;
    int active, prearmed, static_reply, short_frame;
    int frame_len, request_len, request_min_len, nbits, n;
    unsigned frac;
} s_early_stream;
/* Edge-array cache: build the DMA edges once per reply (keyed by the module's reply pointer), then pointer-swap
 * on repeats so the FDT-critical arm/fire never rebuilds. Heap-allocated in emu_setup, freed on teardown. */
#define EC_N 5
static uint32_t     *s_ec_arr[EC_N], *s_ec_bsrr[EC_N];
static const uint8_t *s_ec_ptr[EC_N]; static int s_ec_n[EC_N]; static uint8_t s_ec_rr;
static uint32_t *s_ec_buf;   /* single heap block backing all EC slots (freed on teardown) */
static uint32_t *s_txd_arr_p, *s_txd_bsrr_p;   /* the fire reads these (cache slot or scratch); set by cache alloc */
static void emu_edge_cache_free(void)
{
    if (s_ec_buf) { vPortFree(s_ec_buf); s_ec_buf = 0; }
    for (int i = 0; i < EC_N; i++) { s_ec_arr[i] = 0; s_ec_bsrr[i] = 0; s_ec_ptr[i] = 0; }
    if (s_txd_arr)  { vPortFree(s_txd_arr);  s_txd_arr  = 0; }   /* return the 12 KB TX scratch so a later */
    if (s_txd_bsrr) { vPortFree(s_txd_bsrr); s_txd_bsrr = 0; }   /* sniff gets its 64 KB contiguous ring */
    s_txd_arr_p = s_txd_arr; s_txd_bsrr_p = s_txd_bsrr;
}
static void emu_edge_cache_alloc(void)
{
    s_ec_rr = 0;
    for (int i = 0; i < EC_N; i++) { s_ec_ptr[i] = 0; s_ec_arr[i] = 0; s_ec_bsrr[i] = 0; }
    uint32_t *ecbuf = pvPortMalloc((size_t)EC_N * 512u * 2u * sizeof(uint32_t));
    if (ecbuf) for (int i = 0; i < EC_N; i++) { s_ec_arr[i] = ecbuf + (size_t)i * 1024u; s_ec_bsrr[i] = ecbuf + (size_t)i * 1024u + 512u; }
    s_ec_buf = ecbuf;
    s_txd_arr  = pvPortMalloc(TXD_MAX * sizeof(uint32_t));   /* global TX scratch (crypto replies that overflow a */
    s_txd_bsrr = pvPortMalloc(TXD_MAX * sizeof(uint32_t));   /* 512-word cache slot); freed on teardown */
    s_txd_arr_p = s_txd_arr; s_txd_bsrr_p = s_txd_bsrr;
}
/* Prepare the DMA edge arrays for the staged reply: cache hit -> pointer-swap (no build); miss/no-key -> build.
 * `key` = the module's stable reply pointer (ts_atqa/ts_uid/ts_sak) or NULL for a one-shot (crypto stream). */
static int emu_tx_build_to(uint32_t *da, uint32_t *db, const uint8_t *bits, int nbits);
static void emu_dma_prep(const uint8_t *key, int nbits)
{
    if (key && s_ec_buf) {
        for (int i = 0; i < EC_N; i++)
            if (s_ec_ptr[i] == key) { s_txd_arr_p = s_ec_arr[i]; s_txd_bsrr_p = s_ec_bsrr[i]; s_txd_n = s_ec_n[i]; return; }
        int need = nbits * 9 + 2;                         /* worst case 9 segments/bit + sentinel */
        if (need <= 512) {                                /* fits a cache slot: build once, keep it */
            int e = s_ec_rr; s_ec_rr = (uint8_t)((e + 1) % EC_N);
            s_ec_n[e] = emu_tx_build_to(s_ec_arr[e], s_ec_bsrr[e], s_tx_bits, nbits);
            s_ec_ptr[e] = key; s_txd_arr_p = s_ec_arr[e]; s_txd_bsrr_p = s_ec_bsrr[e]; s_txd_n = s_ec_n[e];
            return;
        }
    }
    if (!s_txd_arr || !s_txd_bsrr) { s_txd_n = 0; return; }               /* scratch alloc failed (OOM) - skip reply */
    s_txd_n = emu_tx_build_to(s_txd_arr, s_txd_bsrr, s_tx_bits, nbits);   /* fallback: global scratch */
    s_txd_arr_p = s_txd_arr; s_txd_bsrr_p = s_txd_bsrr;
}

static int emu_dma_cached(const uint8_t *key)
{
    if (!key || !s_ec_buf) return 0;
    for (int i = 0; i < EC_N; i++)
        if (s_ec_ptr[i] == key) return 1;
    return 0;
}
static uint32_t s_pat_arr[2][8], s_pat_bsrr[2][8];   /* precomputed per-half-bit segment patterns */

static void emu_tx_pat_init(void)
{
    static const uint8_t sub[8] = { 38,38,37,38,38,38,37,38 };   /* fc/16 half-periods @64MHz (= emu_tx_half) */
    for (int e = 0; e < 8; e++) { s_pat_arr[1][e] = sub[e] - 1u; s_pat_bsrr[1][e] = (e & 1) ? MOSI_RST : MOSI_SET; }
    s_pat_arr[0][0] = EMU_HALF_ETU - 1u; s_pat_bsrr[0][0] = MOSI_RST;   /* idle half-bit = 1 low segment */
}

static int emu_tx_append_bit(uint32_t *da, uint32_t *db, int n, int b, unsigned *frac)
{
    for (int half = 0; half < 2; half++) {
        int on = (half == 0) ? b : !b;
        *frac += EMU_HALF_FRAC_NUM;
        int extra = 0;
        if (*frac >= EMU_HALF_FRAC_DEN) { *frac -= EMU_HALF_FRAC_DEN; extra = 1; }
        if (on) {
            __builtin_memcpy(&da[n], s_pat_arr[1], 8 * sizeof(uint32_t));
            __builtin_memcpy(&db[n], s_pat_bsrr[1], 8 * sizeof(uint32_t));
            da[n + 2] += (uint32_t)extra;
            n += 8;
        } else {
            da[n] = s_pat_arr[0][0] + (uint32_t)extra;
            db[n] = s_pat_bsrr[0][0];
            n++;
        }
    }
    return n;
}

/* Expand the Manchester air-bits into edge arrays: each half-bit is modulated (8 subcarrier segments) or idle
 * (1 low segment), exactly like emu_tx_half. Appends a long-low sentinel; the DMA-TC ISR stops the timer on it. */
__attribute__((optimize("O2")))
static int emu_tx_build_to(uint32_t *da, uint32_t *db, const uint8_t *bits, int nbits)
{
    int n = 0; unsigned frac = 0;
    for (int i = 0; i < nbits && n + 20 < TXD_MAX; i++)
        n = emu_tx_append_bit(da, db, n, (bits[i >> 3] >> (i & 7)) & 1, &frac);
    da[n] = 0xFFFFu; db[n] = MOSI_RST; n++;   /* sentinel */
    return n;
}

static void emu_tx_dma_setup(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; (void)RCC->APB2ENR;
    TIM1->CR1 = 0; TIM1->PSC = 0; TIM1->ARR = 0xFFFFu; TIM1->CNT = 0;
    TIM1->DIER = TIM_DIER_UDE;                        /* update event -> DMA request */
    TIM1->EGR = TIM_EGR_UG; TIM1->SR = 0;
    DMAMUX1_Channel1->CCR = 0x19u;                    /* DMA1_Ch2 <- TIM1_UP */
    DMAMUX1_Channel2->CCR = 0x19u;                    /* DMA1_Ch3 <- TIM1_UP */
    DMA1_Channel2->CPAR = (uint32_t)(uintptr_t)&TIM1->ARR;
    DMA1_Channel3->CPAR = (uint32_t)(uintptr_t)&GPIOB->BSRR;
    DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3;
    NVIC_SetPriority(DMA1_Channel2_IRQn, 4);
    NVIC_ClearPendingIRQ(DMA1_Channel2_IRQn);
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}

/* Fire the staged reply: play segments 1..n-1 via DMA on TIM1_UP; segment 0 is loaded by hand at the exact FDT. */
static void emu_tx_dma_fire(void)
{
    int n = s_txd_n;                                /* edges were pre-built (cache) at arm time - no build here */
    uint32_t *da = s_txd_arr_p, *db = s_txd_bsrr_p;
    if (n < 2) return;
    EXTI->IMR1 &= ~(1u << 4);                        /* mask the MISO wake for the reply: our own load-mod reflects onto
                                                       * MISO and would spuriously wake+churn recv (inflating the next
                                                       * frame's detection). The DMA-TC ISR re-arms it when the reply ends. */
    DMA1_Channel2->CCR = 0; DMA1_Channel3->CCR = 0;
    DMA1->IFCR = DMA_IFCR_CTCIF2 | DMA_IFCR_CGIF2;
    DMA1_Channel2->CMAR = (uint32_t)(uintptr_t)&da[1];  DMA1_Channel2->CNDTR = (uint32_t)(n - 1);
    DMA1_Channel3->CMAR = (uint32_t)(uintptr_t)&db[1];  DMA1_Channel3->CNDTR = (uint32_t)(n - 1);
    DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1 |
                         DMA_CCR_PL_1 | DMA_CCR_TCIE | DMA_CCR_EN;
    DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1 |
                         DMA_CCR_PL | DMA_CCR_EN;
    TIM1->ARR = da[0]; TIM1->CNT = 0;
    uint32_t t = s_emu_rx_cyc + s_fdt_ticks + (uint32_t)s_phase_off;   /* start the first edge at the exact FDT */
    while ((int32_t)(DWT->CYCCNT - t) > 0) t += EMU_ETU;
    while ((int32_t)(DWT->CYCCNT - t) < 0) { }
    s_tx_end_cyc = DWT->CYCCNT + (uint32_t)s_tx_nbits * 604u +
                   ((uint32_t)s_tx_nbits * 2u * EMU_HALF_FRAC_NUM) / EMU_HALF_FRAC_DEN + EMU_RX_SETTLE;
    GPIOB->BSRR = db[0];
    TIM1->CR1 = TIM_CR1_CEN;
}

void DMA1_Channel2_IRQHandler(void)   /* reply done: last (sentinel) period loaded -> stop TIM1, hold MOSI low */
{
    if (DMA1->ISR & DMA_ISR_TCIF2) {
        DMA1->IFCR = DMA_IFCR_CTCIF2 | DMA_IFCR_CGIF2;
        TIM1->CR1 = 0;
        DMA1_Channel2->CCR = 0; DMA1_Channel3->CCR = 0;
        GPIOB->BSRR = MOSI_RST;
        emu_rx_disable();                              /* suppress post-reply demodulator recovery edges */
        while ((int32_t)(DWT->CYCCNT - s_tx_end_cyc) < 0) { }
        s_edge_n = 0;
        emu_rx_enable();
        s_tx_end_cyc = DWT->CYCCNT + 128u;             /* let recv rebase its DMA window after re-enabling RX */
        EXTI->PR1 = (1u << 4); EXTI->IMR1 |= (1u << 4);   /* reply done: clear + re-arm the MISO wake (masked at fire) */
    }
}

static void emu_tx_raw(const uint8_t *bits, int nbits)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    /* Phase-pinned start: begin the subcarrier at exactly last_edge + FDT + phase_off. s_emu_rx_cyc is the precise
     * (pri-4, USB-preemption-free) timestamp of the reader's final carrier-grid-aligned edge, so this lands the
     * subcarrier at the same carrier phase every frame - no ISR-entry jitter. EMU_PHASE_OFF slides it within one
     * fc/16 window to sit mid-window (swept). TIM17 fired ~90 cyc early so this busy-wait has slack. */
    uint32_t t = s_emu_rx_cyc + s_fdt_ticks + (uint32_t)s_phase_off;
    /* A crypto reply arms after the FDT target (recv-return + the mandatory nr-feed/ar-advance + the pre-drain
     * exceed the ~1270-cp window), so `t` is already in the past. Left as-is, every emu_tx_half busy-wait below is
     * instantly satisfied and the subcarrier collapses into a sub-bit garbage blip. Snap `t` forward in whole
     * ETU steps (604 cyc = 8*fc/16, preserves the legal fc/128 FDT lattice) until it is just ahead of now: the
     * reply is then well-formed at the earliest phase-consistent slot - later FDT, but still decodable. */
    while ((int32_t)(DWT->CYCCNT - t) > 0) t += EMU_ETU;
    while ((int32_t)(DWT->CYCCNT - t) < 0) { }
    unsigned frac = 0;
    for (int i = 0; i < nbits; i++) {
        int b = (bits[i >> 3] >> (i & 7)) & 1;
        emu_tx_half(&t,  b, &frac);                      /* first half-bit  */
        emu_tx_half(&t, !b, &frac);                      /* second half-bit */
    }
    EMU_MOSI_LO;
    emu_rx_disable();                                  /* suppress post-reply demodulator recovery edges */
    s_tx_end_cyc = DWT->CYCCNT + EMU_RX_SETTLE;
    while ((int32_t)(DWT->CYCCNT - s_tx_end_cyc) < 0) { }
    s_edge_n = 0;
    emu_rx_enable();
    s_tx_end_cyc = DWT->CYCCNT + 128u;                 /* let recv discard the disabled-RX interval */
    if (!pm) __enable_irq();
}

/* Arm a reply for the HW-timer FDT: the air-bits are already in s_tx_bits (built in-place by the senders, no
 * copy - the copy was on the FDT-critical path). TIM17 (armed per-edge in the EXTI ISR) fires emu_tx_raw at
 * last_edge + FDT. Publishing order: nbits before the armed flag (ISR reads armed first). */
static int emu_tx_arm(int nbits, bool keep_basepri)
{
    if (nbits <= 0 || nbits > EMU_FIFO * 8) {
        if (!keep_basepri) __set_BASEPRI(0);
        return RFID_ERR_UNSUPP;
    }
    /* Static edges are already cached and dynamic ciphertext uses the one-shot DMA scratch, so arming only
     * publishes the prepared reply to the timer ISR. */
    int late = (int32_t)(DWT->CYCCNT - s_emu_rx_cyc) >= (int32_t)(s_fdt_ticks - 90u);
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    s_tx_nbits = nbits;
    __asm volatile("" ::: "memory");            /* publish bits before arming */
    s_tx_armed = 1;
    /* If the one-pulse timer already expired, fire this reply now rather than
     * leaving it armed for the next reader frame. */
    if (late || !(TIM17->CR1 & TIM_CR1_CEN) || (TIM17->SR & TIM_SR_UIF)) {
        TIM17->CNT = TIM17->ARR - 1u;
        TIM17->CR1 |= TIM_CR1_CEN;
    }
    if (!pm) __enable_irq();
    if (!keep_basepri)
        __set_BASEPRI(0);   /* armed: the pri-4 TIM17 fires the reply from here, USB may resume */
    return 0;
}

static void emu_early_static_prearm(int nbits)
{
    if (!s_early_stream.short_frame) return;
    uint32_t fdt = (s_early_stream.prefix_raw >> 6) & 1u ? EMU_FDT_1 : EMU_FDT_0;
    uint32_t pm = __get_PRIMASK();
    uint32_t last_edge = 0, rx_cyc = 0;
    int stable = 0;
    for (int attempt = 0; attempt < 2; attempt++) {
        last_edge = s_last_edge_cyc;
        rx_cyc = carrier_lock(last_edge);
        __disable_irq();
        if (last_edge == s_last_edge_cyc && !(EXTI->PR1 & (1u << 4))) {
            stable = 1;
            break;
        }
        if (!pm) __enable_irq();
    }
    if (!stable) return;
    int bad_nbits = nbits <= 0 || nbits > EMU_FIFO * 8;
    int tx_armed = s_tx_armed;
    int owned = s_tx_spec_state != EMU_SPEC_NONE;
    if (!bad_nbits && !tx_armed && !owned && s_mp_maxi <= 7) {
        uint32_t timer_running = TIM17->CR1 & TIM_CR1_CEN;
        uint32_t timer_pending = TIM17->SR & TIM_SR_UIF;
        s_emu_rx_cyc = rx_cyc;
        s_fdt_ticks = fdt;
        s_tx_nbits = nbits;
        s_tx_spec_state = EMU_SPEC_ARMED;
        __DMB();
        s_tx_armed = 1;
        s_early_stream.prearmed = 1;
        if (!timer_running && !timer_pending) {
            TIM17->CNT = TIM17->ARR - 1u;
            TIM17->CR1 |= TIM_CR1_CEN;
        }
    }
    if (!pm) __enable_irq();
}

/* TIM17 fires FDT cycles after the last reader edge (frame end). If a reply is staged, load-modulate it
 * now. The MISO EXTI is masked across the bit-bang so our own load-modulation edges don't re-arm the timer
 * or wake the recv task. Vector shared with TIM1 (LF-only, idle during HF emu). */
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
    if (!(TIM17->SR & TIM_SR_UIF)) return;
    TIM17->SR = ~TIM_SR_UIF;                     /* clear update flag */
    if (s_tx_armed) {
        s_tx_armed = 0;
        if (s_tx_spec_state == EMU_SPEC_ARMED) {
            s_tx_spec_state = EMU_SPEC_FIRED;
        }
        if (s_tx_crypto && s_tx_crypto_dma) {
            emu_tx_dma_fire();                    /* dynamic ciphertext was expanded into the DMA scratch array */
        } else if (s_tx_crypto || s_tx_force_raw) {   /* A static reply's first use is sent directly, then cached
                                                       * outside its FDT-critical staging path. */
            uint32_t im = EXTI->IMR1 & (1u << 4);
            EXTI->IMR1 &= ~(1u << 4);
            emu_tx_raw(s_tx_bits, s_tx_nbits);
            if (s_tx_dma_pending && s_tx_dma_key && s_ec_buf)
                emu_dma_prep(s_tx_dma_key, s_tx_nbits);
            EXTI->PR1 = (1u << 4);
            EXTI->IMR1 |= im;
        } else if (s_use_dma_tx) {
            emu_tx_dma_fire();                    /* DMA plays the reply; CPU free (MOSI/PB5 edges don't hit PB4 EXTI) */
        } else {
            uint32_t im = EXTI->IMR1 & (1u << 4);
            EXTI->IMR1 &= ~(1u << 4);
            emu_tx_raw(s_tx_bits, s_tx_nbits);
            EXTI->PR1 = (1u << 4);
            EXTI->IMR1 |= im;
        }
        s_tx_force_raw = 0;
        s_tx_dma_pending = 0;
        s_tx_dma_key = 0;
        s_tx_crypto = 0;
        s_tx_crypto_dma = 0;
        s_fdt_locked = 0;                        /* reply sent: the next reader frame may re-arm the timer */
    }
}

/* Load-modulate a pre-encoded tag reply. `tosend` = iso14a_tag_encode: [0]=template, [1]=start bit, then
 * 8 data + 1 parity per byte, SEC_F stop. We generate the SOF ourselves, so keep the start bit and drop
 * only the template (skip 1). Each kept symbol is one air bit (SEC_D=1). */
int hal_rfid_hf_emu_send(const uint8_t *tosend, int len)
{
    if (s_mode != RFID_HF_EMU || !tosend || len < 3 || !s_emu_up) return RFID_ERR_UNSUPP;
    if (s_grp_callback_active && !s_use_dma_tx) return RFID_ERR_UNSUPP;
    for (int e = 0; e < SEND_CACHE_N; e++) {                  /* cache hit: memcpy the pre-decoded bits */
        if (s_send_cache[e].ptr == tosend) {
            int nb = (s_send_cache[e].n + 7) / 8;
            for (int i = 0; i < nb; i++) s_tx_bits[i] = s_send_cache[e].bits[i];
            if (s_grp_callback_active && s_use_dma_tx) {
                emu_dma_prep(tosend, s_send_cache[e].n);
                if (s_txd_n < 2) return RFID_ERR_UNSUPP;
                s_early_stream.next = 0;
                s_early_stream.ctx = 0;
                s_early_stream.result = 0;
                s_early_stream.prefix_raw = 0;
                for (int b = 0; b < 7; b++)
                    if (s_seq[b + 1] == 2) s_early_stream.prefix_raw |= (uint8_t)(1u << b);
                s_early_stream.prearmed = 0;
                s_early_stream.static_reply = 1;
                s_early_stream.short_frame = s_grp_nb < 8;
                s_early_stream.frame_len = s_grp_n + 1;
                s_early_stream.request_len = 0;
                s_early_stream.request_min_len = 0;
                s_early_stream.nbits = s_send_cache[e].n;
                s_early_stream.active = 1;
                emu_early_static_prearm(s_send_cache[e].n);
                return 0;
            }
            if (s_use_dma_tx && emu_dma_cached(tosend))
                emu_dma_prep(tosend, s_send_cache[e].n);       /* edge-cache hit -> pointer swap, no build */
            else if (s_use_dma_tx) {
                s_tx_force_raw = 1;
                s_tx_dma_pending = 1;
                s_tx_dma_key = tosend;
            }
            int rc = emu_tx_arm(s_send_cache[e].n, false);
            if (rc != 0) { s_tx_force_raw = 0; s_tx_dma_pending = 0; s_tx_dma_key = 0; }
            return rc;
        }
    }
    int n = 0;                                               /* miss: decode symbols -> air-bits in place */
    for (int i = 1; i < len && n < EMU_FIFO * 8; i++) {       /* skip template, keep start bit */
        if (tosend[i] == EMU_SEC_F) break;
        if ((n & 7) == 0) s_tx_bits[n >> 3] = 0;
        if (tosend[i] == EMU_SEC_D) s_tx_bits[n >> 3] |= (uint8_t)(1u << (n & 7));
        n++;
    }
    int e = s_send_cache_rr; s_send_cache_rr = (uint8_t)((e + 1) % SEND_CACHE_N);   /* store round-robin */
    s_send_cache[e].ptr = tosend; s_send_cache[e].n = n;
    for (int i = 0; i < (n + 7) / 8; i++) s_send_cache[e].bits[i] = s_tx_bits[i];
    if (s_grp_callback_active && s_use_dma_tx) {
        emu_dma_prep(tosend, n);
        if (s_txd_n < 2) return RFID_ERR_UNSUPP;
        s_early_stream.next = 0;
        s_early_stream.ctx = 0;
        s_early_stream.result = 0;
        s_early_stream.prefix_raw = 0;
        for (int b = 0; b < 7; b++)
            if (s_seq[b + 1] == 2) s_early_stream.prefix_raw |= (uint8_t)(1u << b);
        s_early_stream.prearmed = 0;
        s_early_stream.static_reply = 1;
        s_early_stream.short_frame = s_grp_nb < 8;
        s_early_stream.frame_len = s_grp_n + 1;
        s_early_stream.request_len = 0;
        s_early_stream.request_min_len = 0;
        s_early_stream.nbits = n;
        s_early_stream.active = 1;
        emu_early_static_prearm(n);
        return 0;
    }
    if (s_use_dma_tx) {
        s_tx_force_raw = 1;
        s_tx_dma_pending = 1;
        s_tx_dma_key = tosend;
    }
    int rc = emu_tx_arm(n, false);
    if (rc != 0) { s_tx_force_raw = 0; s_tx_dma_pending = 0; s_tx_dma_key = 0; }
    return rc;
}

/* Pull a small generic tag-symbol head, arm the existing FDT/DMA path, then fill the remaining descriptors while
 * that head is waiting or already on air. Every Manchester air bit expands to nine descriptors, independent of
 * its value, so the final DMA length and sentinel location are known before the producer is advanced. */
__attribute__((optimize("O2")))
static int emu_send_stream(uint8_t (*next)(void *ctx), void *ctx, int nsymbols,
                           const uint8_t *request, int request_len, int request_min_len,
                           int *result)
{
    if (result) *result = 0;
    if (s_mode != RFID_HF_EMU || !s_emu_up || !s_use_dma_tx || !next || nsymbols < 3 ||
        nsymbols > EMU_FIFO * 8 + 1 || nsymbols > (TXD_MAX - 1) / 9 + 1 ||
        !s_txd_arr || !s_txd_bsrr)
        return RFID_ERR_UNSUPP;
    int nbits = nsymbols - 1;                 /* transmit start + data/parity; consume the final stop symbol */
    int total = nbits * 9 + 1;

    if (s_grp_callback_active && s_early_stream.active) emu_early_stream_cancel();
    __set_BASEPRI(EMU_MASK_USB);
    unsigned frac = 0;
    int n = 0;
    for (int i = 0; i < 2; i++)
        n = emu_tx_append_bit(s_txd_arr, s_txd_bsrr, n, next(ctx) == EMU_SEC_D, &frac);
    s_txd_arr[total - 1] = 0xFFFFu;
    s_txd_bsrr[total - 1] = MOSI_RST;
    s_txd_arr_p = s_txd_arr;
    s_txd_bsrr_p = s_txd_bsrr;
    s_txd_n = total;
    s_tx_crypto = 1;
    s_tx_crypto_dma = 1;
    __DMB();

    /* Keep only a short head until the completed request is validated. Once
     * committed, the producer stays ahead of the DMA while the reply is on air. */
    if (s_grp_callback_active) {
        s_early_stream.next = next;
        s_early_stream.ctx = ctx;
        s_early_stream.result = result;
        s_early_stream.prefix_raw = 0;
        s_early_stream.prearmed = 0;
        s_early_stream.static_reply = 0;
        s_early_stream.short_frame = 0;
        s_early_stream.frame_len = request ? request_len : s_grp_n + 1;
        s_early_stream.request_len = request ? request_len : 0;
        s_early_stream.request_min_len = request ? request_min_len : 0;
        for (int i = 0; i < s_early_stream.request_len; i++)
            s_early_stream.request[i] = request[i];
        s_early_stream.nbits = nbits;
        s_early_stream.n = n;
        s_early_stream.frac = frac;
        s_early_stream.active = 1;
        return 0;
    }

    /* Keep the USB/SysTick priority mask inherited from recv until the tail is complete. TIM17 and the TX DMA
     * run at priority 4, so transmission can start while this task continues producing future descriptors. */
    (void)emu_tx_arm(nbits, true);
    for (int i = 2; i < nbits; i++) {
        n = emu_tx_append_bit(s_txd_arr, s_txd_bsrr, n, next(ctx) == EMU_SEC_D, &frac);
        __DMB();
    }
    (void)next(ctx);                            /* stop symbol is framing, not an additional Manchester air bit */
    __DMB();
    __set_BASEPRI(0);
    return 0;
}

int hal_rfid_hf_emu_send_stream(uint8_t (*next)(void *ctx), void *ctx, int nsymbols)
{
    return emu_send_stream(next, ctx, nsymbols, 0, 0, 0, 0);
}

int hal_rfid_hf_emu_send_stream_match(uint8_t (*next)(void *ctx), void *ctx, int nsymbols,
                                      const uint8_t *request, int request_len, int request_min_len,
                                      int *result)
{
    if (!result) return RFID_ERR_UNSUPP;
    *result = 0;
    if (!s_grp_callback_active || !request || request_min_len <= 0 ||
        request_min_len > request_len ||
        request_len > (int)sizeof s_early_stream.request)
        return RFID_ERR_UNSUPP;
    return emu_send_stream(next, ctx, nsymbols, request, request_len, request_min_len, result);
}

static int emu_early_stream_commit(int frame_len)
{
    if (!s_early_stream.active) return 0;
    int short_frame = s_grp_n == 0 && s_grp_nb >= 6 && s_grp_nb <= 7;
    int request_match = s_early_stream.request_len == 0;
    if (frame_len >= s_early_stream.request_min_len &&
        frame_len <= s_early_stream.request_len) {
        request_match = 1;
        for (int i = 0; i < frame_len; i++)
            if (s_early_stream.request[i] != s_grp_out[i]) {
                request_match = 0;
                break;
            }
    }
    int frame_len_match = s_early_stream.request_len ?
                          (frame_len >= s_early_stream.request_min_len &&
                           frame_len <= s_early_stream.request_len) :
                          s_early_stream.frame_len == frame_len;
    if (!frame_len_match ||
        !request_match ||
        s_early_stream.short_frame != short_frame ||
        (short_frame && s_early_stream.prefix_raw != s_grp_out[0])) {
        emu_early_stream_cancel();
        return 0;
    }
    uint8_t (*next)(void *) = s_early_stream.next;
    void *ctx = s_early_stream.ctx;
    int nbits = s_early_stream.nbits;
    int n = s_early_stream.n;
    unsigned frac = s_early_stream.frac;
    int prearmed = s_early_stream.prearmed;
    int static_reply = s_early_stream.static_reply;
    int *result = s_early_stream.result;

    s_early_stream.active = 0;
    s_early_stream.prearmed = 0;
    s_early_stream.result = 0;
    if (prearmed) {
        uint32_t pm = __get_PRIMASK();
        __disable_irq();
        int owned = s_tx_spec_state;
        if (owned == EMU_SPEC_ARMED || owned == EMU_SPEC_FIRED)
            s_tx_spec_state = EMU_SPEC_NONE;
        if (!pm) __enable_irq();
        if (owned == EMU_SPEC_ARMED || owned == EMU_SPEC_FIRED) {
            if (result) *result = 1;
            __set_BASEPRI(0);
            return 1;
        }
    } else {
    }
    if (emu_tx_arm(nbits, true) != 0) {
        s_tx_crypto = 0;
        s_tx_crypto_dma = 0;
        __set_BASEPRI(0);
        return 0;
    }
    if (static_reply) {
        if (result) *result = 1;
        __set_BASEPRI(0);
        return 1;
    }
    for (int i = 2; i < nbits; i++) {
        n = emu_tx_append_bit(s_txd_arr, s_txd_bsrr, n, next(ctx) == EMU_SEC_D, &frac);
        __DMB();
    }
    (void)next(ctx);
    __DMB();
    if (result) *result = 1;
    __set_BASEPRI(0);
    return 1;
}

static int emu_early_stream_pending(void)
{
    return s_early_stream.active;
}

static void emu_early_stream_cancel(void)
{
    if (!s_early_stream.active) return;
    int static_reply = s_early_stream.static_reply;
    int prearmed = s_early_stream.prearmed;
    s_early_stream.active = 0;
    s_early_stream.prearmed = 0;
    s_early_stream.result = 0;
    s_early_stream.request_len = 0;
    s_early_stream.request_min_len = 0;
    if (prearmed) {
        uint32_t pm = __get_PRIMASK();
        __disable_irq();
        if (s_tx_spec_state == EMU_SPEC_ARMED) {
            s_tx_armed = 0;
        }
        s_tx_spec_state = EMU_SPEC_NONE;
        if (!pm) __enable_irq();
    }
    if (!static_reply) {
        s_tx_crypto = 0;
        s_tx_crypto_dma = 0;
    }
}

int hal_rfid_hf_emu_prepare(const uint8_t *tosend, int len)
{
    if (s_mode != RFID_HF_EMU || !s_emu_up || !s_use_dma_tx || !s_ec_buf ||
        !tosend || len < 3)
        return RFID_ERR_UNSUPP;
    for (int e = 0; e < SEND_CACHE_N; e++)
        if (s_send_cache[e].ptr == tosend) return 0;

    int n = 0;
    for (int i = 1; i < len && n < EMU_FIFO * 8; i++) {
        if (tosend[i] == EMU_SEC_F) break;
        if ((n & 7) == 0) s_tx_bits[n >> 3] = 0;
        if (tosend[i] == EMU_SEC_D) s_tx_bits[n >> 3] |= (uint8_t)(1u << (n & 7));
        n++;
    }
    int e = s_send_cache_rr;
    s_send_cache_rr = (uint8_t)((e + 1) % SEND_CACHE_N);
    s_send_cache[e].ptr = tosend;
    s_send_cache[e].n = n;
    for (int i = 0; i < (n + 7) / 8; i++) s_send_cache[e].bits[i] = s_tx_bits[i];
    emu_dma_prep(tosend, n);
    return s_txd_n >= 2 ? 0 : RFID_ERR_UNSUPP;
}

/* Dynamic Crypto1 replies arrive as a complete encoded symbol buffer. Decode them into air bits and expand them
 * into the one-shot DMA scratch; ciphertext changes on every authentication and cannot use the static cache. */
int hal_rfid_hf_emu_send_stream_buf(const uint8_t *tosend, int len)
{
    if (s_mode != RFID_HF_EMU || !tosend || len <= 3 || len > EMU_FIFO * 8 || !s_emu_up)
        return RFID_ERR_UNSUPP;
    int n = 0;                                   /* decode symbols -> air-bits (skip template[0], keep start bit) */
    for (int i = 1; i < len && n < EMU_FIFO * 8; i++) {
        if (tosend[i] == EMU_SEC_F) break;       /* stop symbol -> reply complete */
        if ((n & 7) == 0) s_tx_bits[n >> 3] = 0;
        if (tosend[i] == EMU_SEC_D) s_tx_bits[n >> 3] |= (uint8_t)(1u << (n & 7));
        n++;
    }
    if (s_use_dma_tx) emu_dma_prep(NULL, n);      /* ciphertext is one-shot, so use the uncached DMA scratch */
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    s_tx_crypto_dma = s_use_dma_tx && s_txd_n >= 2;
    s_tx_crypto = 1;                             /* select the dynamic DMA scratch and allow a late timer retrigger */
    int rc = emu_tx_arm(n, false);
    if (rc != 0) {
        s_tx_crypto = 0;
        s_tx_crypto_dma = 0;
    }
    if (!pm) __enable_irq();
    return rc;
}

/* ---- HAL contract ---- */

uint32_t hal_rfid_caps(void)
{
    /* HF read (ST25R3916) + LF 125 kHz read (EM4100 & friends) + HF ISO14443-A tag emulation
     * (transparent-mode target; see the hf_emu_* block). LF emulation is still unimplemented. */
    return RFID_CAP_HF_READ | RFID_CAP_LF_READ | RFID_CAP_HF_EMU;
}

int hal_rfid_set_mode(rfid_mode_t mode)
{
    if (mode == s_mode && !s_hf_dirty) return 0;

    /* A prior sniff left the HF front-end in its demod config with s_mode still HF_READER (and, if a ^C
     * abort skipped its cleanup, the capture DMA/timer still running). A plain idempotent set_mode would
     * skip the re-init and the reader would come up deaf. Stop any runaway capture and fall through to a
     * full teardown + re-init (s_mode still reflects the old mode, so the teardown below runs). */
    if (s_hf_dirty) { dma_cap_stop(); s_hf_dirty = 0; }

    /* Tear down whatever is active first (HF and LF own disjoint peripherals). */
    if (s_mode == RFID_HF_READER) {
        reader_teardown();                            /* exit transparent, field off, free capture scratch */
        spi_deinit();
    } else if (s_mode == RFID_HF_EMU || s_emu_starting || s_emu_up) {
        emu_teardown();
        spi_deinit();
    } else if (s_mode == RFID_LF_READER) {
        lf_stop();
    }
    s_mode = RFID_OFF;

    switch (mode) {
    case RFID_HF_READER:
        spi_init();
        if (st25_init() != 0) return -1;
        /* Plain frontend init only. The transparent-mode reader (field on) is set up lazily on the first
         * transceive - so set_mode(HF_READER) followed by hf_sniff (passive, field off) is not disturbed. */
        s_mode = RFID_HF_READER;
        return 0;

    case RFID_HF_EMU:
        s_mode = RFID_HF_EMU;                          /* make interrupted setup visible to RFID_OFF cleanup */
        spi_init();
        if (st25_init() != 0) { spi_deinit(); s_mode = RFID_OFF; return -1; }
        if (emu_setup() != 0) {
            emu_teardown(); spi_deinit(); s_mode = RFID_OFF;
            return -1;
        }
        return 0;

    case RFID_LF_READER:
        lf_hw_init();
        s_mode = RFID_LF_READER;
        return 0;

    case RFID_OFF:
        return 0;

    default:
        return -1;   /* emulation not implemented on Flipper yet */
    }
}

void hal_rfid_field(bool on)
{
    if (s_rdr_up) return;   /* transparent reader: field is on for the whole session (can't reg-op in transparent) */
    if (!s_spi_up) return;
    if (on) {
        reg_write(REG_FIELD_ON_GT, 0);            /* minimum guard time */
        reg_set(REG_OP_CONTROL, OP_CONTROL_rx_en | OP_CONTROL_tx_en);
    } else {
        reg_clear(REG_OP_CONTROL, OP_CONTROL_rx_en | OP_CONTROL_tx_en);
    }
}

int hal_rfid_hf_probe(void)
{
    if (!s_spi_up) return -1;
    uint8_t id = reg_read(REG_IC_IDENTITY);
    if ((id & ICID_mask) != ICID_st25r3916) return -1;
    return id;
}

/* Transparent-mode software reader (the framing engine does not read a card in this config). CRC handling is
 * the portable layer's job: we hand back the raw card bytes (subcarrier-decoded) including any CRC. `flags`
 * (NO_PARITY / CRC) are accepted but the transparent path leaves parity/CRC to the decode + core. */
int hal_rfid_hf_transceive(const uint8_t *tx, int tx_bits, uint32_t flags,
                           uint8_t *rx, int rx_cap, uint32_t timeout_us)
{
    (void)timeout_us; (void)flags;
    if (s_mode != RFID_HF_READER) return RFID_ERR_UNSUPP;
    if (tx_bits <= 0) return RFID_ERR_FRAMING;
    return reader_xcv(tx, tx_bits, NULL, rx, rx_cap, NULL);
}

/* Custom-parity transceive for encrypted 14443-A frames (MIFARE Crypto1): send `nbytes` with the caller's
 * parity bit per byte (par[i]), no CRC; receive the tag answer into rx (+ optional per-byte parity rx_par). */
int hal_rfid_hf_transceive_par(const uint8_t *tx, int nbytes, const uint8_t *par,
                               uint8_t *rx, uint8_t *rx_par, int rx_cap, uint32_t timeout_us)
{
    (void)timeout_us;
    if (s_mode != RFID_HF_READER) return RFID_ERR_UNSUPP;
    if (nbytes <= 0) return RFID_ERR_FRAMING;
    return reader_xcv(tx, nbytes * 8, par, rx, rx_cap, rx_par);
}

/* ---- LF 125 kHz reader ---- */

/* Energise/de-energise the 125 kHz carrier. divisor is unused (carrier is fixed at
 * 125 kHz; the EM4100 decoder tries the bit-rate divisors itself). */
int hal_rfid_lf_field(bool on, uint32_t divisor)
{
    (void)divisor;
    if (s_mode != RFID_LF_READER) return RFID_ERR_UNSUPP;
    if (on) {
        TIM1->CNT   = 0;
        TIM1->BDTR |= TIM_BDTR_MOE;               /* advanced-timer main output enable */
        TIM1->CR1  |= TIM_CR1_CEN;
        vTaskDelay(pdMS_TO_TICKS(400));           /* field + comparator/envelope settle */
    } else {
        TIM1->CR1  &= ~TIM_CR1_CEN;
        TIM1->BDTR &= ~TIM_BDTR_MOE;
    }
    return 0;
}

/* Capture COMP1 rising-edge intervals over a ~100 ms window (>~1 EM4100 frame) and
 * hand them to the decoder as carrier-cycle counts. Field must be on. */
int hal_rfid_lf_acquire(uint8_t *buf, int max, uint32_t opts)
{
    (void)opts;
    if (s_mode != RFID_LF_READER || !buf || max <= 0) return RFID_ERR_UNSUPP;

    /* Interval buffer on the heap only for this ~100 ms acquire - like the HF sniff scratch it needn't pin
     * BSS while idle. Safe vs TIM2_IRQHandler: CC4IE (its only writer) is enabled below after this alloc
     * and disabled before the free, so the ISR can never see a freed pointer. */
    s_lf_iv = pvPortMalloc(LF_CAP * sizeof *s_lf_iv);
    if (!s_lf_iv) return RFID_ERR_UNSUPP;

    s_lf_n = 0;
    s_lf_have_prev = false;
    __asm volatile("" ::: "memory");
    s_lf_capturing = true;
    TIM2->SR = 0;
    TIM2->DIER |= TIM_DIER_CC4IE;

    vTaskDelay(pdMS_TO_TICKS(100));

    TIM2->DIER &= ~TIM_DIER_CC4IE;
    s_lf_capturing = false;

    /* us -> 125 kHz carrier cycles (÷8). Skip [0] (delta from an undefined edge). */
    int count = 0;
    for (int i = 1; i < s_lf_n && count < max; i++) {
        uint32_t cyc = s_lf_iv[i] / LF_US_PER_CYCLE;
        buf[count++] = (cyc > 0xFF) ? 0xFF : (uint8_t)cyc;
    }
    vPortFree((void *)s_lf_iv);                   /* CC4IE is off here, so no ISR can still be writing it */
    s_lf_iv = NULL;
    return count;
}

/* ---- T5577 downlink (OOK gap modulation) ----
 * T55 command bits are brief field gaps: charge the tag, drop the carrier for a start gap, then per bit
 * re-raise the carrier for a long (1) or short (0) burst + a write gap, then leave it on. Gap widths are
 * us-critical: busy-delay off TIM2's free-running 1 MHz counter inside a critical section. Timings are the
 * stock PM3/CU values. A gap drives CH1N to its idle level (MOE off + OSSI) to damp the tank so the tag
 * registers it - the same MOE toggle the stock Flipper's furi_hal_rfid_tim_read_pause uses. */
/* Timings are the stock Flipper values (lib/lfrfid/tools/t5577.c), in Tc x8 = us. They differ from the
 * CU/PM3 (which use a 9 Tc write gap): the Flipper's antenna tank rings down more slowly, so the tag only
 * registers a gap at the stock 18 Tc width (9 Tc leaves the carrier still ringing). */
#define T55_START_GAP_US  240   /* 30 Tc */
#define T55_WRITE_GAP_US  144   /* 18 Tc - field OFF between bits (stock) */
#define T55_WRITE_0_US    192   /* 24 Tc - field ON before the gap => bit 0 */
#define T55_WRITE_1_US    448   /* 56 Tc - field ON before the gap => bit 1 (stock) */
#define T55_POWERUP_MS      8
#define T55_PROGRAM_MS      6
#define T55_HB             32   /* half-bit width in carrier cycles (RF/64: 32 cycles/half-bit) */

static inline void lf_delay_us(uint32_t us)
{
    uint32_t s = TIM2->CNT, g = 0;                 /* TIM2 = 1 MHz free-run */
    while ((uint32_t)(TIM2->CNT - s) < us) if (++g > 40000000u) break;
}
static inline void lf_car_on(void)  { TIM1->BDTR |= TIM_BDTR_MOE; }
static inline void lf_car_off(void) { TIM1->BDTR &= ~TIM_BDTR_MOE; }   /* CH1N -> idle (low): damped gap */

static void lf_field_ensure(void)
{
    lf_car_on();
    TIM1->CR1 |= TIM_CR1_CEN;
    /* Charge/settle the tag + AC-coupled RX before every downlink (400ms cold, a 40ms top-up when warm - a
     * whole-tag dump holds one set_mode, and the warm top-up keeps later blocks' calibration accurate). */
    vTaskDelay(pdMS_TO_TICKS(s_lf_warm ? 40 : 400));
    s_lf_warm = true;
}

/* A read keeps the critical section held after the final field restore so its
 * capture anchor can be established at an exact offset from the downlink. */
static void lf_t55_cmd(const uint8_t *p, int nbits, bool keep_critical)
{
    lf_field_ensure();                             /* RFID_PULL stays LOW (its toggle, not the gaps, caused a
                                                    * ~130 ms RX recovery - see hal_rfid_lf_transceive). */
    vTaskDelay(pdMS_TO_TICKS(T55_POWERUP_MS));
    taskENTER_CRITICAL();                          /* gap timing must not be preempted */
    /* Align the downlink to a carrier-cycle boundary, then pad it to a whole number of bit-periods (64
     * carrier cycles). Every gap width is an integer number of carrier cycles (start 30, write-gap 18, '0' 24,
     * '1' 56), so the field-restore - and therefore the tag's carrier-locked reply relative to the capture -
     * lands at a fixed carrier phase for every block. Without the pad, a target block whose address bits make
     * its downlink an odd number of half-bits longer than block 0's shifts the capture a half-bit into the
     * reply, framing to a half-bit-shifted alias (e.g. 0034A800 read as AE2491A3); the pad removes that.
     *
     * Page 1 aligns to a bit boundary (TIM2 & 511, one carrier bit = 512us, TIM2 synced to the carrier)
     * instead of a carrier cycle: a page-1 page-switch command only registers reliably at a fixed phase of
     * the tag's carrier-locked emulation, and the bit boundary is that fixed phase. Page 0 keeps the carrier-
     * cycle boundary (its commands prefer that phase). Calibration uses the target's page, so calib + target
     * share the alignment and the rotation still transfers. */
    if (nbits >= 2 && p[1]) { while ((TIM2->CNT & 511) > 3) { } }   /* page 1: bit boundary */
    else                    { while ((TIM1->CNT & 0xFF) > 3) { } }  /* page 0: carrier cycle */
    { int cyc = 30;                                             /* start gap, in carrier cycles */
      for (int i = 0; i < nbits; i++) cyc += (p[i] ? 56 : 24) + 18;
      int pad = (64 - (cyc & 63)) & 63;                         /* round the downlink up to a whole bit */
      if (pad) lf_delay_us((uint32_t)pad * 8); }                /* field still ON */
    lf_car_off(); lf_delay_us(T55_START_GAP_US);   /* start gap */
    for (int i = 0; i < nbits; i++) {
        lf_car_on();  lf_delay_us(p[i] ? T55_WRITE_1_US : T55_WRITE_0_US);
        lf_car_off(); lf_delay_us(T55_WRITE_GAP_US);
    }
    lf_car_on();                                    /* field back on */
    if (!keep_critical) taskEXIT_CRITICAL();
}

/* T5577 block WRITE downlink (TX only): gap-modulate `p`, hold the field for the EEPROM commit, drop it. */
int hal_rfid_lf_modulate(const uint8_t *p, int nbits, uint32_t opts)
{
    (void)opts;
    if (s_mode != RFID_LF_READER || !p || nbits <= 0) return RFID_ERR_UNSUPP;
    lf_t55_cmd(p, nbits, false);
    vTaskDelay(pdMS_TO_TICKS(T55_PROGRAM_MS));      /* EEPROM programming window, field steady */
    TIM1->CR1 &= ~TIM_CR1_CEN; TIM1->BDTR &= ~TIM_BDTR_MOE; s_lf_warm = false;
    GPIOA->BSRR = (1u << (2 + 16));                 /* RFID_PULL back LOW (read-mode default) */
    return 0;
}

#define T55_READ_SETTLE_US 120
#define T55_CAP_MS         60

/* Turn the COMP1 both-edge level runs into a clean half-bit level-run stream the shared t55_extract can
 * frame. COMP1's fixed threshold slices the envelope asymmetrically - a high half-bit measures ~47 cycles,
 * a low one ~16 - so t55_extract (which assumes one half-bit width for both) mis-frames the raw runs. The
 * two levels alternate, so their single-half-bit widths are the per-parity minimum run; each run then spans
 * round((run - width)/HB) + 1 half-bits. Re-emit every run as that many HB-cycle half-bits, giving a clean
 * symmetric stream. Crucially this resolves the 1.5T ambiguity from the data (a stretched high run is 2 high
 * half-bits, a stretched low run is 2 low). Preserve the first partial run and its explicitly sampled level
 * behind the shared [0, initial_level] anchor. `raw` is captured in carrier cycles; writes clean runs to `out`. */
static int lf_t55_runs(const uint8_t *raw, int nr, int initial_level, uint8_t *out, int cap)
{
    if (nr < 4 || cap < 3) return 0;
    int w[2] = { 255, 255 };                            /* per-parity single half-bit width (HIGH vs LOW) */
    for (int i = 1; i < nr; i++)                       /* raw[0] is partial: never use it to estimate width */
        if (raw[i] < w[i & 1]) w[i & 1] = raw[i];
    if (w[0] < 4) w[0] = 4;
    if (w[1] < 4) w[1] = 4;
    int count = 0;
    out[count++] = 0;                                  /* fixed-time stream marker (not a real duration) */
    out[count++] = (uint8_t)(initial_level != 0);
    for (int i = 0; i < nr && count < cap; i++) {
        int delta = (int)raw[i] - w[i & 1];
        int nh;
        if (delta >= 0) nh = 1 + (delta + T55_HB / 2) / T55_HB;
        else            nh = 1 - (-delta + T55_HB / 2) / T55_HB;
        if (nh < (i == 0 ? 0 : 1)) nh = i == 0 ? 0 : 1;
        if (nh > 4) nh = 4;
        int d = nh * T55_HB;
        out[count++] = d > 0xFF ? 0xFF : (uint8_t)d;    /* one clean HB-multiple run per captured run */
    }
    return count;
}

/* T5577 READ: send the read downlink (field left on), then capture the tag's repeated block reply as
 * inter-edge run lengths for the shared t55 decoder (t55_extract). Both COMP1 edges -> one run each. */
int hal_rfid_lf_transceive(const uint8_t *cmd, int nbits, uint8_t *buf, int cap)
{
    if (s_mode != RFID_LF_READER || !buf || cap <= 0) return RFID_ERR_UNSUPP;
    s_lf_iv = pvPortMalloc(LF_CAP * sizeof *s_lf_iv);
    if (!s_lf_iv) return RFID_ERR_TIMEOUT;

    TIM2->CCER &= ~TIM_CCER_CC4E;                   /* both edges: each transition = one level run */
    TIM2->CCER |= TIM_CCER_CC4P | TIM_CCER_CC4NP;
    TIM2->CCER |= TIM_CCER_CC4E;

    if (cmd && nbits > 0) lf_t55_cmd(cmd, nbits, true);  /* downlink; returns IRQs-off with field on */
    else { lf_field_ensure(); taskENTER_CRITICAL(); }
    /* Deterministic settle after the command, before capture. Two jobs: (1) let the AC-coupled RX front-end
     * recover from the per-read field-off reset (it is slow); the tag keeps modulating the addressed block
     * continuously, so waiting doesn't lose it. (2) being an lf_delay_us (not vTaskDelay) it's an integer
     * carrier phase, so - with the downlink's bit-period pad + carrier alignment - the capture lands at the
     * same point in the reply every read, and block 0's rotation transfers to every block. */
    lf_delay_us(T55_READ_SETTLE_US);

    /* Capture from a time and level, not merely from the first edge. If an edge races the sample, CC4IF
     * makes us retry; an edge just after the final check remains pending and is consumed by the ISR after
     * the critical section. The first stored interval is therefore the partial run from this fixed anchor. */
    uint8_t initial_level;
    uint32_t capture_start;
    do {
        TIM2->SR = 0;
        initial_level = (COMP1->CSR & COMP_CSR_VALUE) ? 1 : 0;
        capture_start = TIM2->CNT;
    } while (TIM2->SR & TIM_SR_CC4IF);
    s_lf_n = 0;
    s_lf_prev = capture_start;
    s_lf_have_prev = true;
    __asm volatile("" ::: "memory");
    s_lf_capturing = true;
    TIM2->DIER |= TIM_DIER_CC4IE;
    taskEXIT_CRITICAL();
    vTaskDelay(pdMS_TO_TICKS(T55_CAP_MS));
    TIM2->DIER &= ~TIM_DIER_CC4IE;
    s_lf_capturing = false;

    TIM2->CCER &= ~(TIM_CCER_CC4E | TIM_CCER_CC4P | TIM_CCER_CC4NP);   /* restore rising-only (EM4100) */
    TIM2->CCER |= TIM_CCER_CC4E;

    uint8_t *raw = pvPortMalloc(LF_CAP);
    int nr = 0;
    if (raw) {
        for (int i = 0; i < s_lf_n && nr < LF_CAP; i++) {
            uint32_t cyc = s_lf_iv[i] / LF_US_PER_CYCLE;
            raw[nr++] = (cyc > 0xFF) ? 0xFF : (uint8_t)cyc;
        }
    }
    vPortFree((void *)s_lf_iv);
    s_lf_iv = NULL;
    if (!raw) return RFID_ERR_TIMEOUT;

    int count = lf_t55_runs(raw, nr, initial_level, buf, cap); /* fixed-anchor clean runs for t55_extract */
    vPortFree(raw);
    return count;
}
