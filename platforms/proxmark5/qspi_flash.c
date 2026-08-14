#include "qspi_flash.h"
#include "at32f435.h"

/* ---- QSPI1 registers (base 0xA0001000; RM ch.22 / at32f435_437_qspi.h) ---- */
#define QSPI1_BASE 0xA0001000UL
#define QR(off)    (*(volatile uint32_t *)(QSPI1_BASE + (off)))
#define Q_W0       QR(0x00)   /* command address                                */
#define Q_W1       QR(0x04)   /* adrlen[2:0] dum2[23:16] inslen[25:24]           */
#define Q_W2       QR(0x08)   /* dcnt (data byte count)                          */
#define Q_W3       QR(0x0C)   /* wen[1] rstsen[2] rstsc[3] opmode[7:5] insc[31:24] - write kicks */
#define Q_CTRL     QR(0x10)   /* clkdiv[2:0] sckmode[4] busy[18:16] keyen[21]    */
#define Q_FIFOSTS  QR(0x18)   /* txfifordy[0] rxfifordy[1]                        */
#define Q_CMDSTS   QR(0x24)   /* cmdsts[0] (command complete; clear by writing 1) */
#define Q_CTRL3    QR(0x40)   /* ispd[5:0] ispc[8]                               */
#define Q_DT_U8    (*(volatile uint8_t *)(QSPI1_BASE + 0x100))

#define Q_TXRDY    (Q_FIFOSTS & 0x1u)
#define Q_RXRDY    (Q_FIFOSTS & 0x2u)
#define Q_CMDDONE  (Q_CMDSTS  & 0x1u)

#define CRM_AHBEN3_QSPI1EN (1u << 1)   /* CRM AHBEN3 (0x38) bit 1 */

/* operation modes */
#define OM_111  0u    /* all single-line */
#define OM_114  2u    /* quad data (opcode+addr single) */

/* W25Q opcodes */
#define OP_JEDEC       0x9Fu
#define OP_RDSR1       0x05u
#define OP_RDSR2       0x35u
#define OP_WRSR        0x01u
#define OP_WREN        0x06u
#define OP_READ        0x03u   /* Read Data (single line, no dummy) */
#define OP_FASTREAD_QO 0x6Bu   /* Fast Read Quad Output (1-1-4, 8 dummy cycles) */
#define OP_PAGEPROG    0x02u
#define OP_SECTORERASE 0x20u

#define OP_WRDI        0x04u

/* Busy-spin cap: covers a real 4 KB sector erase (~50 ms) with margin, but fails
 * out in ~100 ms so a non-responding chip can never hang the caller (the ext
 * storage bring-up runs on the CLI task). */
#define SPIN_BOUND     8000000u

static uint32_t s_size;      /* detected chip size in bytes */
static uint8_t  s_jedec[3];  /* last JEDEC id (mfr, memtype, capacity) */
static bool     s_quad;      /* QE set + IO2/IO3 muxed to QSPI -> quad-line reads */

/* Configure one pin as a QSPI alternate function: push-pull, stronger drive (a
 * fast master needs it) - not just the mode, unlike a slow slave bus. */
static void qspi_pin(gpio_type *g, int p, int af)
{
    gpio_set_mux(g, p, (uint32_t)af);
    gpio_set_otype(g, p, GPIO_OTYPE_PP);
    gpio_set_drive(g, p, GPIO_DRIVE_STRONGER);
    gpio_set_mode(g, p, GPIO_MODE_MUX);
}

/* Hold a pin high as a push-pull GPIO output. On a W25Q the IO2/IO3 lines double
 * as WP#/HOLD# unless the quad-enable bit is set; in single-line mode they must
 * be driven high or writes are hardware write-protected (reads still work). */
static void qspi_pin_high(gpio_type *g, int p)
{
    g->SCR = (1u << p);                       /* ODT high before enabling output */
    gpio_set_otype(g, p, GPIO_OTYPE_PP);
    gpio_set_drive(g, p, GPIO_DRIVE_STRONGER);
    gpio_set_mode(g, p, GPIO_MODE_OUTPUT);
}

/* Load the address/length/count words (W0-W2); does not start the transfer. */
static void qspi_load(uint32_t addr, uint8_t adrlen, uint8_t dummy, uint32_t dcnt)
{
    Q_W0 = addr;
    Q_W1 = (uint32_t)(adrlen & 7u)
         | ((uint32_t)dummy << 16)
         | (1u << 24);                          /* inslen = 1 byte */
    Q_W2 = dcnt;
}

/* Build the W3 value (opcode + flags). Writing it to Q_W3 starts the transfer. */
static uint32_t qspi_w3(uint8_t opcode, uint8_t opmode, bool wen, bool rstsen, uint8_t rstsc)
{
    return ((wen    ? 1u : 0u) << 1)
         | ((rstsen ? 1u : 0u) << 2)
         | ((uint32_t)(rstsc & 1u) << 3)
         | ((uint32_t)(opmode & 7u) << 5)
         | ((uint32_t)opcode << 24);
}

/* Load the 4 command words; writing Q_W3 starts the transfer. */
static void qspi_kick(uint8_t opcode, uint32_t addr, uint8_t adrlen, uint8_t dummy,
                      uint32_t dcnt, uint8_t opmode, bool wen, bool rstsen, uint8_t rstsc)
{
    qspi_load(addr, adrlen, dummy, dcnt);
    Q_W3 = qspi_w3(opcode, opmode, wen, rstsen, rstsc);   /* <- kicks */
}

/* Wait for command completion, then clear the flag. */
static void qspi_finish(void)
{
    for (uint32_t w = 0; !Q_CMDDONE && w < SPIN_BOUND; w++) { }
    Q_CMDSTS = 0x1u;
}

/* RX/TX FIFO depth (RM 28.2: "128-byte TxFIFO/RxFIFO depth"). rxfifordy asserts
 * when the FIFO is full or holds the last data - not per byte - so reads must be
 * drained a FIFO-full at a time: wait, drain up to 128, repeat. Draining more than
 * the depth per wait reads stale bytes; draining one per wait waits per batch. */
#define QSPI_FIFO_DEPTH 128u
static void qspi_read_bytes(uint8_t *buf, uint32_t n)
{
    uint32_t i = 0;
    while (i < n) {
        for (uint32_t w = 0; !Q_RXRDY && w < SPIN_BOUND; w++) { }
        uint32_t chunk = n - i;
        if (chunk > QSPI_FIFO_DEPTH) chunk = QSPI_FIFO_DEPTH;
        for (uint32_t k = 0; k < chunk; k++) buf[i++] = Q_DT_U8;
    }
    qspi_finish();
}

/* Read a status register (0x05 / 0x35) via the controller's status path. */
static uint8_t qspi_rdsr(uint8_t opcode)
{
    qspi_kick(opcode, 0, 0, 0, 0, OM_111, false, true /*rstsen*/, 1 /*SW_ONCE*/);
    for (uint32_t w = 0; !Q_CMDDONE && w < SPIN_BOUND; w++) { }
    uint8_t s = (uint8_t)(QR(0x28) & 0xFFu);   /* rsts.spists */
    Q_CMDSTS = 0x1u;
    return s;
}

static void qspi_wren(void)
{
    qspi_kick(OP_WREN, 0, 0, 0, 0, OM_111, true /*wen*/, false, 0);
    qspi_finish();
}

/* Poll the chip's WIP (status bit 0) until the erase/program actually finishes.
 * The controller's cmdsts asserts before the chip's internal write completes, so
 * without this the next operation runs while the chip is still busy. Each iteration
 * issues a full status read (~microseconds), so the bound covers a >400 ms erase. */
#define WIP_BOUND 500000u
static void qspi_wait_wip(void)
{
    for (uint32_t w = 0; (qspi_rdsr(OP_RDSR1) & 0x01u) && w < WIP_BOUND; w++) { }
}

/* Write the status registers (SR1 then SR2) via WRSR 0x01, single-line, `n` data bytes.
 * The 0x01 form (not 0x31) writes both registers for W25Q compatibility; non-volatile,
 * so WREN first and wait out the self-timed write. */
static void qspi_write_status(const uint8_t *d, uint32_t n)
{
    qspi_wren();
    qspi_load(0, 0 /*no address*/, 0, n);
    Q_W3 = qspi_w3(OP_WRSR, OM_111, true /*wen*/, false, 0);   /* kick */
    for (uint32_t i = 0; i < n; i++) {
        for (uint32_t w = 0; !Q_TXRDY && w < SPIN_BOUND; w++) { }
        Q_DT_U8 = d[i];
    }
    qspi_finish();
    qspi_wait_wip();
}

/* Set the flash's Quad Enable bit (W25Q status register 2, bit 1) so IO2/IO3 stop
 * acting as WP#/HOLD# and become data lines. Idempotent - skips the write if already
 * set. Returns true once QE reads back set. */
static bool qspi_set_qe(void)
{
    uint8_t sr2 = qspi_rdsr(OP_RDSR2);
    if (sr2 & 0x02u) return true;
    uint8_t regs[2] = { qspi_rdsr(OP_RDSR1), (uint8_t)(sr2 | 0x02u) };
    qspi_write_status(regs, 2);
    return (qspi_rdsr(OP_RDSR2) & 0x02u) != 0;
}

bool qspi_flash_init(void)
{
    /* Clocks: QSPI1 + GPIOB/C/H. */
    CRM->AHBEN3 |= CRM_AHBEN3_QSPI1EN;
    CRM->AHBEN1 |= CRM_AHBEN1_GPIOBEN | CRM_AHBEN1_GPIOCEN | CRM_AHBEN1_GPIOHEN;
    (void)CRM->AHBEN1;

    /* Reset the QSPI peripheral: the ROM bootloader may boot from this QSPI and
     * leave the controller mid-state, which manifests as writes completing with
     * no bus transaction. AHBRST3 (0x18) bit 1 = QSPI1 reset (assert, release). */
    CRM->AHBRST3 |= CRM_AHBEN3_QSPI1EN;
    CRM->AHBRST3 &= ~CRM_AHBEN3_QSPI1EN;
    (void)CRM->AHBRST3;

    /* Pins: IO0-3 on AF10, SCK/CS on AF9. */
    qspi_pin(GPIOB, 11, 10);   /* IO0 (DI) */
    qspi_pin(GPIOH, 3,  10);   /* IO1 (DO) */
    qspi_pin_high(GPIOC, 4);   /* WP#  high (single-line: not IO2) */
    qspi_pin_high(GPIOC, 5);   /* HOLD# high (single-line: not IO3) */
    qspi_pin(GPIOB, 1,  9);    /* SCK */
    qspi_pin(GPIOB, 10, 9);    /* CS  */

    /* Switch cleanly to command-port mode, resetting the command state machine
     * and FIFOs (the SDK's qspi_xip_enable(FALSE) flush). Skipping this leaves
     * short reads working but stalls the sustained write data phase. xiprcmdf
     * (bit 19) refreshes commands/FIFOs; wait for abort (bit 8) to self-clear;
     * then clear xipsel (bit 20). Bounded so a stuck controller can't hang boot. */
    Q_CTRL |= (1u << 19);
    for (uint32_t w = 0; (Q_CTRL & (1u << 8)) && w < SPIN_BOUND; w++) { }
    Q_CTRL &= ~(1u << 20);
    for (uint32_t w = 0; (Q_CTRL & (1u << 8)) && w < SPIN_BOUND; w++) { }

    /* ctrl: clkdiv=7 (AHB 288/12 = 24 MHz), sckmode 0, busy=0 (WIP is SR bit 0),
     * keyen 0, xipsel 0. ctrl3: input-sampling phase correction (ispc, ispd=56). */
    Q_CTRL  = 7u;
    Q_CTRL3 = (56u & 0x3Fu) | (1u << 8);
    (void)Q_CTRL;             /* read-back flushes the config before the first command */

    /* JEDEC ID: [mfr, memtype, capacity]; size = 2^(capacity&0xF) * 64 KB. */
    uint8_t id[3] = {0};
    qspi_kick(OP_JEDEC, 0, 0, 0, 3, OM_111, false, false, 0);
    qspi_read_bytes(id, 3);
    s_jedec[0] = id[0]; s_jedec[1] = id[1]; s_jedec[2] = id[2];
    uint8_t mfr = id[0], nib = id[2] & 0x0Fu;
    if (mfr == 0x00u || mfr == 0xFFu) return false;   /* no chip on the bus */
    if (nib < 4u || nib > 9u) return false;           /* implausible (1 MB .. 32 MB) */
    s_size = (1u << nib) * 65536u;

    /* Prove real bidirectional comms before trusting the chip: WREN must set the
     * WEL status bit. Without this a garbled JEDEC read (bad wiring/timeout) could
     * pass the checks above and send the caller into a format/erase spin. */
    qspi_wren();
    if (!(qspi_rdsr(OP_RDSR1) & 0x02u)) { s_size = 0; return false; }
    qspi_kick(OP_WRDI, 0, 0, 0, 0, OM_111, true, false, 0);   /* clear WEL */
    qspi_finish();

    /* Go quad: set the flash QE bit (done here while IO2/IO3 are still held high as
     * WP#/HOLD#, so this WRSR isn't write-protected), then mux IO2/IO3 (PC4/PC5) to the
     * QSPI alt-function (AF10) - QE=1 has repurposed them from WP#/HOLD# to data lines.
     * Reads then use all four channels. If QE won't set, stay single-line (still works). */
    if (qspi_set_qe()) {
        qspi_pin(GPIOC, 4, 10);   /* IO2 */
        qspi_pin(GPIOC, 5, 10);   /* IO3 */
        s_quad = true;
    }
    return true;
}

uint32_t qspi_flash_size(void) { return s_size; }

bool qspi_flash_is_quad(void) { return s_quad; }

void qspi_flash_id(uint8_t out[3]) { out[0] = s_jedec[0]; out[1] = s_jedec[1]; out[2] = s_jedec[2]; }

int qspi_flash_read(uint32_t addr, void *buf, uint32_t len)
{
    if (!s_size || (uint64_t)addr + len > s_size) return -1;
    if (s_quad)   /* Fast Read Quad Output: 1-1-4, 24-bit address, 8 dummy cycles */
        qspi_kick(OP_FASTREAD_QO, addr, 3, 8, len, OM_114, false, false, 0);
    else
        qspi_kick(OP_READ, addr, 3, 0, len, OM_111, false, false, 0);
    qspi_read_bytes((uint8_t *)buf, len);
    return 0;
}

int qspi_flash_program(uint32_t addr, const void *buf, uint32_t len)
{
    if (!s_size || (uint64_t)addr + len > s_size) return -1;
    const uint8_t *p = (const uint8_t *)buf;
    while (len) {
        uint32_t chunk = 256u - (addr & 0xFFu);   /* don't cross a 256 B page */
        if (chunk > len) chunk = len;
        qspi_wren();
        /* Load W0-W2, kick (write W3), then feed the page data as the FIFO drains. */
        qspi_load(addr, 3 /*adrlen*/, 0 /*dummy*/, chunk);
        Q_W3 = qspi_w3(OP_PAGEPROG, OM_111, true, false, 0);   /* kick */
        for (uint32_t i = 0; i < chunk; i++) {
            for (uint32_t w = 0; !Q_TXRDY && w < SPIN_BOUND; w++) { }
            Q_DT_U8 = p[i];
        }
        qspi_finish();
        qspi_wait_wip();                          /* wait for the chip to finish programming */
        addr += chunk; p += chunk; len -= chunk;
    }
    return 0;
}

int qspi_flash_erase4k(uint32_t addr)
{
    if (!s_size || addr >= s_size) return -1;
    qspi_wren();
    qspi_kick(OP_SECTORERASE, addr & ~0xFFFu, 3, 0, 0, OM_111, true, false, 0);
    qspi_finish();
    qspi_wait_wip();                              /* wait for the chip to finish erasing */
    return 0;
}
