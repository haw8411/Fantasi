/* App-facing RFID API: builds the fantasi_rfid_t table over the HAL + core/rfid
 * protocol layer, and resolves the single `fantasi_rfid` symbol for the ELF
 * loader (core/elf_loader.c calls rfid_resolve_api via a weak hook, exactly like
 * berry_resolve_api). Only compiled where FANTASI_ENABLE_RFID is set, so on
 * non-RFID builds the weak hook returns 0 and apps see no RFID symbol. */
#include "rfid.h"
#include "../app_run.h"
#include "../../hal/hal_rfid.h"
#include "app_rfid.h"

#include <string.h>

/* Flat-signature adapter over core rfid_iso14443a_select (which uses a struct). */
static int api_iso14443a_select(uint8_t uid[10], int *uid_len,
                                uint8_t *sak, uint8_t atqa[2], int *cascade)
{
    rfid_iso14443a_tag_t t;
    int rc = rfid_iso14443a_select(&t);
    if (rc != 0) return rc;
    if (uid)     memcpy(uid, t.uid, sizeof(t.uid));
    if (uid_len) *uid_len = t.uid_len;
    if (sak)     *sak = t.sak;
    if (atqa)  { atqa[0] = t.atqa[0]; atqa[1] = t.atqa[1]; }
    if (cascade) *cascade = t.cascade;
    return 0;
}

static int api_set_mode(int mode)
{
    void *gate = app_api_gate_enter();
    int rc = hal_rfid_set_mode((rfid_mode_t)mode);
    app_api_gate_leave(gate);
    return rc;
}
static void api_field(int on)      { hal_rfid_field(on != 0); }

/* Loadable-gateware entries weak-default to "no FPGA" here, so chip-based
 * platforms (Chameleon, Flipper) link without stubs; the Proxmark3's strong
 * definitions override them. See hal_rfid.h. */
__attribute__((weak)) const char *hal_rfid_fpga_resource(rfid_mode_t mode) { (void)mode; return NULL; }
__attribute__((weak)) int  hal_rfid_fpga_load(const char *path) { (void)path; return RFID_ERR_UNSUPP; }

static const char *api_fpga_resource(int mode) { return hal_rfid_fpga_resource((rfid_mode_t)mode); }
static int  api_fpga_load(const char *path)    { return hal_rfid_fpga_load(path); }

/* Passive HF sniff - weak-defaults to unsupported; only platforms that can passively
 * demodulate an external field (Flipper ST25R3916 transparent mode) override it. */
__attribute__((weak)) int hal_rfid_hf_sniff(const char *path, uint32_t timeout_ms) { (void)path; (void)timeout_ms; return RFID_ERR_UNSUPP; }
static int api_hf_sniff(const char *path, uint32_t timeout_ms) { return hal_rfid_hf_sniff(path, timeout_ms); }

/* Caller-buffer sniff (ABI 5) - weak-defaults to unsupported so platforms that only implement the
 * file-based hf_sniff (Flipper) link without a stub; the Proxmark3's strong definition overrides it. */
__attribute__((weak)) int hal_rfid_hf_sniff_capture(uint8_t *buf, uint32_t cap, uint32_t quiet_ms, uint32_t max_ms)
{ (void)buf; (void)cap; (void)quiet_ms; (void)max_ms; return RFID_ERR_UNSUPP; }
static int api_hf_sniff_capture(uint8_t *buf, uint32_t cap, uint32_t quiet_ms, uint32_t max_ms)
{ return hal_rfid_hf_sniff_capture(buf, cap, quiet_ms, max_ms); }

/* Custom-parity HF transceive (MIFARE Crypto1 encrypted frames) - weak-defaults to unsupported so
 * frontends that can't supply per-byte parity link without a stub; the Proxmark3's strong definition
 * overrides it. */
__attribute__((weak)) int hal_rfid_hf_transceive_par(const uint8_t *tx, int nbytes, const uint8_t *par,
                                                     uint8_t *rx, uint8_t *rx_par, int rx_cap, uint32_t timeout_us)
{ (void)tx; (void)nbytes; (void)par; (void)rx; (void)rx_par; (void)rx_cap; (void)timeout_us; return RFID_ERR_UNSUPP; }
static int api_hf_transceive_par(const uint8_t *tx, int nbytes, const uint8_t *par,
                                 uint8_t *rx, uint8_t *rx_par, int rx_cap, uint32_t timeout_us)
{ return hal_rfid_hf_transceive_par(tx, nbytes, par, rx, rx_par, rx_cap, timeout_us); }

/* LF reader->tag downlink - weak-defaults to unsupported so chip targets without LF transmit
 * (Chameleon/Flipper) link; the Proxmark3's strong hal_rfid_lf_modulate overrides it. */
__attribute__((weak)) int hal_rfid_lf_modulate(const uint8_t *bits, int nbits, uint32_t opts)
{ (void)bits; (void)nbits; (void)opts; return RFID_ERR_UNSUPP; }
static int api_lf_modulate(const uint8_t *bits, int nbits, uint32_t opts) { return hal_rfid_lf_modulate(bits, nbits, opts); }

/* LF raw capture (send optional downlink + capture envelope) - weak-defaults to unsupported so
 * LF-transmit-only or non-LF platforms link without a stub; the Proxmark3's strong
 * hal_rfid_lf_transceive overrides it. */
__attribute__((weak)) int hal_rfid_lf_transceive(const uint8_t *cmd, int nbits, uint8_t *buf, int cap)
{ (void)cmd; (void)nbits; (void)buf; (void)cap; return RFID_ERR_UNSUPP; }
static int api_lf_transceive(const uint8_t *cmd, int nbits, uint8_t *buf, int cap)
{ return hal_rfid_lf_transceive(cmd, nbits, buf, cap); }

/* HF tag emulation (card side) - weak-defaults to unsupported so non-emulating frontends link without
 * stubs; the Proxmark3's strong definitions override them. */
__attribute__((weak)) int hal_rfid_hf_emu_recv(uint8_t *rx, uint8_t *rx_par, int cap, uint32_t timeout_ms)
{ (void)rx; (void)rx_par; (void)cap; (void)timeout_ms; return RFID_ERR_UNSUPP; }
__attribute__((weak)) int hal_rfid_hf_emu_send(const uint8_t *tosend, int len)
{ (void)tosend; (void)len; return RFID_ERR_UNSUPP; }
__attribute__((weak)) int hal_rfid_hf_emu_send_stream(uint8_t (*next)(void *), void *ctx, int nsymbols)
{ (void)next; (void)ctx; (void)nsymbols; return RFID_ERR_UNSUPP; }
__attribute__((weak)) int hal_rfid_hf_emu_send_stream_buf(const uint8_t *tosend, int len)
{ (void)tosend; (void)len; return RFID_ERR_UNSUPP; }
__attribute__((weak)) int hal_rfid_hf_emu_recv_progress(
    uint8_t *rx, uint8_t *rx_par, int cap, uint32_t timeout_ms,
    void (*data_ready)(void *, int, uint8_t, int), void *ctx)
{
    (void)rx; (void)rx_par; (void)cap; (void)timeout_ms; (void)data_ready; (void)ctx;
    return RFID_ERR_UNSUPP;
}
__attribute__((weak)) int hal_rfid_hf_emu_prepare(const uint8_t *tosend, int len)
{ (void)tosend; (void)len; return RFID_ERR_UNSUPP; }
__attribute__((weak)) int hal_rfid_hf_emu_send_stream_match(
    uint8_t (*next)(void *), void *ctx, int nsymbols,
    const uint8_t *request, int request_len, int request_min_len, int *result)
{
    (void)next; (void)ctx; (void)nsymbols; (void)request; (void)request_len;
    (void)request_min_len;
    if (result) *result = 0;
    return RFID_ERR_UNSUPP;
}
static int api_hf_emu_recv(uint8_t *rx, uint8_t *rx_par, int cap, uint32_t timeout_ms)
{ return hal_rfid_hf_emu_recv(rx, rx_par, cap, timeout_ms); }
static int api_hf_emu_send(const uint8_t *tosend, int len)
{ return hal_rfid_hf_emu_send(tosend, len); }
static int api_hf_emu_send_stream(uint8_t (*next)(void *), void *ctx, int nsymbols)
{ return hal_rfid_hf_emu_send_stream(next, ctx, nsymbols); }
static int api_hf_emu_send_stream_buf(const uint8_t *tosend, int len)
{ return hal_rfid_hf_emu_send_stream_buf(tosend, len); }
static const fantasi_rfid_t g_rfid = {
    .abi              = FANTASI_RFID_ABI,
    .caps             = hal_rfid_caps,
    .set_mode         = api_set_mode,
    .field            = api_field,
    .hf_transceive    = hal_rfid_hf_transceive,
    .hf_transceive_par = api_hf_transceive_par,
    .iso14443a_select = api_iso14443a_select,
    .hf_probe         = hal_rfid_hf_probe,
    .lf_em4100        = rfid_lf_em4100_read,
    .lf_modulate      = api_lf_modulate,
    .lf_transceive    = api_lf_transceive,
    .fpga_resource    = api_fpga_resource,
    .fpga_load        = api_fpga_load,
    .hf_sniff         = api_hf_sniff,
    .hf_sniff_capture = api_hf_sniff_capture,
    .hf_emu_recv      = api_hf_emu_recv,
    .hf_emu_send      = api_hf_emu_send,
    .hf_emu_send_stream = api_hf_emu_send_stream,
    .hf_emu_send_stream_buf = api_hf_emu_send_stream_buf,
    .hf_emu_recv_progress = hal_rfid_hf_emu_recv_progress,
    .hf_emu_prepare = hal_rfid_hf_emu_prepare,
    .hf_emu_send_stream_match = hal_rfid_hf_emu_send_stream_match,
};

const fantasi_rfid_t *fantasi_rfid(void) { return &g_rfid; }

/* ELF-loader symbol resolver hook (weak default lives in core/elf_loader.c). */
uint32_t rfid_resolve_api(const char *name)
{
    if (strcmp(name, "fantasi_rfid") == 0)
        return (uint32_t)(uintptr_t)fantasi_rfid;
    return 0;
}
