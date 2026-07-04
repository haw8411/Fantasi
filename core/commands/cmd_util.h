#ifndef CORE_COMMANDS_CMD_UTIL_H
#define CORE_COMMANDS_CMD_UTIL_H

/* Drain the CDC TX ring before resetting or switching USB modes. tud_cdc_write
 * has queued the final line into TinyUSB's FIFO but the host won't see it until
 * the next SOF-driven IN poll; ~100 ms is comfortable at Full-Speed. Shared by
 * the reboot / dfu / msc commands. */
void flush_before_reset(void);

#endif
