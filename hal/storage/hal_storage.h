#ifndef FANTASI_HAL_STORAGE_H
#define FANTASI_HAL_STORAGE_H

#include <stdint.h>
#include <stdbool.h>

int  hal_storage_init(void);
void hal_storage_unmount(void);
int  hal_storage_mount(void);
bool hal_storage_mounted(void);

/* Read an entire file into buf. Returns bytes read, or -1 on error. */
int  hal_storage_read_file(const char *path, void *buf, uint32_t max_len);

/* Write an entire file. Returns bytes written, or -1 on error. */
int  hal_storage_write_file(const char *path, const void *buf, uint32_t len);

/* Return the LFS instance (mounting on demand). NULL on failure. */
struct lfs *hal_storage_lfs(void);

#endif
