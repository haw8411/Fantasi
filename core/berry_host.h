/* berry_host - firmware-side entry points for the embedded Berry VM.
 * Apps that build their own VM (to register native modules) use Berry's C API
 * directly (berry.h, resolved by the ELF loader); this header is for the simple
 * firmware-level runners used by `launch` and the app ABI's be_exec. */
#ifndef CORE_BERRY_HOST_H
#define CORE_BERRY_HOST_H

/* Load + run a Berry script from a VFS path on a fresh VM. The simple entry for
 * `launch foo.be` and script-only apps. Returns 0, or an error code. */
int be_exec(const char *path);

#endif /* CORE_BERRY_HOST_H */
