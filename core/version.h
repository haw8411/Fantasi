#ifndef FANTASI_VERSION_H
#define FANTASI_VERSION_H

/* Project version - CalVer: yy.mm.pp
 *   yy = two-digit year, mm = month (no leading zero), pp = patch within that
 *   month (reset to 0 each new yy.mm). Bump pp for every release in a month.
 * Each release also gets a codename for easy human identification. */
#define FANTASI_VERSION  "26.7.0"
#define FANTASI_CODENAME "Avalon"

/* Short git commit, injected by the build (-DFANTASI_GIT_HASH); falls back to
 * "unknown" for builds made outside a git checkout. */
#ifndef FANTASI_GIT_HASH
#define FANTASI_GIT_HASH "unknown"
#endif

#endif /* FANTASI_VERSION_H */
