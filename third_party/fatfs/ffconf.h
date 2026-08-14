/*---------------------------------------------------------------------------/
/  Fantasi FatFs configuration (R0.15) for the Flipper microSD (/mnt/ext0).
/
/  Read/write FAT12/16/32, long filenames on (the test uses 9-char dir names
/  like "do_serial"), format-on-fail enabled, one 512-byte volume, no exFAT,
/  no RTC timestamps, single-threaded (the storage task is the only caller).
/---------------------------------------------------------------------------*/

#define FFCONF_DEF	80286	/* Revision ID (R0.15) */

/*---- Function Configurations ----*/
#define FF_FS_READONLY	0
#define FF_FS_MINIMIZE	0
#define FF_USE_FIND		0
#define FF_USE_MKFS		1	/* f_mkfs: format a blank/foreign card */
#define FF_USE_FASTSEEK	0
#define FF_USE_EXPAND	0
#define FF_USE_CHMOD	0
#define FF_USE_LABEL	0
#define FF_USE_FORWARD	0
#define FF_USE_STRFUNC	0
#define FF_PRINT_LLI	0
#define FF_PRINT_FLOAT	0
#define FF_STRF_ENCODE	3

/*---- Locale and Namespace Configurations ----*/
#define FF_CODE_PAGE	437	/* U.S. */
#define FF_USE_LFN		1	/* 1: LFN with a static working buffer (single task) */
#define FF_MAX_LFN		255
#define FF_LFN_UNICODE	0	/* 0: ANSI/OEM in SBCS/DBCS */
#define FF_LFN_BUF		255
#define FF_SFN_BUF		12
#define FF_FS_RPATH		0	/* absolute "0:/path" only - no chdir/getcwd */

/*---- Drive/Volume Configurations ----*/
#define FF_VOLUMES		1
#define FF_STR_VOLUME_ID	0
#define FF_MULTI_PARTITION	0
#define FF_MIN_SS		512
#define FF_MAX_SS		512
#define FF_LBA64		0
#define FF_MIN_GPT		0x10000000
#define FF_USE_TRIM		0

/*---- System Configurations ----*/
#define FF_FS_TINY		0
#define FF_FS_EXFAT		0
#define FF_FS_NORTC		1	/* no RTC: fixed timestamp on new files */
#define FF_NORTC_MON	1
#define FF_NORTC_MDAY	1
#define FF_NORTC_YEAR	2025
#define FF_FS_NOFSINFO	0
#define FF_FS_LOCK		0
#define FF_FS_REENTRANT	0
#define FF_FS_TIMEOUT	1000
