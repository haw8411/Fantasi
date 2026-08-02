# Build a Fantasi app into loadable ELF(s):  make APP=hello
#
# Apps are freestanding, relocatable ARM objects (one ET_REL object, relocations
# collapsed to R_ARM_ABS32 + the harmless R_ARM_V4BX marker). The binary is
# architecture-specific, so this builds one ELF per supported core:
#
#   build/apps/<name>.cm4.elf   Cortex-M4 / ARMv7E-M  (Flipper, Chameleon)
#   build/apps/<name>.arm7.elf  ARM7TDMI / ARMv4T     (Proxmark3)
#
# Every *.c in apps/<APP>/ is built into its OWN ELF (basename), so an app folder
# can hold a main program plus the feature modules it hot-loads at runtime - e.g.
# apps/rfid/ builds rfid (the driver) + hf + lf (the modules it streams into RAM).
# A single-file app (apps/hello/hello.c) just builds the one ELF as before.
# Upload the variant matching your target to /ramfs or /apps, then `launch` it.

ROOT  := $(abspath ..)
BUILD := $(ROOT)/build/apps
APP   ?=

CROSS ?= arm-none-eabi-
CC    := $(CROSS)gcc
LD    := $(CROSS)ld

# Flags shared by every variant. -mword-relocations + -mlong-calls force all
# address references (data and calls) through relocated literals, so the loader
# only ever sees R_ARM_ABS32 (plus R_ARM_V4BX on ARMv4T, which it ignores). The
# optimisation level is per-core, NOT here: the arm7 (Proxmark3) modules load into
# a tight ramfs heap so they stay -Os for size, while the cm4 (Chameleon) has ample
# RAM and uses -O2 for speed - it materially lowers the tag-emulation FDT by making
# the per-symbol reply producer faster (full inlining). -O2 not -O3: measured the
# SAME FDT as -O3 here (the crypto is already fast enough that unrolling buys nothing)
# for ~half the code size, so the modules stay small and load with headroom.
COMMON := -ffreestanding -fno-common -mword-relocations -mlong-calls \
          -ffunction-sections -fdata-sections -nostdlib -Wall -I$(ROOT)/apps \
          -I$(ROOT)/third_party/berry -I$(ROOT)/third_party/berry/src

# Per-core flags. Cortex-M is Thumb-2 + hard FP; ARM7TDMI is ARM-mode + interwork.
# -DEMU_STREAM_BUF (cm4 only): compile the mfc_emu crypto-reply BUFFER fallback, needed by CPU-bit-banged
# frontends (Flipper) that can't overlap the producer with TX. The ARM7 build (Proxmark3, RAM-tight) omits it,
# so its module carries only the streaming path (no ~0.6 KB pre-encode bloat). The cm4 binary also serves the
# Chameleon, which streams (path unused there) but has ample RAM.
CM4_FLAGS  := -O2 -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DEMU_STREAM_BUF
ARM7_FLAGS := -Os -mcpu=arm7tdmi -mthumb-interwork

.PHONY: app
app:
	@test -n "$(APP)" || { echo "usage: make app APP=<name>  (source: apps/<name>/*.c)"; exit 1; }
	@test -d $(ROOT)/apps/$(APP) || { echo "error: apps/$(APP)/ not found"; exit 1; }
	@ls $(ROOT)/apps/$(APP)/*.c >/dev/null 2>&1 || { echo "error: no *.c in apps/$(APP)/"; exit 1; }
	@mkdir -p $(BUILD)
	@for src in $(ROOT)/apps/$(APP)/*.c; do \
	   base=$$(basename $$src .c); \
	   $(MAKE) --no-print-directory _variant SRC=$$src BASE=$$base VARIANT=cm4  VFLAGS="$(CM4_FLAGS)" || exit 1; \
	   $(MAKE) --no-print-directory _variant SRC=$$src BASE=$$base VARIANT=arm7 VFLAGS="$(ARM7_FLAGS)" || exit 1; \
	 done
	@echo "built app '$(APP)': $$(ls $(ROOT)/apps/$(APP)/*.c | xargs -n1 basename | sed 's/\.c$$//' | tr '\n' ' ')-> $(BUILD)/*.{cm4,arm7}.elf"
	@echo "upload the matching variant(s) to /ramfs or /apps, then: launch <path>"

# Internal: build one source file, one architecture variant, into its own ELF.
.PHONY: _variant
_variant:
	$(CC) $(VFLAGS) $(COMMON) -c $(SRC) -o $(BUILD)/$(BASE).$(VARIANT).o
	$(LD) -r -T $(ROOT)/apps/app.ld $(BUILD)/$(BASE).$(VARIANT).o -o $(BUILD)/$(BASE).$(VARIANT).elf
	@rm -f $(BUILD)/$(BASE).$(VARIANT).o
