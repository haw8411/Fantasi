# Build a single Fantasi app into loadable ELF(s):  make APP=hello
#
# Apps are freestanding, relocatable ARM objects (one ET_REL object, relocations
# collapsed to R_ARM_ABS32 + the harmless R_ARM_V4BX marker). The binary is
# architecture-specific, so this builds one ELF per supported core:
#
#   build/apps/<name>.cm4.elf   Cortex-M4 / ARMv7E-M  (Flipper, Chameleon)
#   build/apps/<name>.arm7.elf  ARM7TDMI / ARMv4T     (Proxmark3)
#
# Upload the variant matching your target to /ramfs or /apps, then `launch` it.
# Source lives in apps/<name>/<name>.c.

ROOT  := $(abspath ..)
BUILD := $(ROOT)/build/apps
APP   ?=

CROSS ?= arm-none-eabi-
CC    := $(CROSS)gcc
LD    := $(CROSS)ld

# Flags shared by every variant. -mword-relocations + -mlong-calls force all
# address references (data and calls) through relocated literals, so the loader
# only ever sees R_ARM_ABS32 (plus R_ARM_V4BX on ARMv4T, which it ignores).
COMMON := -Os -ffreestanding -fno-common -mword-relocations -mlong-calls \
          -ffunction-sections -fdata-sections -nostdlib -Wall -I$(ROOT)/apps \
          -I$(ROOT)/third_party/berry -I$(ROOT)/third_party/berry/src

# Per-core flags. Cortex-M is Thumb-2 + hard FP; ARM7TDMI is ARM-mode + interwork.
CM4_FLAGS  := -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
ARM7_FLAGS := -mcpu=arm7tdmi -mthumb-interwork

.PHONY: app
app:
	@test -n "$(APP)" || { echo "usage: make app APP=<name>  (source: apps/<name>/<name>.c)"; exit 1; }
	@test -f $(ROOT)/apps/$(APP)/$(APP).c || { echo "error: apps/$(APP)/$(APP).c not found"; exit 1; }
	@mkdir -p $(BUILD)
	@$(MAKE) --no-print-directory _variant APP=$(APP) VARIANT=cm4  VFLAGS="$(CM4_FLAGS)"
	@$(MAKE) --no-print-directory _variant APP=$(APP) VARIANT=arm7 VFLAGS="$(ARM7_FLAGS)"
	@echo "built $(BUILD)/$(APP).cm4.elf (FZ/CU) and $(BUILD)/$(APP).arm7.elf (PM3)"
	@echo "upload the matching variant to /ramfs or /apps, then: launch <path>"

# Internal: build one architecture variant.
.PHONY: _variant
_variant:
	$(CC) $(VFLAGS) $(COMMON) -c $(ROOT)/apps/$(APP)/$(APP).c -o $(BUILD)/$(APP).$(VARIANT).o
	$(LD) -r -T $(ROOT)/apps/app.ld $(BUILD)/$(APP).$(VARIANT).o -o $(BUILD)/$(APP).$(VARIANT).elf
	@rm -f $(BUILD)/$(APP).$(VARIANT).o
