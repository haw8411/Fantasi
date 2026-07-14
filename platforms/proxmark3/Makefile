# Fantasi / Proxmark3 platform build.
#
# AT91SAM7S256/512 is ARM7TDMI - ARMv4T, not Cortex-M. No thumb-2, no
# FPU, mixed ARM/Thumb allowed. FreeRTOS's GCC/ARM7_AT91SAM7S port
# covers it. USB goes through TinyUSB + our own dcd_at91sam7s DCD.

TARGET    := fantasi-proxmark3
ROOT      := $(abspath ../..)
BUILD     := $(ROOT)/build/proxmark3

CROSS     ?= arm-none-eabi-
CC        := $(CROSS)gcc
AS        := $(CROSS)gcc
OBJCOPY   := $(CROSS)objcopy
SIZE      := $(CROSS)size

MCU_FLAGS := -mcpu=arm7tdmi -mthumb-interwork

GIT_HASH := $(shell git rev-parse --short=7 HEAD 2>/dev/null || echo unknown)$(shell git diff --quiet 2>/dev/null || echo -dirty)

DEFS := -DCFG_TUSB_MCU=OPT_MCU_AT91SAM7S \
        -DFANTASI_ENABLE_MSC \
        -DFANTASI_ENABLE_APPS \
        -DFANTASI_ENABLE_BERRY \
        -DFANTASI_ENABLE_WEBUSB \
        -DFANTASI_FAT_STORAGE \
        -DLFS_NO_MALLOC \
        -DFANTASI_GIT_HASH=\"$(GIT_HASH)\" \
        -DCLI_TASK_STACK=1024 \
        -DUSB_TASK_STACK=512 \
        -DWEBUSB_PROTO_STACK=1024
        # PM3 task stacks right-sized from measured high-water marks (usbproto/cli
        # ~416/214 words peak, usb ~72) - was 2048 each; frees ~14 KB of the heap.

# platforms/proxmark3 comes first on the include path so our
# platform-specific tusb_config.h wins over hal/tinyusb's.
INCLUDES := \
    -I$(ROOT)/core \
    -I$(ROOT)/apps \
    -I$(ROOT)/hal \
    -I$(ROOT)/hal/storage \
    -I$(ROOT)/platforms/proxmark3 \
    -I$(ROOT)/hal/tinyusb \
    -I$(ROOT)/third_party/tinyusb/src \
    -I$(ROOT)/third_party/tinyusb/src/portable/microchip/at91sam7s \
    -I$(ROOT)/third_party/FreeRTOS-Kernel/include \
    -I$(ROOT)/third_party/FreeRTOS-Kernel/portable/GCC/ARM7_AT91SAM7S \
    -I$(ROOT)/third_party/littlefs \
    -I$(ROOT)/third_party/nanopb \
    -I$(ROOT)/proto \
    -I$(ROOT)/third_party/berry \
    -I$(ROOT)/third_party/berry/src

# ARM mode only for ISR-adjacent code. Core + CLI use ARM by default
# (safer; Thumb would need -mthumb flag and interwork).
CFLAGS := $(MCU_FLAGS) $(DEFS) $(INCLUDES) \
          -Os -ffunction-sections -fdata-sections -fno-common \
          -Wall -Wextra -Wno-unused-parameter \
          -std=gnu11 -g3

ASFLAGS := $(MCU_FLAGS) -g3

LDSCRIPT := $(ROOT)/platforms/proxmark3/linker.ld
LDFLAGS := $(MCU_FLAGS) -T$(LDSCRIPT) \
           -Wl,--gc-sections \
           -Wl,-Map=$(BUILD)/$(TARGET).map \
           -Wl,--wrap=_malloc_r,--wrap=_free_r,--wrap=_calloc_r,--wrap=_realloc_r \
           -nostartfiles --specs=nano.specs --specs=nosys.specs \
           -u _printf_float -lm

FRTOS_PORT := $(ROOT)/third_party/FreeRTOS-Kernel/portable/GCC/ARM7_AT91SAM7S
TUSB_DIR := $(ROOT)/third_party/tinyusb/src

SRCS_C := \
    $(ROOT)/core/main.c \
    $(ROOT)/core/cli.c \
    $(wildcard $(ROOT)/core/commands/*.c) \
    $(ROOT)/core/log.c \
    $(ROOT)/core/newlib_malloc.c \
    $(ROOT)/platforms/proxmark3/hal.c \
    $(ROOT)/platforms/proxmark3/flash_storage.c \
    $(ROOT)/core/ramfs.c \
    $(ROOT)/core/vfs.c \
    $(ROOT)/core/elf_loader.c \
    $(ROOT)/core/app_run.c \
    $(ROOT)/core/berry_host.c \
    $(ROOT)/core/usb_proto.c \
    $(ROOT)/core/ble_proto.c \
    $(ROOT)/core/ble_pipe.c \
    $(ROOT)/third_party/nanopb/pb_encode.c \
    $(ROOT)/third_party/nanopb/pb_decode.c \
    $(ROOT)/third_party/nanopb/pb_common.c \
    $(ROOT)/proto/fantasi.pb.c \
    $(ROOT)/hal/storage/hal_storage.c \
    $(ROOT)/hal/storage/msc_device.c \
    $(ROOT)/hal/storage/fat_ramdisk.c \
    $(ROOT)/platforms/proxmark3/usb_descriptors.c \
    $(TUSB_DIR)/tusb.c \
    $(TUSB_DIR)/common/tusb_fifo.c \
    $(TUSB_DIR)/device/usbd.c \
    $(TUSB_DIR)/device/usbd_control.c \
    $(TUSB_DIR)/class/cdc/cdc_device.c \
    $(TUSB_DIR)/class/msc/msc_device.c \
    $(TUSB_DIR)/class/hid/hid_device.c \
    $(TUSB_DIR)/class/vendor/vendor_device.c \
    $(TUSB_DIR)/portable/microchip/at91sam7s/dcd_at91sam7s.c \
    $(ROOT)/third_party/FreeRTOS-Kernel/tasks.c \
    $(ROOT)/third_party/FreeRTOS-Kernel/queue.c \
    $(ROOT)/third_party/FreeRTOS-Kernel/list.c \
    $(ROOT)/third_party/FreeRTOS-Kernel/timers.c \
    $(ROOT)/third_party/FreeRTOS-Kernel/stream_buffer.c \
    $(FRTOS_PORT)/port.c \
    $(FRTOS_PORT)/portISR.c \
    $(FRTOS_PORT)/lib_AT91SAM7X256.c \
    $(ROOT)/third_party/FreeRTOS-Kernel/portable/MemMang/heap_4.c

LFS_DIR := $(ROOT)/third_party/littlefs
SRCS_C += $(LFS_DIR)/lfs.c $(LFS_DIR)/lfs_util.c

# Berry scripting VM. Vendored in third_party/berry/.
BERRY_DIR := $(ROOT)/third_party/berry
ifneq (,$(wildcard $(BERRY_DIR)/src/be_vm.c))
SRCS_C += $(wildcard $(BERRY_DIR)/src/*.c)
endif

SRCS_S := $(ROOT)/platforms/proxmark3/startup.S

OBJS := $(patsubst $(ROOT)/%.c,$(BUILD)/%.o,$(SRCS_C)) \
        $(patsubst $(ROOT)/%.S,$(BUILD)/%.o,$(SRCS_S))

.PHONY: all clean size
all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin size

$(BUILD)/%.o: $(ROOT)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: $(ROOT)/%.S
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	@mkdir -p $(BUILD)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

size: $(BUILD)/$(TARGET).elf
	$(SIZE) $<

clean:
	rm -rf $(BUILD)
