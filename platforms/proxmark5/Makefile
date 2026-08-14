# Fantasi / Proxmark5 platform build.
#
# AT32F435 is a Cortex-M4F (ARMv7E-M, hard-float FPU) at up to 288 MHz.
# FreeRTOS's GCC/ARM_CM4F port covers it (same as Flipper/Chameleon).
# USB goes through TinyUSB's Synopsys DWC2 device driver - the AT32 OTGFS
# core is DWC2-compatible; support is added in third_party/tinyusb_patches
# (OPT_MCU_AT32F435 + dwc2_at32.h). The RFID FPGA frontend and ESP32-C2
# BLE are not implemented yet: hal_rfid_* weak-default to unsupported and the
# hal_ble_* stubs return sentinels, so the CLI degrades at runtime.

TARGET    := fantasi-proxmark5
ROOT      := $(abspath ../..)
BUILD     := $(ROOT)/build/proxmark5

CROSS     ?= arm-none-eabi-
CC        := $(CROSS)gcc
OBJCOPY   := $(CROSS)objcopy
SIZE      := $(CROSS)size

MCU_FLAGS := -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16

GIT_HASH := $(shell git rev-parse --short=7 HEAD 2>/dev/null || echo unknown)$(shell git diff --quiet 2>/dev/null || echo -dirty)

DEFS := -DAT32F435xx \
        -DCFG_TUSB_MCU=OPT_MCU_AT32F435 \
        -DFANTASI_ENABLE_MSC \
        -DFANTASI_ENABLE_APPS \
        -DFANTASI_ENABLE_BERRY \
        -DFANTASI_ENABLE_WEBUSB \
        -DFANTASI_ENABLE_RFID \
        -DFANTASI_FAT_STORAGE \
        -DLFS_NO_MALLOC \
        -DLFS_NO_DEBUG -DLFS_NO_WARN -DLFS_NO_ERROR \
        -DFANTASI_GIT_HASH=\"$(GIT_HASH)\"

INCLUDES := \
    -I$(ROOT)/core \
    -I$(ROOT)/apps \
    -I$(ROOT)/hal \
    -I$(ROOT)/hal/storage \
    -I$(ROOT)/platforms/proxmark5 \
    -I$(ROOT)/hal/tinyusb \
    -I$(ROOT)/third_party/cmsis_core \
    -I$(ROOT)/third_party/FreeRTOS-Kernel/include \
    -I$(ROOT)/third_party/FreeRTOS-Kernel/portable/GCC/ARM_CM4F \
    -I$(ROOT)/third_party/tinyusb/src \
    -I$(ROOT)/third_party/littlefs \
    -I$(ROOT)/third_party/nanopb \
    -I$(ROOT)/proto \
    -I$(ROOT)/third_party/berry \
    -I$(ROOT)/third_party/berry/src

CFLAGS := $(MCU_FLAGS) $(DEFS) $(INCLUDES) \
          -Os -ffunction-sections -fdata-sections -fno-common \
          -Wall -Wextra -Wno-unused-parameter \
          -std=gnu11 -g3 \
          --specs=picolibc.specs

LDSCRIPT := $(ROOT)/platforms/proxmark5/linker.ld
LDFLAGS := $(MCU_FLAGS) -T$(LDSCRIPT) \
           -Wl,--gc-sections \
           -Wl,-Map=$(BUILD)/$(TARGET).map \
           -Wl,--wrap=malloc,--wrap=free,--wrap=calloc,--wrap=realloc \
           -Wl,--defsym=vfprintf=__d_vfprintf -Wl,--defsym=vfscanf=__d_vfscanf \
           -nostartfiles --specs=picolibc.specs

SRCS := \
    $(ROOT)/core/main.c \
    $(ROOT)/core/cli.c \
    $(wildcard $(ROOT)/core/commands/*.c) \
    $(ROOT)/core/proto.c \
    $(ROOT)/core/usb_proto.c \
    $(ROOT)/core/proto_pipe.c \
    $(ROOT)/core/ramfs.c \
    $(ROOT)/core/vfs.c \
    $(ROOT)/core/elf_loader.c \
    $(ROOT)/core/app_run.c \
    $(ROOT)/core/log.c \
    $(ROOT)/core/libc_glue.c \
    $(ROOT)/core/lzss.c \
    $(ROOT)/core/berry_host.c \
    $(ROOT)/platforms/proxmark5/startup.c \
    $(ROOT)/platforms/proxmark5/system.c \
    $(ROOT)/platforms/proxmark5/hal.c \
    $(ROOT)/platforms/proxmark5/flash_storage.c \
    $(ROOT)/platforms/proxmark5/rfid.c \
    $(ROOT)/platforms/proxmark5/qspi_flash.c \
    $(ROOT)/platforms/proxmark5/ext_storage.c \
    $(ROOT)/core/rfid/rfid_crc.c \
    $(ROOT)/core/rfid/rfid_iso14443a.c \
    $(ROOT)/core/rfid/rfid_em4100.c \
    $(ROOT)/core/rfid/rfid_api.c \
    $(ROOT)/hal/storage/hal_storage.c \
    $(ROOT)/hal/storage/msc_device.c \
    $(ROOT)/hal/storage/fat_ramdisk.c \
    $(ROOT)/hal/tinyusb/usb_descriptors.c \
    $(ROOT)/hal/tinyusb/hal_serial_tinyusb.c \
    $(ROOT)/third_party/FreeRTOS-Kernel/tasks.c \
    $(ROOT)/third_party/FreeRTOS-Kernel/queue.c \
    $(ROOT)/third_party/FreeRTOS-Kernel/list.c \
    $(ROOT)/third_party/FreeRTOS-Kernel/timers.c \
    $(ROOT)/third_party/FreeRTOS-Kernel/stream_buffer.c \
    $(ROOT)/third_party/FreeRTOS-Kernel/portable/GCC/ARM_CM4F/port.c \
    $(ROOT)/third_party/FreeRTOS-Kernel/portable/MemMang/heap_4.c \
    $(ROOT)/third_party/nanopb/pb_encode.c \
    $(ROOT)/third_party/nanopb/pb_decode.c \
    $(ROOT)/third_party/nanopb/pb_common.c \
    $(ROOT)/proto/fantasi.pb.c

TUSB_DIR := $(ROOT)/third_party/tinyusb/src
ifneq (,$(wildcard $(TUSB_DIR)/tusb.c))
SRCS += \
    $(TUSB_DIR)/tusb.c \
    $(TUSB_DIR)/common/tusb_fifo.c \
    $(TUSB_DIR)/device/usbd.c \
    $(TUSB_DIR)/device/usbd_control.c \
    $(TUSB_DIR)/class/cdc/cdc_device.c \
    $(TUSB_DIR)/class/msc/msc_device.c \
    $(TUSB_DIR)/class/hid/hid_device.c \
    $(TUSB_DIR)/class/vendor/vendor_device.c \
    $(TUSB_DIR)/portable/synopsys/dwc2/dcd_dwc2.c
endif

LFS_DIR := $(ROOT)/third_party/littlefs
ifneq (,$(wildcard $(LFS_DIR)/lfs.c))
SRCS += $(LFS_DIR)/lfs.c $(LFS_DIR)/lfs_util.c
endif

# Berry scripting VM. Vendored in third_party/berry/.
BERRY_DIR := $(ROOT)/third_party/berry
ifneq (,$(wildcard $(BERRY_DIR)/src/be_vm.c))
SRCS += $(wildcard $(BERRY_DIR)/src/*.c)
endif

OBJS := $(patsubst $(ROOT)/%.c,$(BUILD)/%.o,$(SRCS))

.PHONY: all clean size
all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin size

$(BUILD)/%.o: $(ROOT)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

# Header dependencies (see the note in platforms/chameleon/Makefile).
-include $(OBJS:.o=.d)

$(BUILD)/$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	@mkdir -p $(BUILD)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

size: $(BUILD)/$(TARGET).elf
	$(SIZE) $<

clean:
	rm -rf $(BUILD)
