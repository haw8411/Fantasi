# Fantasi / Flipper Zero platform build.
# Invoked from the top-level Makefile via `make PLATFORM=flipper`.
#
# The Kiisu is the same STM32WB55 board with an identical peripheral pinout,
# so platforms/kiisu/Makefile reuses this whole build verbatim - it only
# overrides TARGET, BUILD, and DEVICE_ID before `include`-ing this file.
# Hence the `?=` on those three: a straight `make PLATFORM=flipper` gets the
# Flipper defaults; the Kiisu include supplies its own first.

TARGET    ?= fantasi-flipper
ROOT      := $(abspath ../..)
BUILD     ?= $(ROOT)/build/flipper
DEVICE_ID ?= FZ

CROSS     ?= arm-none-eabi-
CC        := $(CROSS)gcc
OBJCOPY   := $(CROSS)objcopy
SIZE      := $(CROSS)size

MCU_FLAGS := -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16

# STM32WB55xx selects the right CMSIS device header; CFG_TUSB_MCU tells
# TinyUSB to build the STM32 FSDEV driver path.
# Short git commit baked into the version string (appends -dirty if the tree
# has uncommitted changes). "unknown" outside a git checkout.
GIT_HASH := $(shell git rev-parse --short=7 HEAD 2>/dev/null || echo unknown)$(shell git diff --quiet 2>/dev/null || echo -dirty)

DEFS := -DSTM32WB55xx \
        -DCFG_TUSB_MCU=OPT_MCU_STM32WB \
        -DFANTASI_ENABLE_MSC \
        -DFANTASI_ENABLE_BLE_CLI \
        -DFANTASI_ENABLE_APPS \
        -DFANTASI_ENABLE_BERRY \
        -DFANTASI_ENABLE_GUI \
        -DFANTASI_ENABLE_WEBUSB \
        -DFANTASI_ENABLE_PWR_BUTTON \
        -DFANTASI_ENABLE_RFID \
        -DFANTASI_FAT_STORAGE \
        -DLFS_NO_MALLOC \
        -DLFS_NO_DEBUG -DLFS_NO_WARN -DLFS_NO_ERROR \
        -DFANTASI_DEVICE_ID=\"$(DEVICE_ID)\" \
        -DFANTASI_GIT_HASH=\"$(GIT_HASH)\" \
        -DWEBUSB_PROTO_STACK=1024 \
        -DBLE_PROTO_STACK=1024 \
        -DUSB_TASK_STACK=2048 \
        $(EXTRA_DEFS)
# USB_TASK_STACK bumped 1024->2048 words (4KB->8KB): the MSC host-write path (tud_task -> fatrd_write ->
# build_model's littlefs traversal + write_dir + picolibc vsnprintf) overflows the 4KB "usb" task. Flipper has ample RAM.

INCLUDES := \
    -I$(ROOT)/core \
    -I$(ROOT)/apps \
    -I$(ROOT)/hal \
    -I$(ROOT)/hal/storage \
    -I$(ROOT)/platforms/flipper \
    -I$(ROOT)/hal/tinyusb \
    -I$(ROOT)/third_party/cmsis_core \
    -I$(ROOT)/third_party/stm32wb_cmsis \
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

LDSCRIPT := $(ROOT)/platforms/flipper/linker.ld
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
    $(ROOT)/core/ble_proto.c \
    $(ROOT)/core/usb_proto.c \
    $(ROOT)/core/ble_pipe.c \
    $(ROOT)/core/ramfs.c \
    $(ROOT)/core/vfs.c \
    $(ROOT)/core/elf_loader.c \
    $(ROOT)/core/app_run.c \
    $(ROOT)/core/log.c \
    $(ROOT)/core/libc_glue.c \
    $(ROOT)/core/berry_host.c \
    $(ROOT)/platforms/flipper/startup.c \
    $(ROOT)/platforms/flipper/system.c \
    $(ROOT)/platforms/flipper/hal.c \
    $(ROOT)/platforms/flipper/ble.c \
    $(ROOT)/platforms/flipper/ble_serial.c \
    $(ROOT)/platforms/flipper/display.c \
    $(ROOT)/platforms/flipper/gui.c \
    $(ROOT)/platforms/flipper/font_data.c \
    $(ROOT)/platforms/flipper/flash_storage.c \
    $(ROOT)/platforms/flipper/rfid.c \
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
    $(ROOT)/third_party/FreeRTOS-Kernel/portable/MemMang/heap_4.c

# TinyUSB sources (built only if user has cloned the library into
# third_party/tinyusb/). Missing files would error here; we guard by
# adding only what's present, so the user gets a clear "clone tinyusb"
# message from the top-level Makefile instead of a confusing GCC error.
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
    $(TUSB_DIR)/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
endif

# Berry scripting VM. Vendored in third_party/berry/.
BERRY_DIR := $(ROOT)/third_party/berry
ifneq (,$(wildcard $(BERRY_DIR)/src/be_vm.c))
SRCS += $(wildcard $(BERRY_DIR)/src/*.c)
endif

NANOPB_DIR := $(ROOT)/third_party/nanopb
ifneq (,$(wildcard $(NANOPB_DIR)/pb_encode.c))
SRCS += \
    $(NANOPB_DIR)/pb_encode.c \
    $(NANOPB_DIR)/pb_decode.c \
    $(NANOPB_DIR)/pb_common.c \
    $(ROOT)/proto/fantasi.pb.c
endif

LFS_DIR := $(ROOT)/third_party/littlefs
ifneq (,$(wildcard $(LFS_DIR)/lfs.c))
SRCS += $(LFS_DIR)/lfs.c $(LFS_DIR)/lfs_util.c
endif

OBJS := $(patsubst $(ROOT)/%.c,$(BUILD)/%.o,$(SRCS))

SPLASH_PNG ?= $(ROOT)/art/fantasi_splash.png

.PHONY: all clean size storage
all: $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).elf size

storage: $(BUILD)/splash.bin

$(BUILD)/splash.bin: $(SPLASH_PNG)
	@mkdir -p $(BUILD)
	python3 $(ROOT)/platforms/flipper/gen_splash.py $< $@

$(BUILD)/%.o: $(ROOT)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

# Header dependencies. Without these a header-only edit rebuilds nothing, so an object
# silently keeps a stale view of any struct/macro declared in that header - e.g. a change
# to app_image_t in core/elf_loader.h would leave core/app_run.o (which owns a static
# app_image_t) laid out to the old struct while the loader wrote the new fields. -MMD -MP
# emits a .d per object; -MP adds phony targets so a deleted header doesn't stall make.
-include $(OBJS:.o=.d)

$(BUILD)/$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	@mkdir -p $(BUILD)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

size: $(BUILD)/$(TARGET).elf
	$(SIZE) $<

clean:
	rm -rf $(BUILD)
