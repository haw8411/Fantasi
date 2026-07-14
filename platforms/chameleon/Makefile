# Fantasi / Chameleon Ultra platform build.

TARGET    := fantasi-chameleon
ROOT      := $(abspath ../..)
BUILD     := $(ROOT)/build/chameleon

CROSS     ?= arm-none-eabi-
CC        := $(CROSS)gcc
OBJCOPY   := $(CROSS)objcopy
SIZE      := $(CROSS)size

MCU_FLAGS := -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16

# NRF52840_XXAA is the standard preprocessor switch Nordic headers
# use to select the nRF52840 variant. CFG_TUSB_MCU picks TinyUSB's
# nRF5x driver path.
GIT_HASH := $(shell git rev-parse --short=7 HEAD 2>/dev/null || echo unknown)$(shell git diff --quiet 2>/dev/null || echo -dirty)

DEFS := -DNRF52840_XXAA \
        -DCFG_TUSB_MCU=OPT_MCU_NRF5X \
        -DFANTASI_ENABLE_MSC \
        -DFANTASI_ENABLE_BLE_CLI \
        -DFANTASI_ENABLE_APPS \
        -DFANTASI_ENABLE_BERRY \
        -DFANTASI_ENABLE_WEBUSB \
        -DFANTASI_ENABLE_PWR_BUTTON \
        -DFANTASI_FAT_STORAGE \
        -DLFS_NO_MALLOC \
        -DFANTASI_GIT_HASH=\"$(GIT_HASH)\"

INCLUDES := \
    -I$(ROOT)/core \
    -I$(ROOT)/apps \
    -I$(ROOT)/hal \
    -I$(ROOT)/hal/storage \
    -I$(ROOT)/platforms/chameleon \
    -I$(ROOT)/hal/tinyusb \
    -I$(ROOT)/third_party/cmsis_core \
    -I$(ROOT)/third_party/nrf52_mdk \
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
          -std=gnu11 -g3

LDSCRIPT := $(ROOT)/platforms/chameleon/linker.ld
LDFLAGS := $(MCU_FLAGS) -T$(LDSCRIPT) \
           -Wl,--gc-sections \
           -Wl,-Map=$(BUILD)/$(TARGET).map \
           -Wl,--wrap=_malloc_r,--wrap=_free_r,--wrap=_calloc_r,--wrap=_realloc_r \
           -nostartfiles --specs=nano.specs --specs=nosys.specs \
           -u _printf_float -lm

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
    $(ROOT)/core/newlib_malloc.c \
    $(ROOT)/core/berry_host.c \
    $(ROOT)/platforms/chameleon/startup.c \
    $(ROOT)/platforms/chameleon/system.c \
    $(ROOT)/platforms/chameleon/ble.c \
    $(ROOT)/platforms/chameleon/ble_serial.c \
    $(ROOT)/platforms/chameleon/hal.c \
    $(ROOT)/platforms/chameleon/flash_storage.c \
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
    $(TUSB_DIR)/portable/nordic/nrf5x/dcd_nrf5x.c
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
all: $(BUILD)/$(TARGET).hex $(BUILD)/$(TARGET).bin size

$(BUILD)/%.o: $(ROOT)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

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
