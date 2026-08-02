TARGET  := fantasi
ROOT    := $(abspath ..)
BUILD   := $(ROOT)/build/cli

CC      := cc
CFLAGS  := -Wall -Wextra -O2 -std=gnu11 -I.

LDLIBS  := -lreadline

# main.c owns the transport/mount/BLE plumbing; each command is its own file
# under commands/ and self-registers (see cli/commands/README.md). The wildcard
# picks up new command files automatically.
SRCS    := main.c theme.c $(wildcard commands/*.c)

# BLE transport (requires libsystemd)
HAS_SYSTEMD := $(shell pkg-config --exists libsystemd 2>/dev/null && echo 1)
ifeq ($(HAS_SYSTEMD),1)
CFLAGS += -DHAS_BLE \
          -I$(ROOT)/third_party/nanopb \
          -I$(ROOT)/proto
SRCS   += ble_transport.c \
          $(ROOT)/third_party/nanopb/pb_encode.c \
          $(ROOT)/third_party/nanopb/pb_decode.c \
          $(ROOT)/third_party/nanopb/pb_common.c \
          $(ROOT)/proto/fantasi.pb.c
LDLIBS += -lsystemd
endif

# USB vendor (WebUSB) transport via libusb. The protobuf machinery it shares is
# compiled under HAS_BLE, so the USB path is wired only when both are present
# (this host has both).
HAS_USB := $(shell pkg-config --exists libusb-1.0 2>/dev/null && echo 1)
ifeq ($(HAS_USB),1)
CFLAGS += -DHAS_USB_VENDOR $(shell pkg-config --cflags libusb-1.0)
SRCS   += usb_transport.c
LDLIBS += $(shell pkg-config --libs libusb-1.0)
endif

OBJS    := $(patsubst %.c,$(BUILD)/%.o,$(notdir $(SRCS)))

vpath %.c . commands $(ROOT)/third_party/nanopb $(ROOT)/proto

.PHONY: all clean

all: $(BUILD)/$(TARGET)

$(BUILD)/$(TARGET): $(OBJS)
	$(CC) $(OBJS) $(LDLIBS) -o $@

$(BUILD)/%.o: %.c
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD)
