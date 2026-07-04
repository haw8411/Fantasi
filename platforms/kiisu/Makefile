# Fantasi / Kiisu platform build.
# Invoked from the top-level Makefile via `make PLATFORM=kiisu`.
#
# The Kiisu (kiisu.io v4) is a Flipper Zero-compatible board: same STM32WB55
# MCU and an identical peripheral pinout - display SPI (SCK PD1, MOSI PB15,
# CS PC11, D/I PB1, RST PB0), LP5562 LED + BQ27220 gauge on I2C1 (PA9/PA10),
# PA3 peripheral-power rail, and the same six button pins (UP PB10, DOWN PC6,
# LEFT PB11, RIGHT PB12, OK PH3, BACK PC13). See
# original_fw/kiisu-firmware/targets/f7 - every resource matches the Flipper.
#
# So there is nothing platform-specific to write: this build is the Flipper
# build with a different output name and device id. It sets those three
# variables and includes the Flipper Makefile, which does the rest (and
# compiles the shared platforms/flipper/*.c sources verbatim).

TARGET    := fantasi-kiisu
BUILD     := $(abspath ../..)/build/kiisu
DEVICE_ID := KIISU

# The Kiisu LCD is an SH1106 behind a companion MCU that emulates the Flipper's
# ST7565; it needs the u8g2-style single-CS-per-page flush (see display.c).
# EXTRA_DEFS is appended to DEFS inside the Flipper Makefile (which reassigns
# DEFS with :=, so it must be threaded through this hook, not DEFS directly).
EXTRA_DEFS := -DFANTASI_DISPLAY_COMPANION

include ../flipper/Makefile
