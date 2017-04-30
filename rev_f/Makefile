ARDUINO_DIR = ../../Arduino
ARDMK_DIR = ../../Arduino-Makefile
ALTERNATE_CORE = ubiquity/avr
ALTERNATE_CORE_PATH = $(ARDUINO_DIR)/hardware/$(ALTERNATE_CORE)
ARCHITECTURE = avr
BOARD_TAG = mega2560
ARDUINO_PORT = /dev/ttyUSB0
ARDUINO_LIBS := . ./bus_slave ./bus_common ./bus_server
USER_LIB_PATH := $(realpath .)
MCU=atmega2560
F_CPU = 16000000L
VARIANT = mega

include $(ARDMK_DIR)/Arduino.mk
