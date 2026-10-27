CC = cc
PYTHON ?= python3
ROOT = ../..
APP = $(ROOT)/app_program/subsystem1
MQTT = $(ROOT)/lib/liblwip/sysdepend/pico_rp2040
BUILD = $(ROOT)/build/buddy1
CPPFLAGS = -DAPP_H -DAPP_TEST=1 -DTM_MQTT=1 -include host_kernel.h -I. -Ifakes -I$(APP) -I$(MQTT)
CFLAGS = -std=c11 -Wall -Wextra -Werror -O2 -g -pthread
SOURCES = host_test.c $(APP)/telemetry_codec.c $(APP)/subsystem1.c \
          $(APP)/command.c $(APP)/mqtt_bridge.c $(APP)/subsystem1_test.c $(MQTT)/mqtt_utk.c

.PHONY: test sanitize sizes
test: $(BUILD)/host_test
	$(BUILD)/host_test
	$(PYTHON) test_laptop_codec.py

$(BUILD)/host_test: $(SOURCES) $(APP)/telemetry_codec.h $(APP)/subsystem1.h $(MQTT)/mqtt_utk.h host_kernel.h
	mkdir -p $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SOURCES) -o $@

sanitize:
	mkdir -p $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -O1 -fsanitize=address,undefined -fno-omit-frame-pointer $(SOURCES) -o $(BUILD)/host_test_sanitize
	$(BUILD)/host_test_sanitize

sizes:
	mkdir -p $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -fshort-enums size_report.c $(APP)/telemetry_codec.c -o $(BUILD)/size_report
	$(BUILD)/size_report
