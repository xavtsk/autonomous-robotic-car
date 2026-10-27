CC = cc
ROOT = ../..
BUILD = $(ROOT)/build/diagnostics
CPPFLAGS = -DAPP_H -DAPP_DIAGNOSTICS=1 -include firmware_test_kernel.h -I. -Ifakes
CFLAGS = -std=c11 -Wall -Wextra -Werror -pedantic -O2 -g
SOURCES = firmware_test.c $(ROOT)/app_program/subsystem2/motion_codec.c

.PHONY: test sanitize disabled smoke
smoke:
	mkdir -p $(BUILD)
	$(CC) -Ifakes $(CFLAGS) motion_smoke_test.c $(ROOT)/app_program/subsystem2/motion_codec.c -o $(BUILD)/motion_smoke_test
	$(BUILD)/motion_smoke_test

test:
	mkdir -p $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SOURCES) -o $(BUILD)/firmware_test
	$(BUILD)/firmware_test
	$(CC) $(CPPFLAGS) $(CFLAGS) startup_test.c -o $(BUILD)/startup_test
	$(BUILD)/startup_test

sanitize:
	$(MAKE) -f firmware_test.mk test CFLAGS='$(CFLAGS) -O1 -fsanitize=address,undefined -fno-omit-frame-pointer'

disabled:
	mkdir -p $(BUILD)
	$(CC) -DAPP_H -DAPP_DIAGNOSTICS=0 -include firmware_test_kernel.h -I. $(CFLAGS) -Wno-empty-translation-unit -c $(ROOT)/app_program/diagnostics/robot_diagnostics.c -o $(BUILD)/disabled.o
	test -z "`nm $(BUILD)/disabled.o | sed '/ ltmp[0-9][0-9]*$$/d; /^[[:space:]]*$$/d'`"
