CC = cc
APP = ../../app_program/subsystem2
SOURCE_DIR ?= $(APP)
BUILD = ../../build/buddy2
CPPFLAGS = -DAPP_H -DAPP_TEST=1 -include host_kernel.h -I. -Ifakes -I$(SOURCE_DIR)
# Both IRQ pins must be acknowledged: the original intentional boolean |
# must not be changed to short-circuit || merely to silence Clang.
CFLAGS = -std=c11 -Wall -Wextra -Werror -Wno-bitwise-instead-of-logical -O2 -g
MODULES = motion motion_controller motion_estimation motor_control speed_pid wheel_encoder subsystem2_test
SOURCES = host_regression.c $(addprefix $(SOURCE_DIR)/,$(addsuffix .c,$(MODULES)))
ifeq ($(SOURCE_DIR),$(APP))
CPPFLAGS += -DTEST_PACKED
SOURCES += $(APP)/motion_codec.c $(APP)/../subsystem1/telemetry_codec.c
endif
OUTPUT ?= $(BUILD)/host_regression

.PHONY: test sanitize adapter regression sizes legacy-test pack
# The old IRQ/field-layout harness below is retained as historical evidence.
# PIO, report queues and calibration cancellation supersede that implementation.
test: pack
	$(MAKE) -C ../subsystem3 feedback drive

pack:
	mkdir -p $(BUILD)
	$(CC) -I$(APP) $(CFLAGS) pack_test.c $(APP)/motion_pack.c $(APP)/motion_codec.c $(APP)/motion_snapshot_adapter.c -o $(BUILD)/pack_test
	$(BUILD)/pack_test

legacy-test:
	mkdir -p $(dir $(OUTPUT))
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SOURCES) -o $(OUTPUT)
	$(OUTPUT)

sanitize:
	$(MAKE) test CFLAGS='$(CFLAGS) -O1 -fsanitize=address,undefined -fno-omit-frame-pointer' OUTPUT=$(BUILD)/host_sanitize

adapter:
	$(MAKE) -C ../integration -f controller_test.mk test

regression:
	bash run_regression.sh

sizes:
	mkdir -p $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -fshort-enums size_report.c -o $(BUILD)/size_report
	$(BUILD)/size_report
