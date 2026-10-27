CC = cc
APP = ../../app_program/subsystem3
SOURCE_DIR ?= $(APP)
BASELINE_DIR ?=
BUILD = ../../build/buddy3
CPPFLAGS = -I$(SOURCE_DIR) -I.
CFLAGS = -std=c11 -Wall -Wextra -Werror -pedantic -O2 -g
MODULES = ir_acquisition ir_calibration ir_reflectance line_detection line_controller barcode_decoder line_navigation
SOURCES = $(addprefix $(SOURCE_DIR)/,$(addsuffix .c,$(MODULES)))

.PHONY: test regression compare bench bench-compare codec sanitize sizes drive feedback
test:
	mkdir -p $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) test_core.c test_navigation.c $(SOURCES) -o $(BUILD)/test_core
	$(BUILD)/test_core
	$(CC) $(CPPFLAGS) $(CFLAGS) test_mapping.c $(SOURCES) -o $(BUILD)/test_mapping
	$(BUILD)/test_mapping
	$(CC) $(CPPFLAGS) $(CFLAGS) test_ir_reflectance.c $(SOURCE_DIR)/ir_reflectance.c -o $(BUILD)/test_ir_reflectance
	$(BUILD)/test_ir_reflectance

regression:
	bash run_regression.sh

compare:
	mkdir -p $(BUILD)
	$(CC) -I$(BASELINE_DIR) -I. $(CFLAGS) host_trace.c $(addprefix $(BASELINE_DIR)/,$(addsuffix .c,$(MODULES))) -o $(BUILD)/before_trace
	$(BUILD)/before_trace > $(BUILD)/before.trace
	$(CC) $(CPPFLAGS) $(CFLAGS) host_trace.c $(SOURCES) -o $(BUILD)/after_trace
	$(BUILD)/after_trace > $(BUILD)/after.trace
	cmp $(BUILD)/before.trace $(BUILD)/after.trace
	shasum -a 256 $(BUILD)/before.trace $(BUILD)/after.trace

bench:
	mkdir -p $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) bench_trace.c $(SOURCES) $(APP)/navigation_codec.c -o $(BUILD)/after_bench
	$(BUILD)/after_bench > $(BUILD)/after-bench.trace
	tail -n 1 $(BUILD)/after-bench.trace

bench-compare: bench
	mkdir -p $(BUILD)
	$(CC) -I$(BASELINE_DIR) -I. $(CFLAGS) bench_trace.c $(addprefix $(BASELINE_DIR)/,$(addsuffix .c,$(MODULES))) -o $(BUILD)/before_bench
	$(BUILD)/before_bench > $(BUILD)/before-bench.trace
	cmp $(BUILD)/before-bench.trace $(BUILD)/after-bench.trace
	shasum -a 256 $(BUILD)/before-bench.trace $(BUILD)/after-bench.trace

codec:
	mkdir -p $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) test_codec.c $(APP)/navigation_codec.c -o $(BUILD)/test_codec
	$(BUILD)/test_codec

sanitize:
	$(MAKE) test codec drive feedback CFLAGS='$(CFLAGS) -O1 -fsanitize=address,undefined -fno-omit-frame-pointer'

MOTION = $(APP)/../subsystem2
MOTION_CORE = $(addprefix $(MOTION)/,motion_controller.c motion_pack.c motion_estimation.c speed_pid.c)
drive:
	mkdir -p $(BUILD)
	$(CC) $(CPPFLAGS) -I$(MOTION) -I$(APP)/../controller $(CFLAGS) test_drive.c $(SOURCES) $(APP)/../controller/s3_drive.c $(MOTION_CORE) -o $(BUILD)/test_drive
	$(BUILD)/test_drive

feedback:
	mkdir -p $(BUILD)
	$(CC) $(CPPFLAGS) -I$(MOTION) -include motion_kernel_stub.h $(CFLAGS) test_motion_feedback.c $(MOTION)/motion.c $(MOTION_CORE) -o $(BUILD)/test_motion_feedback
	$(BUILD)/test_motion_feedback

sizes:
	mkdir -p $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -fshort-enums size_report.c -o $(BUILD)/size_report
	$(BUILD)/size_report
