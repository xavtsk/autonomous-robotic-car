CC = cc
ROOT = ../..
APP = $(ROOT)/app_program
BUILD = $(ROOT)/build/controller
CPPFLAGS = -DAPP_H -DAPP_TEST=0 -include ctrl_test_kernel.h -I.
CFLAGS = -std=c11 -Wall -Wextra -Werror -O2 -g -pthread
# Python 3 selects the reviewed private layout; override with PYTHON=/path/to/python.
PYTHON ?= python3
PRIVATE_STORAGE_FLAGS := $(shell $(PYTHON) ../../tools/compliance/instrument_harnesses.py --private-storage-flag ../..)
ifneq ($(.SHELLSTATUS),0)
$(error Private storage detection failed; verify PYTHON names a Python 3 interpreter)
endif
override CFLAGS += $(PRIVATE_STORAGE_FLAGS)
SOURCES = team_adapter_main.c $(APP)/controller/vehicle_controller.c \
          $(APP)/subsystem3/navigation_codec.c $(APP)/subsystem2/codec_motion.c
RUNTIME_FIXTURE = -I. -Ifakes -I$(APP)/controller -DS3_RUNTIME_TEST -include runtime_main.h
RUNTIME_SOURCES = runtime_main.c $(APP)/controller/s3_integration.c \
          $(APP)/controller/bc_capture.c $(APP)/controller/integration_codec.c \
          $(APP)/controller/s3_drive.c $(APP)/subsystem3/navigation_codec.c \
          $(addprefix $(APP)/subsystem3/,ir_acquisition.c ir_calibration.c ir_reflectance.c line_detection.c line_controller.c bc_decoder.c line_navigation.c)

.PHONY: test sanitize
test:
	mkdir -p $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SOURCES) -o $(BUILD)/controller_test
	$(BUILD)/controller_test
	$(CC) $(CFLAGS) codec_main.c $(APP)/controller/integration_codec.c $(APP)/subsystem3/navigation_codec.c -o $(BUILD)/codec_test
	$(BUILD)/codec_test
	$(CC) $(CFLAGS) timing_main.c -o $(BUILD)/timing_test
	$(BUILD)/timing_test
	$(CC) $(CFLAGS) ir_wire_main.c $(APP)/controller/ir_debug.c -o $(BUILD)/ir_debug_test
	$(BUILD)/ir_debug_test
	$(CC) $(CFLAGS) cmd_wire_main.c $(APP)/controller/control_debug.c -o $(BUILD)/control_debug_test
	$(BUILD)/control_debug_test
	$(CC) -DAPP_H -DAPP_TEST=0 -DAPP_S3_MOTION=1 -DTM_MQTT=1 -include ../subsystem1/host_kernel.h -I$(ROOT)/lib/liblwip/sysdepend/pico_rp2040 $(CFLAGS) remote_bridge_main.c $(APP)/subsystem1/mqtt_bridge.c $(APP)/subsystem1/telemetry_codec.c -o $(BUILD)/remote_bridge_test
	$(BUILD)/remote_bridge_test
	$(CC) $(RUNTIME_FIXTURE) $(CFLAGS) $(RUNTIME_SOURCES) -o $(BUILD)/runtime_test
	$(BUILD)/runtime_test
	$(CC) $(RUNTIME_FIXTURE) -DTM_MQTT=1 -I$(ROOT)/lib/liblwip/sysdepend/pico_rp2040 $(CFLAGS) $(RUNTIME_SOURCES) -o $(BUILD)/runtime_mqtt_test
	$(BUILD)/runtime_mqtt_test
	$(CC) $(RUNTIME_FIXTURE) -DAPP_IR_DEBUG=1 -DTM_MQTT=1 -I$(ROOT)/lib/liblwip/sysdepend/pico_rp2040 $(CFLAGS) $(RUNTIME_SOURCES) $(APP)/controller/ir_debug.c $(APP)/controller/control_debug.c -o $(BUILD)/runtime_ir_debug_test
	$(BUILD)/runtime_ir_debug_test
	$(CC) -I. -Ifakes -I$(APP)/controller $(CFLAGS) -DS3_OPERATOR_TEST -include operator_main.h \
	    operator_main.c $(APP)/controller/s3_integration.c $(APP)/controller/bc_capture.c $(APP)/controller/op_logic.c $(APP)/controller/integration_codec.c $(APP)/controller/s3_drive.c $(APP)/subsystem3/navigation_codec.c $(addprefix $(APP)/subsystem3/,ir_acquisition.c ir_calibration.c ir_reflectance.c line_detection.c line_controller.c bc_decoder.c line_navigation.c) -o $(BUILD)/operator_test
	$(BUILD)/operator_test
	$(CC) $(CFLAGS) -DAPP_H -DAPP_S3_MOTION=1 -DAPP_TEST=0 -include map_freeze_main.h -Ifakes \
	    -c $(APP)/subsystem1/command.c -o $(BUILD)/map_command.o
	$(CC) $(CFLAGS) -DAPP_H -DAPP_S3_MOTION=1 -DAPP_TEST=0 -include map_freeze_main.h -Ifakes \
	    -c $(APP)/controller/vehicle_controller.c -o $(BUILD)/map_controller.o
	$(CC) $(CFLAGS) -I. -Ifakes -DAPP_H -DAPP_S3_MOTION=1 -DAPP_DIAGNOSTICS=0 -DAPP_TEST=0 \
	    -DS3_BUILTIN_CALIBRATION=0 -DS3_LINE_BETWEEN=0 -DS3_CROSS_STRAIGHT=0 \
	    -DS3_COURSE_PENDING_WAIT_US=5000000U -DS3_MAP_FREEZE_TEST -include map_freeze_main.h \
	    -c $(APP)/controller/s3_integration.c -o $(BUILD)/map_runtime.o
	$(CC) -Ifakes $(CFLAGS) map_freeze_main.c $(BUILD)/map_runtime.o $(BUILD)/map_command.o $(BUILD)/map_controller.o $(APP)/controller/bc_capture.c $(APP)/controller/integration_codec.c $(APP)/controller/s3_drive.c $(APP)/subsystem3/navigation_codec.c $(APP)/subsystem1/telemetry_codec.c $(APP)/subsystem2/codec_motion.c $(addprefix $(APP)/subsystem3/,ir_acquisition.c ir_calibration.c ir_reflectance.c line_detection.c line_controller.c bc_decoder.c line_navigation.c) -o $(BUILD)/map_freeze_test
	$(BUILD)/map_freeze_test
	$(CC) $(CFLAGS) -I. -Ifakes -DAPP_H -DAPP_S3_MOTION=1 -DAPP_DIAGNOSTICS=0 -DAPP_TEST=0 \
	    -DS3_BUILTIN_CALIBRATION=1 -DS3_CAL_DEFAULTS_TEST -include cal_defaults_main.h \
	    cal_defaults_main.c $(APP)/controller/s3_integration.c $(APP)/controller/bc_capture.c $(APP)/controller/op_logic.c $(APP)/controller/integration_codec.c $(APP)/controller/s3_drive.c $(APP)/subsystem3/navigation_codec.c $(addprefix $(APP)/subsystem3/,ir_acquisition.c ir_calibration.c ir_reflectance.c line_detection.c line_controller.c bc_decoder.c line_navigation.c) -o $(BUILD)/builtin_test
	$(BUILD)/builtin_test
	$(CC) -I$(APP)/subsystem2 -I$(APP)/subsystem3 -I$(APP)/controller $(CFLAGS) course_main.c $(APP)/controller/s3_drive.c $(addprefix $(APP)/subsystem3/,ir_acquisition.c ir_calibration.c ir_reflectance.c line_detection.c line_controller.c bc_decoder.c line_navigation.c) -o $(BUILD)/course_test
	$(BUILD)/course_test
	$(CC) -DS3_STRAIGHT_TEST=1 -I$(APP)/subsystem2 -I$(APP)/subsystem3 -I$(APP)/controller $(CFLAGS) course_main.c $(APP)/controller/s3_drive.c $(addprefix $(APP)/subsystem3/,ir_acquisition.c ir_calibration.c ir_reflectance.c line_detection.c line_controller.c bc_decoder.c line_navigation.c) -o $(BUILD)/straight_test
	$(BUILD)/straight_test
	$(CC) -I$(APP)/controller $(CFLAGS) bcap_main.c $(APP)/controller/bc_capture.c -o $(BUILD)/bcap_test
	$(BUILD)/bcap_test
	$(CC) -I$(APP)/controller $(CFLAGS) bcap_wire_main.c $(APP)/controller/bc_capture.c -o $(BUILD)/bcap_wire_test
	$(BUILD)/bcap_wire_test
	$(CC) -D_DEFAULT_SOURCE -D_USE_MATH_DEFINES -I$(APP)/subsystem2 -I$(APP)/subsystem3 -I$(APP)/controller $(CFLAGS) course_sim_main.c $(APP)/controller/s3_drive.c $(addprefix $(APP)/subsystem3/,ir_acquisition.c ir_calibration.c ir_reflectance.c line_detection.c line_controller.c bc_decoder.c line_navigation.c) $(addprefix $(APP)/subsystem2/,ctrl_motion.c motion_pack.c motion_estimation.c speed_pid.c) -lm -o $(BUILD)/course_sim
	$(BUILD)/course_sim

sanitize:
	# Apple Clang 21 crashes on this codec with -O1 + sanitizers; -O2 passes.
	$(MAKE) -f controller_test.mk test CFLAGS='$(CFLAGS) -O2 -fsanitize=address,undefined -fno-omit-frame-pointer'
