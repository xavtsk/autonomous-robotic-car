CC = cc
ROOT = ../..
APP = $(ROOT)/app_program
BUILD = $(ROOT)/build/controller
CPPFLAGS = -DAPP_H -DAPP_TEST=0 -include controller_test_kernel.h -I.
CFLAGS = -std=c11 -Wall -Wextra -Werror -O2 -g -pthread
SOURCES = team_adapter_test.c $(APP)/controller/vehicle_controller.c \
          $(APP)/subsystem3/navigation_codec.c $(APP)/subsystem2/motion_codec.c

.PHONY: test sanitize
test:
	mkdir -p $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SOURCES) -o $(BUILD)/controller_test
	$(BUILD)/controller_test
	$(CC) $(CFLAGS) codec_test.c $(APP)/controller/integration_codec.c $(APP)/subsystem3/navigation_codec.c -o $(BUILD)/codec_test
	$(BUILD)/codec_test
	$(CC) $(CFLAGS) timing_test.c -o $(BUILD)/timing_test
	$(BUILD)/timing_test
	$(CC) $(CFLAGS) ir_debug_test.c $(APP)/controller/ir_debug.c -o $(BUILD)/ir_debug_test
	$(BUILD)/ir_debug_test
	$(CC) $(CFLAGS) control_debug_test.c $(APP)/controller/control_debug.c -o $(BUILD)/control_debug_test
	$(BUILD)/control_debug_test
	$(CC) -Ifakes $(CFLAGS) runtime_test.c $(APP)/controller/barcode_capture.c $(APP)/controller/integration_codec.c $(APP)/controller/s3_drive.c $(APP)/subsystem3/navigation_codec.c $(addprefix $(APP)/subsystem3/,ir_acquisition.c ir_calibration.c ir_reflectance.c line_detection.c line_controller.c barcode_decoder.c line_navigation.c) -o $(BUILD)/runtime_test
	$(BUILD)/runtime_test
	$(CC) -Ifakes -DTM_MQTT=1 -I$(ROOT)/lib/liblwip/sysdepend/pico_rp2040 $(CFLAGS) runtime_test.c $(APP)/controller/barcode_capture.c $(APP)/controller/integration_codec.c $(APP)/controller/s3_drive.c $(APP)/subsystem3/navigation_codec.c $(addprefix $(APP)/subsystem3/,ir_acquisition.c ir_calibration.c ir_reflectance.c line_detection.c line_controller.c barcode_decoder.c line_navigation.c) -o $(BUILD)/runtime_mqtt_test
	$(BUILD)/runtime_mqtt_test
	$(CC) -Ifakes -DAPP_IR_DEBUG=1 -DTM_MQTT=1 -I$(ROOT)/lib/liblwip/sysdepend/pico_rp2040 $(CFLAGS) runtime_test.c $(APP)/controller/ir_debug.c $(APP)/controller/control_debug.c $(APP)/controller/barcode_capture.c $(APP)/controller/integration_codec.c $(APP)/controller/s3_drive.c $(APP)/subsystem3/navigation_codec.c $(addprefix $(APP)/subsystem3/,ir_acquisition.c ir_calibration.c ir_reflectance.c line_detection.c line_controller.c barcode_decoder.c line_navigation.c) -o $(BUILD)/runtime_ir_debug_test
	$(BUILD)/runtime_ir_debug_test
	$(CC) -Ifakes $(CFLAGS) operator_test.c $(APP)/controller/barcode_capture.c $(APP)/controller/operator_logic.c $(APP)/controller/integration_codec.c $(APP)/controller/s3_drive.c $(APP)/subsystem3/navigation_codec.c $(addprefix $(APP)/subsystem3/,ir_acquisition.c ir_calibration.c ir_reflectance.c line_detection.c line_controller.c barcode_decoder.c line_navigation.c) -o $(BUILD)/operator_test
	$(BUILD)/operator_test
	$(CC) -Ifakes $(CFLAGS) map_freeze_test.c $(APP)/controller/barcode_capture.c $(APP)/controller/integration_codec.c $(APP)/controller/s3_drive.c $(APP)/subsystem3/navigation_codec.c $(APP)/subsystem1/telemetry_codec.c $(APP)/subsystem2/motion_codec.c $(addprefix $(APP)/subsystem3/,ir_acquisition.c ir_calibration.c ir_reflectance.c line_detection.c line_controller.c barcode_decoder.c line_navigation.c) -o $(BUILD)/map_freeze_test
	$(BUILD)/map_freeze_test
	$(CC) -Ifakes $(CFLAGS) builtin_test.c $(APP)/controller/barcode_capture.c $(APP)/controller/operator_logic.c $(APP)/controller/integration_codec.c $(APP)/controller/s3_drive.c $(APP)/subsystem3/navigation_codec.c $(addprefix $(APP)/subsystem3/,ir_acquisition.c ir_calibration.c ir_reflectance.c line_detection.c line_controller.c barcode_decoder.c line_navigation.c) -o $(BUILD)/builtin_test
	$(BUILD)/builtin_test
	$(CC) -I$(APP)/subsystem2 -I$(APP)/subsystem3 -I$(APP)/controller $(CFLAGS) course_test.c $(APP)/controller/s3_drive.c $(addprefix $(APP)/subsystem3/,ir_acquisition.c ir_calibration.c ir_reflectance.c line_detection.c line_controller.c barcode_decoder.c line_navigation.c) -o $(BUILD)/course_test
	$(BUILD)/course_test
	$(CC) -I$(APP)/controller $(CFLAGS) bcap_test.c $(APP)/controller/barcode_capture.c -o $(BUILD)/bcap_test
	$(BUILD)/bcap_test
	$(CC) -D_DEFAULT_SOURCE -D_USE_MATH_DEFINES -I$(APP)/subsystem2 -I$(APP)/subsystem3 -I$(APP)/controller $(CFLAGS) course_sim.c $(APP)/controller/s3_drive.c $(addprefix $(APP)/subsystem3/,ir_acquisition.c ir_calibration.c ir_reflectance.c line_detection.c line_controller.c barcode_decoder.c line_navigation.c) $(addprefix $(APP)/subsystem2/,motion_controller.c motion_pack.c motion_estimation.c speed_pid.c) -lm -o $(BUILD)/course_sim
	$(BUILD)/course_sim

sanitize:
	# Apple Clang 21 crashes on this codec with -O1 + sanitizers; -O2 passes.
	$(MAKE) -f controller_test.mk test CFLAGS='$(CFLAGS) -O2 -fsanitize=address,undefined -fno-omit-frame-pointer'
