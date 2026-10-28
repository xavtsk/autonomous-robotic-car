CC = cc
APP = ../../app_program
BUILD = ../../build/buddy5
CFLAGS = -std=c11 -Wall -Wextra -Werror -pedantic -O2 -g
S5 = $(addprefix $(APP)/subsystem5/,obstacle_profile.c obstacle_scan.c avoidance_planner.c line_recovery.c obstacle_avoidance.c s5_codec.c)
S3 = $(addprefix $(APP)/subsystem3/,ir_acquisition.c ir_calibration.c ir_reflectance.c line_detection.c line_controller.c barcode_decoder.c line_navigation.c)
S2 = $(addprefix $(APP)/subsystem2/,motion_controller.c motion_pack.c motion_estimation.c speed_pid.c)
INC = -I$(APP)/subsystem5 -I$(APP)/subsystem3 -I$(APP)/subsystem2 -I$(APP)/controller
.PHONY: test sanitize
test:
	mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(INC) host_test.c $(S5) $(APP)/subsystem5/ultrasonic_sensor.c -o $(BUILD)/host_test
	$(BUILD)/host_test
	$(CC) $(CFLAGS) $(INC) -DAPP_S5=1 controller_test.c $(S5) $(S3) $(S2) $(APP)/controller/s3_drive.c -o $(BUILD)/controller_test
	$(BUILD)/controller_test
	$(CC) $(CFLAGS) $(INC) -DAPP_S5=1 -DAPP_S4=1 -DAPP_H -DAPP_TEST=0 -include ../integration/controller_test_kernel.h controller_test.c $(S5) $(S3) $(S2) $(APP)/controller/s3_drive.c $(APP)/controller/terrain_integration.c -o $(BUILD)/terrain_controller_test
	$(BUILD)/terrain_controller_test
	$(CC) $(CFLAGS) $(INC) -DAPP_S5=1 -D_DEFAULT_SOURCE -D_USE_MATH_DEFINES course_clear_test.c $(S5) $(S3) $(S2) $(APP)/controller/s3_drive.c -lm -o $(BUILD)/course_clear_test
	$(BUILD)/course_clear_test
	$(CC) $(CFLAGS) $(INC) -Ifakes hardware_test.c -o $(BUILD)/hardware_test
	$(BUILD)/hardware_test
	$(CC) $(CFLAGS) $(INC) -Ifakes runtime_test.c $(S5) -o $(BUILD)/runtime_test
	$(BUILD)/runtime_test
	$(CC) $(CFLAGS) $(INC) -Ifakes -DS5_HARDWARE_CONFIRMED=0 runtime_test.c $(S5) -o $(BUILD)/activation_gate_test
	$(BUILD)/activation_gate_test
	$(CC) $(CFLAGS) $(INC) -DAPP_S5=1 -DAPP_H -DAPP_TEST=0 -include ../integration/controller_test_kernel.h telemetry_test.c $(APP)/controller/vehicle_controller.c -o $(BUILD)/telemetry_test
	$(BUILD)/telemetry_test
sanitize:
	$(MAKE) test CFLAGS='$(CFLAGS) -O1 -fsanitize=address,undefined -fno-omit-frame-pointer'
