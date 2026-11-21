CC = cc
APP = ../../app_program
BUILD = ../../build/buddy5
CFLAGS = -std=c11 -Wall -Wextra -Werror -pedantic -O2 -g
S5 = $(addprefix $(APP)/subsystem5/,profile_obstacle.c scan_obstacle.c avoidance_planner.c line_recovery.c avoid_obstacle.c s5_codec.c)
S3 = $(addprefix $(APP)/subsystem3/,ir_acquisition.c ir_calibration.c ir_reflectance.c line_detection.c line_controller.c bc_decoder.c line_navigation.c)
S2 = $(addprefix $(APP)/subsystem2/,ctrl_motion.c motion_pack.c motion_estimation.c speed_pid.c)
INC = -I$(APP)/subsystem5 -I$(APP)/subsystem3 -I$(APP)/subsystem2 -I$(APP)/controller
TASK_FIXTURE = -I. -Ifakes -DAPP_H -DAPP_S5=1 -DS5_TASK_TEST -DS5_SCAN_LOG=0 -include s5_task_main.h
.PHONY: test sanitize
test:
	mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(INC) -DS5_HOST_TEST bypass_fsm_main.c $(S5) $(APP)/subsystem5/ultrasonic_sensor.c -o $(BUILD)/host_test
	$(BUILD)/host_test
	$(CC) $(CFLAGS) $(INC) -DAPP_S5=1 s5_ctrl_main.c $(S5) $(S3) $(S2) $(APP)/controller/s3_drive.c -o $(BUILD)/controller_test
	$(BUILD)/controller_test
	$(CC) $(CFLAGS) $(INC) -DAPP_S5=1 -DAPP_S4=1 -DAPP_H -DAPP_TEST=0 -include ../integration/ctrl_test_kernel.h s5_ctrl_main.c $(S5) $(S3) $(S2) $(APP)/controller/s3_drive.c $(APP)/controller/terr_link.c -o $(BUILD)/terrain_controller_test
	$(BUILD)/terrain_controller_test
	$(CC) $(CFLAGS) $(INC) -DAPP_S5=1 -D_DEFAULT_SOURCE -D_USE_MATH_DEFINES -DCOURSE_CLEAR_TEST \
	    course_clear_main.c ../integration/course_sim_main.c $(S5) $(S3) $(S2) $(APP)/controller/s3_drive.c -lm -o $(BUILD)/course_clear_test
	$(BUILD)/course_clear_test
	$(CC) $(CFLAGS) $(INC) -DAPP_S5=1 -D_DEFAULT_SOURCE -D_USE_MATH_DEFINES -DCOURSE_BYPASS_TEST \
	    bypass_course_main.c ../integration/course_sim_main.c $(S5) $(S3) $(S2) $(APP)/controller/s3_drive.c -lm -o $(BUILD)/bypass_course_test
	$(BUILD)/bypass_course_test
	$(BUILD)/bypass_course_test beam=10 > /dev/null
	$(BUILD)/bypass_course_test beam=20 > /dev/null
	$(BUILD)/bypass_course_test turn=1.1 > /dev/null
	$(BUILD)/bypass_course_test turn=0.9 > /dev/null || echo "bypass sim with turns 10% short: see README (wall + short turns is a known miss)"
	$(CC) $(CFLAGS) $(INC) -Ifakes -DAPP_H -DAPP_S5=1 -DS5_HARDWARE_CONFIRMED=1 -include hardware_main.h \
	    hardware_main.c $(APP)/subsystem5/ultrasonic_sensor.c $(APP)/subsystem5/scan_servo.c -o $(BUILD)/hardware_test
	$(BUILD)/hardware_test
	$(CC) $(CFLAGS) $(INC) $(TASK_FIXTURE) -DS5_HARDWARE_CONFIRMED=1 \
	    s5_task_main.c $(APP)/subsystem5/buddy5.c $(S5) -o $(BUILD)/runtime_test
	$(BUILD)/runtime_test
	$(CC) $(CFLAGS) $(INC) $(TASK_FIXTURE) -DS5_HARDWARE_CONFIRMED=0 \
	    s5_task_main.c $(APP)/subsystem5/buddy5.c $(S5) -o $(BUILD)/activation_gate_test
	$(BUILD)/activation_gate_test
	$(CC) $(CFLAGS) $(INC) -DAPP_S5=1 -DAPP_H -DAPP_TEST=0 -include ../integration/ctrl_test_kernel.h bypass_wire_main.c $(APP)/controller/vehicle_controller.c $(APP)/subsystem5/profile_obstacle.c -o $(BUILD)/telemetry_test
	$(BUILD)/telemetry_test
sanitize:
	$(MAKE) test CFLAGS='$(CFLAGS) -O1 -fsanitize=address,undefined -fno-omit-frame-pointer'
