CC = cc
APP = ../../app_program
BUILD = ../../build/buddy4
CFLAGS = -std=c11 -Wall -Wextra -Werror -pedantic -O2 -g
CORE = $(addprefix $(APP)/subsystem4/,imu_sensor.c imu_calibration.c tilt_estimation.c hump_detection.c motion_events.c terrain_monitor.c)
S3 = $(addprefix $(APP)/subsystem3/,ir_acquisition.c ir_calibration.c ir_reflectance.c line_detection.c line_controller.c barcode_decoder.c line_navigation.c navigation_codec.c)
CONTROL = $(addprefix $(APP)/controller/,s3_drive.c terrain_integration.c barcode_capture.c integration_codec.c)
FAKE = -DAPP_H -DAPP_S4=1 -DAPP_TEST=0 -include ../integration/controller_test_kernel.h -I../integration -I../integration/fakes
.PHONY: test sanitize
test:
	mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -I$(APP)/subsystem4 host_test.c $(CORE) -lm -o $(BUILD)/host_test
	$(BUILD)/host_test
	$(CC) $(CFLAGS) $(FAKE) integration_test.c $(CONTROL) $(S3) -lm -o $(BUILD)/integration_test
	$(BUILD)/integration_test
	$(CC) $(CFLAGS) $(FAKE) -Ifakes transport_test.c $(CORE) -lm -o $(BUILD)/transport_test
	$(BUILD)/transport_test
sanitize:
	$(MAKE) test CFLAGS='$(CFLAGS) -O1 -fsanitize=address,undefined -fno-omit-frame-pointer'
