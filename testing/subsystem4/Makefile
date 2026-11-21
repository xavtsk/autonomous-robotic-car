CC = cc
APP = ../../app_program
BUILD = ../../build/buddy4
CFLAGS = -std=c11 -Wall -Wextra -Werror -pedantic -O2 -g
# Python 3 selects the reviewed private layout; override with PYTHON=/path/to/python.
PYTHON ?= python3
PRIVATE_STORAGE_FLAGS := $(shell $(PYTHON) ../../tools/compliance/instrument_harnesses.py --private-storage-flag ../..)
ifneq ($(.SHELLSTATUS),0)
$(error Private storage detection failed; verify PYTHON names a Python 3 interpreter)
endif
override CFLAGS += $(PRIVATE_STORAGE_FLAGS)
CORE = $(addprefix $(APP)/subsystem4/,imu_sensor.c imu_calibration.c tilt.c hump_detection.c events_motion.c terr_data.c)
S3 = $(addprefix $(APP)/subsystem3/,ir_acquisition.c ir_calibration.c ir_reflectance.c line_detection.c line_controller.c bc_decoder.c line_navigation.c navigation_codec.c)
CONTROL = $(addprefix $(APP)/controller/,s3_drive.c terr_link.c bc_capture.c integration_codec.c)
FAKE = -DAPP_H -DAPP_S4=1 -DAPP_TEST=0 -include ../integration/ctrl_test_kernel.h -I../integration -I../integration/fakes
.PHONY: test sanitize
test:
	mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -DAPP_IMU_DEBUG=1 -I$(APP)/subsystem4 imu_host_main.c $(CORE) -lm -o $(BUILD)/host_test
	$(BUILD)/host_test
	$(CC) $(CFLAGS) $(FAKE) -DTEST_SLOPE_LINK -DS3_RUNTIME_TEST -include ../integration/runtime_main.h -I$(APP)/controller \
	    slope_link_main.c ../integration/runtime_main.c $(APP)/controller/s3_integration.c $(CONTROL) $(S3) -lm -o $(BUILD)/integration_test
	$(BUILD)/integration_test
	$(CC) $(CFLAGS) $(FAKE) -DAPP_IMU_DEBUG=1 -DBUDDY4_TRANSPORT_TEST -I. -Ifakes \
	    transport_main.c $(APP)/subsystem4/buddy4.c $(CORE) -lm -o $(BUILD)/transport_test
	$(BUILD)/transport_test
sanitize:
	$(MAKE) test CFLAGS='$(CFLAGS) -O1 -fsanitize=address,undefined -fno-omit-frame-pointer'
