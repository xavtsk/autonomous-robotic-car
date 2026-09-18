# Host verification for Buddy 1's pure telemetry and command/configuration code.

CC      ?= cc
ROOT    := ../..
CFLAGS  := -std=c99 -Wall -Wextra -Wformat=2 -Wconversion -Wshadow \
           -Wswitch-enum -Wpedantic -Werror -O2 -g -I$(ROOT)/include

BUILD_DIR := $(ROOT)/build/host
TARGET    := $(BUILD_DIR)/buddy1_test
SAN_TARGET := $(BUILD_DIR)/buddy1_sanitize
SOURCES   := $(ROOT)/src/shared/robot_types.c \
             $(ROOT)/src/shared/robot_config.c \
             $(ROOT)/src/buddy1/telemetry.c \
             $(ROOT)/src/buddy1/command_receiver.c \
             host_test.c

.PHONY: test sanitize analyze clean

test: $(TARGET)
	@$(TARGET)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TARGET): $(SOURCES) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $(SOURCES)

sanitize: | $(BUILD_DIR)
	@$(CC) $(CFLAGS:-O2=-O1) -fsanitize=address,undefined \
		-fno-omit-frame-pointer -o $(SAN_TARGET) $(SOURCES)
	@$(SAN_TARGET) >/dev/null
	@echo "Buddy 1 clean under ASan+UBSan"

analyze:
	@clang --analyze -Xanalyzer -analyzer-output=text \
		-Xanalyzer -analyzer-werror -std=c99 -I$(ROOT)/include \
		$(ROOT)/src/shared/robot_types.c \
		$(ROOT)/src/shared/robot_config.c \
		$(ROOT)/src/buddy1/telemetry.c \
		$(ROOT)/src/buddy1/command_receiver.c
	@echo "Buddy 1 static analysis clean"

clean:
	rm -f $(TARGET) $(SAN_TARGET)
