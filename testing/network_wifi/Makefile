CC = cc
ROOT = ../..
DRIVER = $(ROOT)/lib/libwifi/sysdepend/pico_rp2040
BUILD = $(ROOT)/build/network_wifi
CPPFLAGS = -Ifakes -I$(DRIVER) -DCNF_SMP=1 -DTM_NET_LWIP=1 -DTM_MQTT=1
CFLAGS = -std=c11 -Wall -Wextra -Werror -O2 -g

.PHONY: test sanitize
test:
	mkdir -p $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) policy_main.c -o $(BUILD)/policy_test
	$(BUILD)/policy_test
	$(CC) $(CPPFLAGS) $(CFLAGS) wifi_service_main.c $(DRIVER)/cyw43_utk.c -o $(BUILD)/runtime_test
	$(BUILD)/runtime_test

sanitize:
	$(MAKE) test CFLAGS='$(CFLAGS) -O1 -fsanitize=address,undefined -fno-omit-frame-pointer'
