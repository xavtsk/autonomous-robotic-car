#ifndef TEST_CYW43_H
#define TEST_CYW43_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#define CYW43_LINK_DOWN 0
#define CYW43_LINK_JOIN 1
#define CYW43_LINK_NOIP 2
#define CYW43_LINK_UP 3
#define CYW43_LINK_FAIL (-1)
#define CYW43_LINK_NONET (-2)
#define CYW43_LINK_BADAUTH (-3)
#define CYW43_AUTH_WPA2_MIXED_PSK 1U
#define CYW43_CHANNEL_NONE 0U
#define CYW43_ITF_STA 0
#define CYW43_EIO 5
typedef struct { uint32_t addr; } ip4_addr_t;
struct netif { ip4_addr_t address; };
typedef struct { unsigned itf_state; struct netif netif[1]; } cyw43_t;
extern cyw43_t cyw43_state;
extern void (*cyw43_poll)(void);
void cyw43_init(cyw43_t *state);
void cyw43_thread_enter(void);
void cyw43_thread_exit(void);
void cyw43_wifi_set_up(cyw43_t *state, int itf, bool up, unsigned country);
int cyw43_wifi_get_mac(cyw43_t *state, int itf, uint8_t mac[6]);
int cyw43_wifi_join(cyw43_t *state, size_t ssid_len, const uint8_t *ssid,
                    size_t key_len, const uint8_t *key, uint32_t auth,
                    const uint8_t *bssid, uint32_t channel);
int cyw43_wifi_leave(cyw43_t *state, int itf);
int cyw43_tcpip_link_status(cyw43_t *state, int itf);
#endif
