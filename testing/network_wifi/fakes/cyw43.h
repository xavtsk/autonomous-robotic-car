/** @file cyw43.h
 * @brief Declare fake CYW43 link states and radio operations for host tests.
 */
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

typedef struct
{
    uint32_t addr;
} ip4_addr_t;

typedef struct netif
{
    ip4_addr_t address;
} cyw43_test_netif_t;

typedef struct
{
    unsigned           itf_state;
    cyw43_test_netif_t netif[1];
} cyw43_t;

extern cyw43_t cyw43_state;
extern void    ( * cyw43_poll)(void);
void           cyw43_init (cyw43_t * p_state);
void           cyw43_thread_enter (void);
void           cyw43_thread_exit (void);
void           cyw43_wifi_set_up (cyw43_t * p_state, int itf, bool b_is_up,
                                  unsigned country);
int            cyw43_wifi_get_mac (cyw43_t * p_state, int itf, uint8_t mac[6]);
int cyw43_wifi_join (cyw43_t * p_state, size_t ssid_len, const uint8_t * p_ssid,
                     size_t key_len, const uint8_t * p_key, uint32_t auth,
                     const uint8_t * p_bssid, uint32_t channel);
int cyw43_wifi_leave (cyw43_t * p_state, int itf);
int cyw43_tcpip_link_status (cyw43_t * p_state, int itf);
#endif

/* End of file. */

