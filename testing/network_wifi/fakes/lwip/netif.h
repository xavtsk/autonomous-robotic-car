/** @file netif.h
 * @brief Expose fake network address accessors for Wi-Fi host tests.
 */
#ifndef TESTING_NETWORK_WIFI_FAKES_LWIP_NETIF_H
#define TESTING_NETWORK_WIFI_FAKES_LWIP_NETIF_H

#include "cyw43.h"
#define netif_ip4_addr(netif) (&(netif)->address)
#define ip4_addr_get_u32(address) ((address)->addr)

#endif /* TESTING_NETWORK_WIFI_FAKES_LWIP_NETIF_H */

/* End of file. */

