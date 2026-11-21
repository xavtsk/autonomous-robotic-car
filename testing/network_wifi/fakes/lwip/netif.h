/** @file netif.h
 * @brief Expose fake network address accessors for Wi-Fi host tests.
 */
#ifndef TESTING_NETWORK_WIFI_FAKES_LWIP_NETIF_H
#define TESTING_NETWORK_WIFI_FAKES_LWIP_NETIF_H

#include "cyw43.h"

/** @brief Borrow the IPv4 address stored in the simulated interface. */
static inline ip4_addr_t * netif_ip4_addr (struct netif * p_netif)
{
    return &p_netif->address;
}

/** @brief Return the address word in the fake stack's stored byte order. */
static inline uint32_t ip4_addr_get_u32 (const ip4_addr_t * p_address)
{
    return p_address->addr;
}

#endif /* TESTING_NETWORK_WIFI_FAKES_LWIP_NETIF_H */

/* End of file. */

