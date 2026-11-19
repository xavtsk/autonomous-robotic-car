/** @file netif.h
 * @brief Expose fake network address accessors for Wi-Fi host tests.
 */
#include "cyw43.h"
#define netif_ip4_addr(netif) (&(netif)->address)
#define ip4_addr_get_u32(address) ((address)->addr)

/* End of file. */

