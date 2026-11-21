/** @file ip_addr.h
 * @brief Provide fake IPv4 address types and conversion hooks for MQTT tests.
 */
#ifndef HOST_IP_ADDR_H
#define HOST_IP_ADDR_H
typedef unsigned ip_addr_t;

/** @brief Borrow the writable IPv4 address used by the host parser. */
static inline ip_addr_t * ip_2_ip4 (ip_addr_t * p_address)
{
    return p_address;
}

#define IPADDR_TYPE_V4 0
#define IP_SET_TYPE(p, type) ((void)(p))
int ip4addr_aton (const char * p_text, ip_addr_t * p_address);
#endif

/* End of file. */

