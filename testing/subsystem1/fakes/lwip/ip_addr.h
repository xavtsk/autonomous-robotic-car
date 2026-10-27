#ifndef HOST_IP_ADDR_H
#define HOST_IP_ADDR_H
typedef unsigned ip_addr_t;
#define ip_2_ip4(p) (p)
#define IPADDR_TYPE_V4 0
#define IP_SET_TYPE(p, type) ((void)(p))
int ip4addr_aton(const char *text, ip_addr_t *addr);
#endif
