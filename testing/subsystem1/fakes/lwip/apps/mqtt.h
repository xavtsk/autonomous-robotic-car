#ifndef HOST_MQTT_H
#define HOST_MQTT_H
#include <stdint.h>
typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int err_t;
#define ERR_OK 0
#define ERR_MEM -1
#define ERR_TIMEOUT -3
#define ERR_ARG -16
#define ERR_CONN -11
#define MQTT_CONNECT_ACCEPTED 0
#define MQTT_DATA_FLAG_LAST 1
typedef int mqtt_connection_status_t;
typedef struct { int connected; } mqtt_client_t;
struct mqtt_connect_client_info_t { const char *client_id; unsigned short keep_alive; };
typedef void (*host_connection_cb)(mqtt_client_t *, void *, mqtt_connection_status_t);
typedef void (*host_publish_cb)(void *, const char *, u32_t);
typedef void (*host_data_cb)(void *, const u8_t *, u16_t, u8_t);
typedef void (*host_done_cb)(void *, err_t);
mqtt_client_t *mqtt_client_new(void);
int mqtt_client_is_connected(mqtt_client_t *c);
err_t mqtt_client_connect(mqtt_client_t *c, const void *addr, unsigned short port,
                          host_connection_cb cb, void *arg, const struct mqtt_connect_client_info_t *info);
void mqtt_set_inpub_callback(mqtt_client_t *c, host_publish_cb p, host_data_cb d, void *arg);
err_t mqtt_subscribe(mqtt_client_t *c, const char *topic, u8_t qos, host_done_cb cb, void *arg);
err_t mqtt_publish(mqtt_client_t *c, const char *topic, const void *payload, u16_t len,
                   u8_t qos, u8_t retain, host_done_cb cb, void *arg);
#endif
