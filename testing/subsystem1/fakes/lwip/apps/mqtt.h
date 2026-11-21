/** @file mqtt.h
 * @brief Declare fake MQTT callbacks, errors and client operations for host
 * tests.
 */
#ifndef HOST_MQTT_H
#define HOST_MQTT_H
#include <stdint.h>
typedef uint8_t  u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int      err_t;
#define ERR_OK 0
#define ERR_MEM -1
#define ERR_TIMEOUT -3
#define ERR_ARG -16
#define ERR_CONN -11
#define MQTT_CONNECT_ACCEPTED 0
#define MQTT_DATA_FLAG_LAST 1
typedef int mqtt_connection_status_t;

typedef struct
{
    int connected;
} mqtt_client_t;

typedef struct mqtt_connect_client_info_t
{
    const char * client_id;
    u16_t        keep_alive;
} mqtt_connect_client_info_t;

typedef void ( * mqtt_host_connection_fn_t)(mqtt_client_t *          p_client,
                                          void *                   p_context,
                                          mqtt_connection_status_t status);
typedef void ( * mqtt_host_publish_fn_t)(void * p_context, const char * p_topic,
                                       u32_t total_length);
typedef void ( * mqtt_host_data_fn_t)(void * p_context, const u8_t * p_data,
                                    u16_t length, u8_t flags);
typedef void ( * mqtt_host_done_fn_t)(void * p_context, err_t error);
mqtt_client_t * mqtt_client_new (void);
int             mqtt_client_is_connected (mqtt_client_t * p_client);
err_t mqtt_client_connect (mqtt_client_t * p_client, const void * p_address,
                           u16_t port, mqtt_host_connection_fn_t p_callback,
                           void *                             p_context,
                           const mqtt_connect_client_info_t * p_info);
void  mqtt_set_inpub_callback (mqtt_client_t *        p_client,
                               mqtt_host_publish_fn_t p_incoming,
                               mqtt_host_data_fn_t p_on_data, void * p_context);
err_t mqtt_subscribe (mqtt_client_t * p_client, const char * p_topic, u8_t qos,
                      mqtt_host_done_fn_t p_callback, void * p_context);
err_t mqtt_publish (mqtt_client_t * p_client, const char * p_topic,
                    const void * p_payload, u16_t len, u8_t qos, u8_t retain,
                    mqtt_host_done_fn_t p_callback, void * p_context);
#endif

/* End of file. */

