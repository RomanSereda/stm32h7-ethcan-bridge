#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief MQTT Interface Handle Structure
 */
typedef struct {
    mqtt_client_t *client;          /**< LwIP MQTT client instance */
    ip_addr_t broker_ip;            /**< Broker IP address */
    uint16_t broker_port;           /**< Broker Port */
    struct mqtt_connect_client_info_t client_info; /**< Client ID, credentials, etc. */
    
    volatile uint8_t is_connected;      /**< Connection flag */
    volatile uint8_t pub_in_progress;   /**< Publish busy flag */
    uint32_t last_pub_attempt;          /**< Timestamp of last publish */
} mqtt_client_handle_t;

/**
 * @brief Initialize the MQTT interface
 * * @param[in,out] p_handle Pointer to handle
 * @param[in] broker_ip    IP address of the broker
 * @param[in] client_id    Unique client ID string
 */
void mqtt_client_init(mqtt_client_handle_t *p_handle, const ip_addr_t *broker_ip, const char *client_id);

/**
 * @brief Ensure connection to the broker. 
 * If not connected, attempts to connect (blocking delay on fail).
 * * @param[in,out] p_handle Pointer to handle
 * @return err_t ERR_OK if connected/reconnecting, error code otherwise
 */
err_t mqtt_client_connect_process(mqtt_client_handle_t *p_handle);

/**
 * @brief Publish data to a topic
 * * @param[in,out] p_handle Pointer to handle
 * @param[in] topic        Topic string
 * @param[in] payload      Data to send
 * @return err_t ERR_OK if request sent, or error code
 */
err_t mqtt_client_publish(mqtt_client_handle_t *p_handle, const char *topic, const char *payload);

/**
 * @brief Check for publish timeouts and reset flags if necessary
 * * @param[in,out] p_handle Pointer to handle
 */
void mqtt_client_check_timeout(mqtt_client_handle_t *p_handle);

#ifdef __cplusplus
}
#endif

#endif /* MQTT_CLIENT_H */