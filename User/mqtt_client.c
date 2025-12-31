#include "mqtt_client.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

static void mqtt_connection_cb(mqtt_client_t* client, void* arg, mqtt_connection_status_t status);
static void mqtt_pub_request_cb(void* arg, err_t result);

void mqtt_client_init(
    mqtt_client_handle_t* p_handle, const ip_addr_t* broker_ip, const char* client_id) {
    if (p_handle == NULL) return;

    memset(p_handle, 0, sizeof(mqtt_client_handle_t));

    p_handle->broker_ip = *broker_ip;
    p_handle->broker_port = 1883;

    p_handle->client_info.client_id = client_id;
    p_handle->client_info.keep_alive = 60;

    p_handle->client = mqtt_client_new();
}

err_t mqtt_client_connect_process(mqtt_client_handle_t* p_handle) {
    if (mqtt_client_is_connected(p_handle->client)) { return ERR_OK; /* Already connected */ }

    printf("MQTT: Not connected, connecting...\n");
    p_handle->is_connected = 0;
    p_handle->pub_in_progress = 0;

    err_t err = mqtt_client_connect(p_handle->client, &p_handle->broker_ip, p_handle->broker_port,
        mqtt_connection_cb, p_handle, &p_handle->client_info);

    if (err != ERR_OK) { printf("MQTT: Connect failed with error: %d\n", err); }

    osDelay(5000);
    return err;
}

void mqtt_client_check_timeout(mqtt_client_handle_t* p_handle) {
    if (p_handle->pub_in_progress && (HAL_GetTick() - p_handle->last_pub_attempt > 10000)) {
        printf("MQTT: Timeout detected, force reset flag\n");
        p_handle->pub_in_progress = 0;
    }
}

err_t mqtt_client_publish(mqtt_client_handle_t* p_handle, const char* topic, const char* payload) {
    if (!p_handle->is_connected || p_handle->pub_in_progress) { return ERR_INPROGRESS; }

    p_handle->pub_in_progress = 1;
    p_handle->last_pub_attempt = HAL_GetTick();

    err_t err = mqtt_publish(p_handle->client, topic, payload, strlen(payload), 0, 0,
        mqtt_pub_request_cb, p_handle); /* Pass handle as arg */

    if (err != ERR_OK) {
        printf("MQTT: Pub immediate err: %d\n", err);
        p_handle->pub_in_progress = 0;
    }

    return err;
}

static void mqtt_connection_cb(mqtt_client_t* client, void* arg, mqtt_connection_status_t status) {
    mqtt_client_handle_t* p_handle = (mqtt_client_handle_t*)arg;

    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("MQTT: Connected successfully!\n");
        p_handle->is_connected = 1;
    } else {
        printf("MQTT: Connection failed, status: %d\n", status);
        p_handle->is_connected = 0;
        p_handle->pub_in_progress = 0;
    }
}

static void mqtt_pub_request_cb(void* arg, err_t result) {
    mqtt_client_handle_t* p_handle = (mqtt_client_handle_t*)arg;

    if (p_handle != NULL) { p_handle->pub_in_progress = 0; }

    if (result != ERR_OK) {
        printf("MQTT: Pub request err: %d\n", result);
    } else {
        printf("MQTT: Pub request OK\n");
    }
}