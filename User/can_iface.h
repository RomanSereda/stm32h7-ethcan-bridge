#ifndef CAN_IFACE_H
#define CAN_IFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define CAN_IFACE_QUEUE_SIZE (32U)
#define CAN_IFACE_MAX_DATA_LEN (8U)

#ifndef CAN_IFACE_MAX_INSTANCES
#define CAN_IFACE_MAX_INSTANCES 2U
#endif

/**
 * @brief CAN message structure
 */
typedef struct {
    uint32_t id;                        /**< CAN identifier */
    uint8_t data[CAN_IFACE_MAX_DATA_LEN]; /**< Data bytes */
    uint8_t len;                        /**< Data length code (0-8) */
    bool is_extended;                   /**< Extended ID flag */
    bool is_rtr;                        /**< Remote transmission request */
} can_msg_t;

/**
 * @brief CAN iface status codes
 */
typedef enum {
    CAN_IFACE_OK = 0,            /**< Operation successful */
    CAN_IFACE_ERROR,             /**< General error */
    CAN_IFACE_ERROR_TIMEOUT,     /**< Timeout error */
    CAN_IFACE_ERROR_QUEUE_FULL,  /**< Queue is full */
    CAN_IFACE_ERROR_QUEUE_EMPTY, /**< Queue is empty */
    CAN_IFACE_ERROR_INVALID,     /**< Invalid parameter */
    CAN_IFACE_ERROR_NOT_INIT     /**< Module not initialized */
} can_iface_status_t;

/**
 * @brief CAN iface configuration structure
 */
typedef struct {
    uint32_t len;  /**< Number of filter IDs */
    uint32_t* ids; /**< List of filter IDs */
} can_filter_config_t;

/**
 * @brief CAN iface handle structure
 */
typedef struct {
    FDCAN_HandleTypeDef* p_fdcan;      /**< Pointer to HAL FDCAN handle */
    QueueHandle_t rx_queue;            /**< FreeRTOS queue for RX messages */
    volatile bool is_init;             /**< Initialization flag */
    volatile uint32_t last_error;      /**< Last FDCAN error code (updated in ISR) */
    volatile uint32_t cnt_errorr;      /**< Count of errors occurred */
} can_iface_handle_t;

/**
 * @brief Initialize CAN iface module (registers instance, starts FDCAN, enables interrupts)
 *
 * @param[in,out] p_handle  Pointer to CAN iface handle (must be persistent)
 * @param[in]     p_fdcan   Pointer to HAL FDCAN handle (must be initialized by MX)
 * @param[in]     p_config  Pointer to configuration (NULL for defaults)
 *
 * @return Status code
 */
can_iface_status_t can_iface_init(
    can_iface_handle_t* p_handle, FDCAN_HandleTypeDef* p_fdcan, can_filter_config_t const* p_config);

/**
 * @brief Deinitialize CAN iface module (deregister, stop FDCAN, free queue)
 *
 * @param[in,out] p_handle  Pointer to CAN iface handle
 *
 * @return Status code
 */
can_iface_status_t can_iface_deinit(can_iface_handle_t* p_handle);

/**
 * @brief Transmit CAN message (blocking with timeout)
 */
can_iface_status_t can_iface_transmit(
    can_iface_handle_t const* p_handle, can_msg_t const* p_msg, uint32_t timeout_ms);

/**
 * @brief Receive CAN message from queue (blocks up to timeout_ms)
 */
can_iface_status_t can_iface_receive(
    can_iface_handle_t const* p_handle, can_msg_t* p_msg, uint32_t timeout_ms);

/**
 * @brief Peek CAN message from queue without removing it
 */
can_iface_status_t can_iface_peek(
    can_iface_handle_t const* p_handle, can_msg_t* p_msg, uint32_t timeout_ms);

/**
 * @brief Get number of messages in RX queue
 */
uint32_t can_iface_get_queue_count(can_iface_handle_t const* p_handle);

/**
 * @brief Check if RX queue is empty
 */
bool can_iface_is_queue_empty(can_iface_handle_t const* p_handle);

#ifdef __cplusplus
}
#endif

#endif /* CAN_IFACE_H */
