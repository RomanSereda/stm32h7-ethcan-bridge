#ifndef CAN_CTRL_H
#define CAN_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "can_iface.h"

#define CAN_CTRL_MAX_IDS_LEN (16U)
#define CAN_CTRL_DATA_OFFSET (16U)

/* Configuration defaults */
#ifndef CAN_CTRL_DEFAULT_TIMEOUT_MS
#define CAN_CTRL_DEFAULT_TIMEOUT_MS (1000U) /* 1 second default resp timeout */
#endif

/**
 * @brief Controller status codes
 */
typedef enum {
    CAN_CTRL_OK = 0,        /**< Operation successful */
    CAN_CTRL_ERROR,         /**< General error */
    CAN_CTRL_ERROR_TIMEOUT, /**< Response timeout */
    CAN_CTRL_ERROR_INVALID, /**< Invalid parameter */
    CAN_CTRL_ERROR_NOT_INIT /**< Controller not initialized */
} can_ctrl_status_t;

/**
 * @brief Controller resp structure
 */
typedef struct {
    can_ctrl_status_t status; /**< status of the request */
    can_msg_t msg;            /**< valid when status == CAN_CTRL_OK */
} can_ctrl_resp_t;

/**
 * @brief Controller handle
 */
typedef struct {
    can_iface_handle_t p_iface;         /**< pointer to lower-level CAN iface */
    volatile bool is_init;              /**< initialization flag */
    uint32_t ids[CAN_CTRL_MAX_IDS_LEN]; /**< list of monitored IDs */
    uint32_t ids_len;                   /**< number of monitored IDs */
} can_ctrl_handle_t;

/**
 * @brief Initialize CAN controller.
 */
can_ctrl_status_t can_ctrl_init(can_ctrl_handle_t* p_handle, FDCAN_HandleTypeDef* p_fdcan,
    const uint32_t* ids, uint32_t ids_len);

/**
 * @brief Deinitialize CAN controller.
 */
can_ctrl_status_t can_ctrl_deinit(can_ctrl_handle_t* p_handle);

/**
 * @brief Synchronously request data from remote node via RTR and wait for resp.
 */
can_ctrl_status_t can_ctrl_request_sync(
    can_ctrl_handle_t* p_handle, uint32_t index, bool is_extended, can_ctrl_resp_t* out_resp);

#ifdef __cplusplus
}
#endif

#endif /* CAN_CTRL_H */