#ifndef CAN_CTRL_H
#define CAN_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>

/* CANopen includes */
#include "CANopen.h"
#include "CO_app_STM32.h"

/* Configuration */
#define CAN_CTRL_NODE_ID            (24U)       /* This node ID */
#define CAN_CTRL_BAUDRATE           (125U)      /* 125 kbit/s */
#define CAN_CTRL_TASK_STACK_SIZE    (512U)      /* FreeRTOS task stack */
#define CAN_CTRL_TASK_PRIORITY      (osPriorityHigh)
#define CAN_CTRL_MAX_REMOTE_NODES   (16U)

/**
 * @brief Controller status codes
 */
typedef enum {
    CAN_CTRL_OK = 0,
    CAN_CTRL_ERROR,
    CAN_CTRL_ERROR_TIMEOUT,
    CAN_CTRL_ERROR_INVALID,
    CAN_CTRL_ERROR_NOT_INIT,
    CAN_CTRL_ERROR_CANOPEN
} can_ctrl_status_t;

/**
 * @brief PDO data structure (8 bytes max for standard PDO)
 */
typedef struct {
    uint8_t data[8];
    uint8_t len;
    uint32_t timestamp_ms;
} can_ctrl_pdo_data_t;

/**
 * @brief Response structure
 */
typedef struct {
    can_ctrl_status_t status;
    can_ctrl_pdo_data_t pdo_data;
} can_ctrl_resp_t;

/**
 * @brief CANopen controller handle
 */
typedef struct {
    /* CANopen STM32 integration */
    CANopenNodeSTM32 canopen_stm32;
    
    /* FreeRTOS task handle */
    osThreadId_t task_handle;
    
    /* HAL handles */
    FDCAN_HandleTypeDef* p_fdcan;
    TIM_HandleTypeDef* p_tim;
    
    /* State */
    volatile bool is_init;
    volatile bool is_operational;
    
    /* Latest received PDO data from remote nodes */
    can_ctrl_pdo_data_t rx_pdo_cache[CAN_CTRL_MAX_REMOTE_NODES];
    uint8_t remote_node_ids[CAN_CTRL_MAX_REMOTE_NODES];
    uint8_t remote_nodes_count;
    
} can_ctrl_handle_t;

/**
 * @brief Initialize CANopen controller
 * 
 * @param p_handle Pointer to controller handle
 * @param p_fdcan Pointer to FDCAN handle
 * @param p_tim Pointer to timer handle (must generate 1ms interrupts)
 * @param remote_node_ids Array of remote node IDs to monitor
 * @param nodes_count Number of remote nodes
 * @return can_ctrl_status_t Status
 */
can_ctrl_status_t can_ctrl_init(
    can_ctrl_handle_t* p_handle,
    FDCAN_HandleTypeDef* p_fdcan,
    TIM_HandleTypeDef* p_tim,
    const uint8_t* remote_node_ids,
    uint8_t nodes_count
);

/**
 * @brief Deinitialize CANopen controller
 * 
 * @param p_handle Pointer to controller handle
 * @return can_ctrl_status_t Status
 */
can_ctrl_status_t can_ctrl_deinit(can_ctrl_handle_t* p_handle);

/**
 * @brief Get latest PDO data from remote node
 * 
 * @param p_handle Pointer to controller handle
 * @param node_id Remote node ID
 * @param out_resp Output response structure
 * @return can_ctrl_status_t Status
 */
can_ctrl_status_t can_ctrl_get_pdo_data(
    can_ctrl_handle_t* p_handle,
    uint8_t node_id,
    can_ctrl_resp_t* out_resp
);

/**
 * @brief Send PDO data (TPDO)
 * 
 * @param p_handle Pointer to controller handle
 * @param data Pointer to data buffer
 * @param len Data length (max 8 bytes)
 * @return can_ctrl_status_t Status
 */
can_ctrl_status_t can_ctrl_send_pdo(
    can_ctrl_handle_t* p_handle,
    const uint8_t* data,
    uint8_t len
);

/**
 * @brief Check if controller is in operational state
 * 
 * @param p_handle Pointer to controller handle
 * @return true if operational, false otherwise
 */
bool can_ctrl_is_operational(can_ctrl_handle_t* p_handle);

/**
 * @brief Get NMT state
 * 
 * @param p_handle Pointer to controller handle
 * @return CO_NMT_internalState_t Current NMT state
 */
uint8_t can_ctrl_get_nmt_state(can_ctrl_handle_t* p_handle);

/**
 * @brief Timer interrupt callback (must be called from TIM IRQ)
 * Should be called from HAL_TIM_PeriodElapsedCallback
 */
void can_ctrl_timer_interrupt(can_ctrl_handle_t* p_handle);

#ifdef __cplusplus
}
#endif

#endif /* CAN_CTRL_H */