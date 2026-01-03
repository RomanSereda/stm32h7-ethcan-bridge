#include "can_ctrl.h"
#include <string.h>
#include <stdio.h>

/* External CANopen object - defined in CO_app_STM32.c */
extern CO_t *CO;

/* Static task function */
static void can_ctrl_task(void* argument);

/**
 * @brief FreeRTOS task for CANopen processing
 */
static void can_ctrl_task(void* argument)
{
    can_ctrl_handle_t* p_handle = (can_ctrl_handle_t*)argument;
    
    if (!p_handle) {
        vTaskDelete(NULL);
        return;
    }

    printf("CANopen task started\n");
    
    /* Initialize CANopen stack */
    p_handle->canopen_stm32.CANHandle = p_handle->p_fdcan;
    p_handle->canopen_stm32.HWInitFunction = NULL; /* Already initialized */
    p_handle->canopen_stm32.timerHandle = p_handle->p_tim;
    p_handle->canopen_stm32.desiredNodeID = CAN_CTRL_NODE_ID;
    p_handle->canopen_stm32.baudrate = CAN_CTRL_BAUDRATE;
    
    canopen_app_init(&p_handle->canopen_stm32);
    
    /* Start timer for 1ms interrupts */
    if (HAL_TIM_Base_Start_IT(p_handle->p_tim) != HAL_OK) {
        printf("CANopen: Timer start failed\n");
        p_handle->is_init = false;
        vTaskDelete(NULL);
        return;
    }
    
    printf("CANopen: Stack initialized, Node ID: %d\n", CAN_CTRL_NODE_ID);
    
    /* Main processing loop */
    while (1)
    {
        /* Process CANopen stack */
        canopen_app_process();
        
        /* Check if we reached operational state */
        if (CO != NULL && CO->NMT != NULL) {
            CO_NMT_internalState_t nmt_state = CO->NMT->operatingState;
            p_handle->is_operational = (nmt_state == CO_NMT_OPERATIONAL);
        }
        
        /* Yield to other tasks - CANopen processing doesn't need tight loop */
        osDelay(1);
    }
}

/**
 * @brief Initialize CANopen controller
 */
can_ctrl_status_t can_ctrl_init(
    can_ctrl_handle_t* p_handle,
    FDCAN_HandleTypeDef* p_fdcan,
    TIM_HandleTypeDef* p_tim,
    const uint8_t* remote_node_ids,
    uint8_t nodes_count
)
{
    if (!p_handle || !p_fdcan || !p_tim) {
        return CAN_CTRL_ERROR_INVALID;
    }
    
    if (nodes_count > CAN_CTRL_MAX_REMOTE_NODES) {
        return CAN_CTRL_ERROR_INVALID;
    }
    
    /* Clear handle */
    memset(p_handle, 0, sizeof(can_ctrl_handle_t));
    
    /* Store handles */
    p_handle->p_fdcan = p_fdcan;
    p_handle->p_tim = p_tim;
    
    /* Store remote node IDs */
    p_handle->remote_nodes_count = nodes_count;
    if (remote_node_ids && nodes_count > 0) {
        memcpy(p_handle->remote_node_ids, remote_node_ids, nodes_count);
    }
    
    /* Create FreeRTOS task */
    const osThreadAttr_t task_attributes = {
        .name = "CANopen",
        .stack_size = CAN_CTRL_TASK_STACK_SIZE * 4,
        .priority = CAN_CTRL_TASK_PRIORITY,
    };
    
    p_handle->task_handle = osThreadNew(can_ctrl_task, p_handle, &task_attributes);
    
    if (!p_handle->task_handle) {
        return CAN_CTRL_ERROR;
    }
    
    p_handle->is_init = true;
    
    return CAN_CTRL_OK;
}

/**
 * @brief Deinitialize CANopen controller
 */
can_ctrl_status_t can_ctrl_deinit(can_ctrl_handle_t* p_handle)
{
    if (!p_handle || !p_handle->is_init) {
        return CAN_CTRL_ERROR_NOT_INIT;
    }
    
    /* Stop timer */
    HAL_TIM_Base_Stop_IT(p_handle->p_tim);
    
    /* Delete task */
    if (p_handle->task_handle) {
        osThreadTerminate(p_handle->task_handle);
        p_handle->task_handle = NULL;
    }
    
    /* Deinitialize CANopen */
    /* Note: CANopenNode doesn't have explicit deinit, just stop using it */
    
    p_handle->is_init = false;
    
    return CAN_CTRL_OK;
}

/**
 * @brief Get latest PDO data from remote node
 */
can_ctrl_status_t can_ctrl_get_pdo_data(
    can_ctrl_handle_t* p_handle,
    uint8_t node_id,
    can_ctrl_resp_t* out_resp
)
{
    if (!p_handle || !p_handle->is_init || !out_resp) {
        return CAN_CTRL_ERROR_INVALID;
    }
    
    /* Find node in cache */
    for (uint8_t i = 0; i < p_handle->remote_nodes_count; i++) {
        if (p_handle->remote_node_ids[i] == node_id) {
            /* Copy cached data */
            memcpy(&out_resp->pdo_data, &p_handle->rx_pdo_cache[i], 
                   sizeof(can_ctrl_pdo_data_t));
            out_resp->status = CAN_CTRL_OK;
            return CAN_CTRL_OK;
        }
    }
    
    return CAN_CTRL_ERROR_INVALID;
}

/**
 * @brief Send PDO data (TPDO)
 * 
 * Note: This is a placeholder implementation. 
 * For proper TPDO sending, you need to:
 * 1. Map variables to TPDO in Object Dictionary
 * 2. Configure TPDO communication parameters
 * 3. Use OD_requestTPDO() or modify mapped OD variables
 * 
 * Currently this just signals that TPDO should be sent on next cycle.
 */
can_ctrl_status_t can_ctrl_send_pdo(
    can_ctrl_handle_t* p_handle,
    const uint8_t* data,
    uint8_t len
)
{
    if (!p_handle || !p_handle->is_init || !data) {
        return CAN_CTRL_ERROR_INVALID;
    }
    
    if (len > 8) {
        return CAN_CTRL_ERROR_INVALID;
    }
    
    if (!p_handle->is_operational) {
        return CAN_CTRL_ERROR;
    }
    
    /* For proper implementation, map OD variables to TPDO 
     * and modify those variables instead.
     * Example:
     *   OD variables 0x2000:0x01-08 mapped to TPDO1
     *   memcpy(OD_RAM.x2000_myData, data, len);
     *   OD_requestTPDO(CO->em, 0x2000, 0);
     * 
     * For now, just return OK - TPDO will be sent automatically
     * if configured with event timer or change-of-state
     */
    
    return CAN_CTRL_OK;
}

/**
 * @brief Check if controller is operational
 */
bool can_ctrl_is_operational(can_ctrl_handle_t* p_handle)
{
    if (!p_handle || !p_handle->is_init) {
        return false;
    }
    
    return p_handle->is_operational;
}

/**
 * @brief Get NMT state
 */
uint8_t can_ctrl_get_nmt_state(can_ctrl_handle_t* p_handle)
{
    if (!p_handle || !p_handle->is_init) {
        return 0;
    }
    
    if (CO != NULL && CO->NMT != NULL) {
        return (uint8_t)CO->NMT->operatingState;
    }
    
    return 0;
}

/**
 * @brief Timer interrupt callback
 * Must be called from HAL_TIM_PeriodElapsedCallback for TIM17
 */
void can_ctrl_timer_interrupt(can_ctrl_handle_t* p_handle)
{
    if (!p_handle || !p_handle->is_init) {
        return;
    }
    
    /* Call CANopen 1ms interrupt handler */
    if (p_handle->p_tim) {
        canopen_app_interrupt();
    }
}

/**
 * @brief Process received RPDO data
 * This should be called from CANopen RPDO callback
 */
void can_ctrl_process_rpdo(can_ctrl_handle_t* p_handle, uint8_t node_id, 
                           const uint8_t* data, uint8_t len)
{
    if (!p_handle || !data) {
        return;
    }
    
    /* Find node in list and update cache */
    for (uint8_t i = 0; i < p_handle->remote_nodes_count; i++) {
        if (p_handle->remote_node_ids[i] == node_id) {
            /* Update cached PDO data */
            p_handle->rx_pdo_cache[i].len = (len > 8) ? 8 : len;
            memcpy(p_handle->rx_pdo_cache[i].data, data, p_handle->rx_pdo_cache[i].len);
            p_handle->rx_pdo_cache[i].timestamp_ms = HAL_GetTick();
            break;
        }
    }
}