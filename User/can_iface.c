#include "can_iface.h"

/* Internal registry */
static can_iface_handle_t* s_instances[CAN_IFACE_MAX_INSTANCES] = {0};

/* Internal helpers */
static bool is_handle_valid(can_iface_handle_t const* p_handle);
static void convert_hal_to_msg(
    FDCAN_RxHeaderTypeDef const* p_rx_header, uint8_t const* p_data, can_msg_t* p_msg);
static void convert_msg_to_hal(can_msg_t const* p_msg, FDCAN_TxHeaderTypeDef* p_tx_header);
static can_iface_status_t configure_default_filter(
    FDCAN_HandleTypeDef* p_fdcan, uint32_t filter_index);
static can_iface_status_t configure_filters_from_list(
    FDCAN_HandleTypeDef* p_fdcan, const uint32_t* ids, size_t len, uint32_t start_filter_index);
static can_iface_handle_t* find_handle_by_hw(FDCAN_HandleTypeDef* p_fdcan);
static void can_iface_rx_callback(can_iface_handle_t* p_handle, uint32_t RxFifo0ITs);

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
    can_iface_handle_t* h = find_handle_by_hw(hfdcan);
    if (h != NULL) { can_iface_rx_callback(h, RxFifo0ITs); }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef* hfdcan) {
    can_iface_handle_t* h = find_handle_by_hw(hfdcan);
    if (h == NULL) return;

    uint32_t err = HAL_FDCAN_GetError(hfdcan);
    h->last_error = err;

    h->cnt_errorr++;
    __NOP();
}

can_iface_status_t can_iface_init(can_iface_handle_t* p_handle, FDCAN_HandleTypeDef* p_fdcan,
    can_filter_config_t const* p_config) {
    can_iface_status_t status = CAN_IFACE_OK;

    if ((NULL == p_handle) || (NULL == p_fdcan)) { return CAN_IFACE_ERROR_INVALID; }

    memset(p_handle, 0, sizeof(can_iface_handle_t));
    p_handle->p_fdcan = p_fdcan;

    p_handle->rx_queue = xQueueCreate(CAN_IFACE_QUEUE_SIZE, sizeof(can_msg_t));
    if (NULL == p_handle->rx_queue) { return CAN_IFACE_ERROR; }

    if (NULL == p_config) {
        status = configure_default_filter(p_fdcan, 0);
        if (CAN_IFACE_OK != status) {
            vQueueDelete(p_handle->rx_queue);
            p_handle->rx_queue = NULL;
            return status;
        }
    } else {
        status =
            configure_filters_from_list(p_fdcan, p_config->ids, p_config->len, 0);
        if (CAN_IFACE_OK != status) {
            vQueueDelete(p_handle->rx_queue);
            p_handle->rx_queue = NULL;
            return status;
        }
    }

    if (HAL_FDCAN_Start(p_fdcan) != HAL_OK) {
        vQueueDelete(p_handle->rx_queue);
        p_handle->rx_queue = NULL;
        return CAN_IFACE_ERROR;
    }

    if (HAL_FDCAN_ActivateNotification(p_fdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        HAL_FDCAN_Stop(p_fdcan);
        vQueueDelete(p_handle->rx_queue);
        p_handle->rx_queue = NULL;
        return CAN_IFACE_ERROR;
    }

    for (uint32_t i = 0; i < CAN_IFACE_MAX_INSTANCES; ++i) {
        if (s_instances[i] == NULL) {
            s_instances[i] = p_handle;
            p_handle->is_init = true;
            p_handle->last_error = 0;
            return CAN_IFACE_OK;
        }
    }

    HAL_FDCAN_DeactivateNotification(p_fdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    HAL_FDCAN_Stop(p_fdcan);
    vQueueDelete(p_handle->rx_queue);
    p_handle->rx_queue = NULL;
    return CAN_IFACE_ERROR;
}

can_iface_status_t can_iface_deinit(can_iface_handle_t* p_handle) {
    if (!is_handle_valid(p_handle)) { return CAN_IFACE_ERROR_INVALID; }

    for (uint32_t i = 0; i < CAN_IFACE_MAX_INSTANCES; ++i) {
        if (s_instances[i] == p_handle) {
            s_instances[i] = NULL;
            break;
        }
    }

    if (HAL_FDCAN_DeactivateNotification(p_handle->p_fdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE) !=
        HAL_OK) {
        /* continue anyway */
    }

    if (HAL_FDCAN_Stop(p_handle->p_fdcan) != HAL_OK) { /* continue anyway */
    }

    if (NULL != p_handle->rx_queue) {
        vQueueDelete(p_handle->rx_queue);
        p_handle->rx_queue = NULL;
    }

    p_handle->is_init = false;
    p_handle->p_fdcan = NULL;
    return CAN_IFACE_OK;
}

can_iface_status_t can_iface_transmit(
    can_iface_handle_t const* p_handle, can_msg_t const* p_msg, uint32_t timeout_ms) {
    if (!is_handle_valid(p_handle) || (NULL == p_msg)) { return CAN_IFACE_ERROR_INVALID; }
    if (p_msg->len > CAN_IFACE_MAX_DATA_LEN) { return CAN_IFACE_ERROR_INVALID; }

    FDCAN_TxHeaderTypeDef tx_header;
    convert_msg_to_hal(p_msg, &tx_header);

    uint32_t tick_start = HAL_GetTick();
    while (HAL_FDCAN_GetTxFifoFreeLevel(p_handle->p_fdcan) == 0) {
        if ((HAL_GetTick() - tick_start) >= timeout_ms) { return CAN_IFACE_ERROR_TIMEOUT; }
        osDelay(1);
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(p_handle->p_fdcan, &tx_header, (uint8_t*)p_msg->data) !=
        HAL_OK) {
        return CAN_IFACE_ERROR;
    }

    return CAN_IFACE_OK;
}

can_iface_status_t can_iface_receive(
    can_iface_handle_t const* p_handle, can_msg_t* p_msg, uint32_t timeout_ms) {
    if (!is_handle_valid(p_handle) || (NULL == p_msg)) { return CAN_IFACE_ERROR_INVALID; }

    TickType_t ticks_to_wait;
    if (timeout_ms == 0xFFFFFFFFU) {
        ticks_to_wait = portMAX_DELAY;
    } else {
        ticks_to_wait = pdMS_TO_TICKS(timeout_ms);
    }

    BaseType_t result = xQueueReceive(p_handle->rx_queue, p_msg, ticks_to_wait);
    if (result != pdTRUE) { return CAN_IFACE_ERROR_TIMEOUT; }

    return CAN_IFACE_OK;
}

can_iface_status_t can_iface_peek(
    can_iface_handle_t const* p_handle, can_msg_t* p_msg, uint32_t timeout_ms) {
    if (!is_handle_valid(p_handle) || (NULL == p_msg)) { return CAN_IFACE_ERROR_INVALID; }

    TickType_t ticks_to_wait;
    if (timeout_ms == 0xFFFFFFFFU) {
        ticks_to_wait = portMAX_DELAY;
    } else {
        ticks_to_wait = pdMS_TO_TICKS(timeout_ms);
    }

    BaseType_t result = xQueuePeek(p_handle->rx_queue, p_msg, ticks_to_wait);
    if (result != pdTRUE) { return CAN_IFACE_ERROR_TIMEOUT; }

    return CAN_IFACE_OK;
}

uint32_t can_iface_get_queue_count(can_iface_handle_t const* p_handle) {
    if (!is_handle_valid(p_handle)) { return 0; }
    return (uint32_t)uxQueueMessagesWaiting(p_handle->rx_queue);
}

bool can_iface_is_queue_empty(can_iface_handle_t const* p_handle) {
    if (!is_handle_valid(p_handle)) { return true; }
    return (uxQueueMessagesWaiting(p_handle->rx_queue) == 0);
}

static void can_iface_rx_callback(can_iface_handle_t* p_handle, uint32_t RxFifo0ITs) {
    if (!is_handle_valid(p_handle)) { return; }

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;

    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[CAN_IFACE_MAX_DATA_LEN];
    can_msg_t msg;
    BaseType_t higher_prio_task_woken = pdFALSE;

    memset(rx_data, 0, sizeof(rx_data));
    if (HAL_FDCAN_GetRxMessage(p_handle->p_fdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        convert_hal_to_msg(&rx_header, rx_data, &msg);
        if (xQueueSendFromISR(p_handle->rx_queue, &msg, &higher_prio_task_woken) != pdTRUE) {
            can_msg_t tmp;
            (void)xQueueReceiveFromISR(p_handle->rx_queue, &tmp, &higher_prio_task_woken);
            (void)xQueueSendFromISR(p_handle->rx_queue, &msg, &higher_prio_task_woken);
        }

        portYIELD_FROM_ISR(higher_prio_task_woken);
    }
}

static bool is_handle_valid(can_iface_handle_t const* p_handle) {
    return ((NULL != p_handle) && (p_handle->is_init) && (NULL != p_handle->p_fdcan) &&
            (NULL != p_handle->rx_queue));
}

static void convert_hal_to_msg(
    FDCAN_RxHeaderTypeDef const* p_rx_header, uint8_t const* p_data, can_msg_t* p_msg) {

    if (p_rx_header == NULL || p_data == NULL || p_msg == NULL) { return; }

    memset(p_msg, 0, sizeof(can_msg_t));
    p_msg->id = p_rx_header->Identifier;
    p_msg->is_extended = (p_rx_header->IdType == FDCAN_EXTENDED_ID);
    p_msg->is_rtr = (p_rx_header->RxFrameType == FDCAN_REMOTE_FRAME);

    p_msg->len = p_rx_header->DataLength;

    if (!p_msg->is_rtr && p_msg->len > 0) { memcpy(p_msg->data, p_data, p_msg->len); }
}

static void convert_msg_to_hal(can_msg_t const* p_msg, FDCAN_TxHeaderTypeDef* p_tx_header) {
    if (p_msg == NULL || p_tx_header == NULL) return;
    memset(p_tx_header, 0, sizeof(FDCAN_TxHeaderTypeDef));

    p_tx_header->Identifier = p_msg->id;
    p_tx_header->IdType = p_msg->is_extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    p_tx_header->TxFrameType = p_msg->is_rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;

    static const uint32_t dlc_map[9] = {FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6,
        FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8};
    uint32_t len = (p_msg->len > 8) ? 8U : (uint32_t)p_msg->len;
    p_tx_header->DataLength = dlc_map[len];

    p_tx_header->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    p_tx_header->FDFormat = FDCAN_CLASSIC_CAN;
    p_tx_header->BitRateSwitch = FDCAN_BRS_OFF;
    p_tx_header->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    p_tx_header->MessageMarker = 0;
}

static can_iface_status_t configure_default_filter(
    FDCAN_HandleTypeDef* p_fdcan, uint32_t filter_index) {
    FDCAN_FilterTypeDef filter_config;

    filter_config.IdType = FDCAN_STANDARD_ID;
    filter_config.FilterIndex = filter_index;
    filter_config.FilterType = FDCAN_FILTER_RANGE;
    filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter_config.FilterID1 = 0x000;
    filter_config.FilterID2 = 0x7FF;

    if (HAL_FDCAN_ConfigFilter(p_fdcan, &filter_config) != HAL_OK) { return CAN_IFACE_ERROR; }

    filter_config.IdType = FDCAN_EXTENDED_ID;
    filter_config.FilterIndex = filter_index;
    filter_config.FilterType = FDCAN_FILTER_RANGE;
    filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter_config.FilterID1 = 0x00000000;
    filter_config.FilterID2 = 0x1FFFFFFF;

    if (HAL_FDCAN_ConfigFilter(p_fdcan, &filter_config) != HAL_OK) { return CAN_IFACE_ERROR; }

    if (HAL_FDCAN_ConfigGlobalFilter(p_fdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE,
            FDCAN_FILTER_REMOTE) != HAL_OK) {
        return CAN_IFACE_ERROR;
    }

    return CAN_IFACE_OK;
}

static can_iface_status_t configure_filters_from_list(
    FDCAN_HandleTypeDef* p_fdcan, const uint32_t* ids, size_t len, uint32_t start_filter_index) {
    FDCAN_FilterTypeDef filter_config;
    size_t i;
    uint32_t idx = start_filter_index;

    if (ids == NULL || len == 0) return CAN_IFACE_ERROR;

    for (i = 0; i < len; i += 2) {
        uint32_t id1 = ids[i];
        uint32_t id2 = (i + 1 < len) ? ids[i + 1] : ids[i];

        filter_config.IdType = FDCAN_STANDARD_ID;
        filter_config.FilterIndex = idx++;
        filter_config.FilterType = FDCAN_FILTER_DUAL;
        filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        filter_config.FilterID1 = id1;
        filter_config.FilterID2 = id2;

        if (HAL_FDCAN_ConfigFilter(p_fdcan, &filter_config) != HAL_OK) { return CAN_IFACE_ERROR; }
    }

    if (HAL_FDCAN_ConfigGlobalFilter(p_fdcan, FDCAN_REJECT, /* non matching standard IDs */
            FDCAN_REJECT,                                   /* non matching extended IDs */
            FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
        return CAN_IFACE_ERROR;
    }

    return CAN_IFACE_OK;
}

static can_iface_handle_t* find_handle_by_hw(FDCAN_HandleTypeDef* p_fdcan) {
    if (p_fdcan == NULL) return NULL;
    for (uint32_t i = 0; i < CAN_IFACE_MAX_INSTANCES; ++i) {
        if (s_instances[i] && s_instances[i]->p_fdcan == p_fdcan) return s_instances[i];
    }
    return NULL;
}
