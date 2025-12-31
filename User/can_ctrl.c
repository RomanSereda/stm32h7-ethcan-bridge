#include "can_ctrl.h"

#define CAN_IFACE_TIMEOUT pdMS_TO_TICKS(100U)

can_ctrl_status_t can_ctrl_init(
    can_ctrl_handle_t* p_handle, FDCAN_HandleTypeDef* p_fdcan, const uint32_t* ids, uint32_t ids_len) {
    if (p_handle == NULL || p_fdcan == NULL) return CAN_CTRL_ERROR_INVALID;

    uint32_t data_ids[CAN_CTRL_MAX_IDS_LEN] = {0};
    for (size_t i = 0; i < ids_len; i++) {
        data_ids[i] = ids[i] + CAN_CTRL_DATA_OFFSET;
    }

    can_filter_config_t filter_config = {
        .len = ids_len,
        .ids = data_ids
    };

    memset(p_handle, 0, sizeof(*p_handle));
    if (can_iface_init(&p_handle->p_iface, p_fdcan, &filter_config) != CAN_IFACE_OK) {
        return CAN_CTRL_ERROR_NOT_INIT;
    }

    memcpy(p_handle->ids, ids, ids_len * sizeof(uint32_t));
    p_handle->ids_len = ids_len;
    p_handle->is_init = true;

    return CAN_CTRL_OK;
}

can_ctrl_status_t can_ctrl_deinit(can_ctrl_handle_t* p_handle) {
    if (p_handle == NULL) return CAN_CTRL_ERROR_INVALID;
    can_iface_deinit(&p_handle->p_iface);
    p_handle->is_init = false;
    return CAN_CTRL_OK;
}

can_ctrl_status_t can_ctrl_request_sync(
    can_ctrl_handle_t* p_handle, uint32_t index, bool is_extended, can_ctrl_resp_t* out_resp) {
    if (p_handle == NULL || out_resp == NULL) return CAN_CTRL_ERROR_INVALID;
    if (!p_handle->is_init) return CAN_CTRL_ERROR_NOT_INIT;

    out_resp->status = CAN_CTRL_ERROR;

    const TickType_t timeout_ticks = pdMS_TO_TICKS(CAN_CTRL_DEFAULT_TIMEOUT_MS);
    const TickType_t start_tick = xTaskGetTickCount();

    can_msg_t req;
    memset(&req, 0, sizeof(req));
    req.id = p_handle->ids[index];
    req.is_extended = is_extended;
    req.is_rtr = true;

    if (can_iface_transmit(&p_handle->p_iface, &req, CAN_IFACE_TIMEOUT) != CAN_IFACE_OK) {
        return CAN_CTRL_ERROR;
    }

    bool has_data = false;
    TickType_t duration = xTaskGetTickCount() - start_tick;
    while ((duration < timeout_ticks) && !has_data) {
        can_msg_t current_msg;
        bool found = false;

        for (;;) {
            const uint32_t timeout = found ? 0 : CAN_IFACE_TIMEOUT;
            if (can_iface_receive(&p_handle->p_iface, &current_msg, timeout) == CAN_IFACE_OK) {
                if (current_msg.id == (p_handle->ids[index] + CAN_CTRL_DATA_OFFSET) && !current_msg.is_rtr) {
                    memcpy(&out_resp->msg, &current_msg, sizeof(can_msg_t));
                    found = true;
                    continue;
                }
            } else {
                break;
            }
        }

        has_data = found;
        duration = xTaskGetTickCount() - start_tick;
    }

    if (has_data) {
        out_resp->status = CAN_CTRL_OK;
        return CAN_CTRL_OK;
    }

    return CAN_CTRL_ERROR_TIMEOUT;
}