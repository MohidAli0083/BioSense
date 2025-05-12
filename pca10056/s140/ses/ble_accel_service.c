#include "ble_accel_service.h"
#include "ble_srv_common.h"
#include "nrf_ble_gatt.h"
#include "nrf_log.h"
#include "app_error.h"


#define BLE_ACCEL_BLE_OBSERVER_PRIO 2

static void on_connect(ble_accel_t * p_accel, ble_evt_t const * p_ble_evt)
{
    p_accel->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

static void on_disconnect(ble_accel_t * p_accel, ble_evt_t const * p_ble_evt)
{
    p_accel->conn_handle = BLE_CONN_HANDLE_INVALID;
}

void ble_accel_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_accel_t * p_accel = (ble_accel_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_accel, p_ble_evt);
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_accel, p_ble_evt);
            break;
        default:
            break;
    }
}

void ble_accel_service_init(ble_accel_t * p_accel)
{
    ret_code_t err_code;
    ble_uuid_t ble_uuid;
    ble_uuid128_t base_uuid = BLE_ACCEL_BASE_UUID;
    ble_add_char_params_t add_char_params;

    p_accel->conn_handle = BLE_CONN_HANDLE_INVALID;

    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_accel->uuid_type);
    APP_ERROR_CHECK(err_code);

    ble_uuid.type = p_accel->uuid_type;
    ble_uuid.uuid = BLE_UUID_ACCEL_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_accel->service_handle);
    APP_ERROR_CHECK(err_code);

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid = BLE_UUID_ACCEL_DATA_CHAR;
    add_char_params.uuid_type = p_accel->uuid_type;
    add_char_params.max_len = 10; // seq (2) + x (2) + y (2) + z (2) + reserved (2)
    add_char_params.init_len = 10;
    add_char_params.is_var_len = false;
    add_char_params.char_props.notify = 1;
    add_char_params.read_access = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    err_code = characteristic_add(p_accel->service_handle, &add_char_params, &p_accel->accel_data_handles);
    APP_ERROR_CHECK(err_code);

    // Log handles for debugging
    NRF_LOG_INFO("Accelerometer Service Handle: %d", p_accel->service_handle);
    NRF_LOG_INFO("Accelerometer Characteristic Value Handle: %d", p_accel->accel_data_handles.value_handle);
    NRF_LOG_INFO("Accelerometer CCCD Handle: %d", p_accel->accel_data_handles.cccd_handle);
}

uint32_t ble_accel_data_send(ble_accel_t * p_accel, ble_accel_meas_t * p_meas)
{
    if (p_accel->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    uint8_t data[10];
    uint16_t len = 0;

    data[len++] = (uint8_t)(p_meas->sequence_counter & 0xFF);
    data[len++] = (uint8_t)(p_meas->sequence_counter >> 8);
    data[len++] = (uint8_t)(p_meas->accel_x & 0xFF);
    data[len++] = (uint8_t)(p_meas->accel_x >> 8);
    data[len++] = (uint8_t)(p_meas->accel_y & 0xFF);
    data[len++] = (uint8_t)(p_meas->accel_y >> 8);
    data[len++] = (uint8_t)(p_meas->accel_z & 0xFF);
    data[len++] = (uint8_t)(p_meas->accel_z >> 8);
    data[len++] = (uint8_t)(p_meas->reserved & 0xFF);
    data[len++] = (uint8_t)(p_meas->reserved >> 8);

    ble_gatts_hvx_params_t hvx_params;
    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_accel->accel_data_handles.value_handle;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len = &len;
    hvx_params.p_data = data;

    NRF_LOG_INFO("Sending notification: handle=%d, len=%d, conn_handle=%d",
                 hvx_params.handle, len, p_accel->conn_handle);

    uint32_t err_code = sd_ble_gatts_hvx(p_accel->conn_handle, &hvx_params);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("sd_ble_gatts_hvx failed: 0x%08X", err_code);
    }

    return err_code;
}