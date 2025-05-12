#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"

#include "mpu6050.h"
#include "ble_accel_service.h"

#define DEVICE_NAME                     "Accel_Sensor"                          /**< Name of device. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                   /**< Manufacturer. */
#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (187.5 ms). */
#define APP_ADV_DURATION                18000                                   /**< The advertising duration (180 seconds). */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define ACCEL_MEAS_INTERVAL             APP_TIMER_TICKS(500)                    /**< Accelerometer measurement interval (0.5 seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event to first sd_ble_gap_conn_param_update (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump. */

#define P_TARGET                        -75.0f                                  /**< Target RSSI (dBm). */
#define OFFSET                          3.0f                                    /**< Acceptable RSSI deviation (dB). */
#define FILTER_COEFF                    0.9f                                    /**< Smoothing factor for RSSI averaging. */
#define P_MIN                           -20.0f                                  /**< Minimum transmit power (dBm). */
#define P_MAX                           8.0f                                    /**< Maximum transmit power (dBm). */
#define TPC_INTERVAL                    APP_TIMER_TICKS(500)                    /**< TPC update interval (0.5 seconds). */
#define SLEEP_ACCEL_MEAS_INTERVAL       APP_TIMER_TICKS(60000) /**< Accelerometer measurement interval in sleep mode (60 seconds). */
static bool is_sleep_mode = false;                            /**< Flag to track sleep mode state. */
NRF_BLE_GATT_DEF(m_gatt);                                              /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                /**< Context for the Queued Write module. */
BLE_ADVERTISING_DEF(m_advertising);                                    /**< Advertising module instance. */
BLE_ACCEL_DEF(m_accel);                                                /**< Accelerometer Service instance. */
APP_TIMER_DEF(m_accel_timer_id);                                       /**< Accelerometer measurement timer. */
APP_TIMER_DEF(m_tpc_timer_id);                                         /**< TPC update timer. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;               /**< Handle of the current connection. */
static uint16_t m_sequence_counter = 0;                                /**< Sequence counter for packet loss tracking. */
static float p_received_average = P_TARGET;                            /**< Smoothed RSSI (dBm). */
static float Pt = 0.0f;                                                /**< Current transmit power (dBm). */
static bool tpc_enabled = false;                                       /**< Flag to enable TPC after connection. */
static const int8_t SUPPORTED_TX_POWER[] = {-40, -20, -16, -12, -8, -4, 0, 4, 8}; /**< Supported TX power levels (dBm). */

static ble_uuid_t m_adv_uuids[] =                                      /**< Universally unique service identifiers. */
{
    {BLE_UUID_ACCEL_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN}
};

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void delete_bonds(void)
{
    ret_code_t err_code;
    NRF_LOG_INFO("Erase bonds!");
    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}

static void advertising_start(bool erase_bonds)
{
    if (erase_bonds)
    {
        delete_bonds();
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}

static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);
    if (p_evt->evt_id == PM_EVT_PEERS_DELETE_SUCCEEDED)
    {
        advertising_start(false);
    }
}

static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    switch (p_evt->evt_id)
    {
        case NRF_BLE_GATT_EVT_ATT_MTU_UPDATED:
            NRF_LOG_INFO("MTU updated. Effective MTU: %d", p_evt->params.att_mtu_effective);
            break;
        default:
            NRF_LOG_DEBUG("Unhandled GATT event: %d", p_evt->evt_id);
            break;
    }
}

static int8_t snap_to_supported_power(float power)
{
    int8_t closest = SUPPORTED_TX_POWER[0];
    float min_diff = fabs(power - closest);
    
    for (uint32_t i = 1; i < sizeof(SUPPORTED_TX_POWER) / sizeof(SUPPORTED_TX_POWER[0]); i++)
    {
        float diff = fabs(power - SUPPORTED_TX_POWER[i]);
        if (diff < min_diff)
        {
            min_diff = diff;
            closest = SUPPORTED_TX_POWER[i];
        }
    }
    
    return closest;
}

static void tpc_timeout_handler(void * p_context)
{
    ret_code_t err_code;
    int8_t rssi;
    uint8_t ch_index;
    float p_current;

    UNUSED_PARAMETER(p_context);

    if (m_conn_handle == BLE_CONN_HANDLE_INVALID || !tpc_enabled)
    {
        NRF_LOG_INFO("TPC: No valid connection or TPC disabled, skipping.");
        return;
    }

    // Get RSSI
    err_code = sd_ble_gap_rssi_get(m_conn_handle, &rssi, &ch_index);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("TPC: Failed to get RSSI: 0x%08X", err_code);
        return;
    }

    p_current = (float)rssi; // Current RSSI in dBm

    // Smooth RSSI using exponential moving average
    p_received_average = (1.0f - FILTER_COEFF) * p_received_average + FILTER_COEFF * p_current;
    
    // Log smoothed RSSI for debugging
    NRF_LOG_INFO("TPC: Raw RSSI: %d dBm, Smoothed RSSI: %d dBm/10", rssi, (int)(p_received_average * 10));

    // Determine power adjustment
    float p_delta = 0.0f;
    if (p_received_average > P_TARGET + OFFSET)
    {
        p_delta = -0.5f; // Decrease power
        NRF_LOG_INFO("TPC: RSSI too high, reducing power by %.1f dB", p_delta);
    }
    else if (p_received_average < P_TARGET - OFFSET)
    {
        p_delta = 1.0f; // Increase power
        NRF_LOG_INFO("TPC: RSSI too low, increasing power by %.1f dB", p_delta);
    }

    // Update transmit power
    Pt += p_delta;
    Pt = MAX(MIN(Pt, P_MAX), P_MIN);

    // Snap to nearest supported power level
    int8_t new_power = snap_to_supported_power(Pt);

    // Log power calculations
    NRF_LOG_INFO("TPC: Requested TX Power: %d dBm/10, Snapped TX Power: %d dBm", (int)(Pt * 10), new_power);

    // Set new transmit power
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, new_power);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("TPC: Failed to set TX power: 0x%08X", err_code);
    }
}

static void accel_meas_timeout_handler(void * p_context)
{
    ret_code_t err_code;
    int16_t acc_x, acc_y, acc_z;
    bool acc_ok;
    ble_accel_meas_t meas;

    UNUSED_PARAMETER(p_context);

    if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        NRF_LOG_INFO("No valid connection, skipping notification.");
        return;
    }

    ble_gatts_value_t cccd_value = {0};
    uint16_t cccd_data = 0;
    cccd_value.p_value = (uint8_t *)&cccd_data;
    cccd_value.len = sizeof(cccd_data);
    err_code = sd_ble_gatts_value_get(m_conn_handle, m_accel.accel_data_handles.cccd_handle, &cccd_value);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to read CCCD: 0x%08X", err_code);
        NRF_LOG_WARNING("CCCD read failed, assuming notifications disabled.");
        return;
    }
    if ((cccd_data & BLE_GATT_HVX_NOTIFICATION) == 0)
    {
        NRF_LOG_INFO("Notifications not enabled by client, skipping notification.");
        return;
    }

    acc_ok = MPU6050_ReadAcc(&acc_x, &acc_y, &acc_z);
    if (acc_ok)
    {
        NRF_LOG_INFO("Accel X: %d, Y: %d, Z: %d, Seq: %d", acc_x, acc_y, acc_z, m_sequence_counter);
        meas.sequence_counter = m_sequence_counter;
        meas.accel_x = acc_x;
        meas.accel_y = acc_y;
        meas.accel_z = acc_z;
        meas.reserved = 0;

        err_code = ble_accel_data_send(&m_accel, &meas);
        if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE &&
            err_code != NRF_ERROR_RESOURCES && err_code != NRF_ERROR_BUSY &&
            err_code != NRF_ERROR_DATA_SIZE)
        {
            NRF_LOG_ERROR("ble_accel_data_send failed: 0x%08X", err_code);
            APP_ERROR_CHECK(err_code);
        }
        else
        {
            if (err_code == NRF_ERROR_DATA_SIZE)
            {
                NRF_LOG_WARNING("Notification failed: Data size too large for MTU");
            }
            else
            {
                NRF_LOG_INFO("Notification sent, Seq: %d", m_sequence_counter);
            }
        }

        m_sequence_counter++;
    }
    else
    {
        NRF_LOG_ERROR("Failed to read MPU6050 data");
        meas.sequence_counter = m_sequence_counter;
        meas.accel_x = 0;
        meas.accel_y = 0;
        meas.accel_z = 0;
        meas.reserved = 0;

        err_code = ble_accel_data_send(&m_accel, &meas);
        if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE &&
            err_code != NRF_ERROR_RESOURCES && err_code != NRF_ERROR_BUSY &&
            err_code != NRF_ERROR_DATA_SIZE)
        {
            NRF_LOG_ERROR("ble_accel_data_send failed: 0x%08X", err_code);
            APP_ERROR_CHECK(err_code);
        }
        else
        {
            if (err_code == NRF_ERROR_DATA_SIZE)
            {
                NRF_LOG_WARNING("Error notification failed: Data size too large for MTU");
            }
            else
            {
                NRF_LOG_INFO("Error notification sent, Seq: %d", m_sequence_counter);
            }
        }

        m_sequence_counter++;
    }
}

static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_accel_timer_id, APP_TIMER_MODE_REPEATED, accel_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_tpc_timer_id, APP_TIMER_MODE_REPEATED, tpc_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

static void gap_params_init(void)
{
    ret_code_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void gatt_init(void)
{
    ret_code_t err_code;
    
    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, 65);
    APP_ERROR_CHECK(err_code);
    
    NRF_LOG_INFO("GATT MTU initialized. Desired MTU: %d", 65);
}

static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void services_init(void)
{
    ret_code_t err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    qwr_init.error_handler = nrf_qwr_error_handler;
    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    ble_accel_service_init(&m_accel);
}

static void application_timers_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_start(m_accel_timer_id, ACCEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void conn_params_init(void)
{
    ret_code_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));
    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = false;
    cp_init.evt_handler = on_conn_params_evt;
    cp_init.error_handler = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

static void sleep_mode_enter(void)
{
    ret_code_t err_code;
    
    // Set BSP indication to idle
    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);
    
    // Stop advertising if not connected
    if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        err_code = sd_ble_gap_adv_stop(m_advertising.adv_handle);
        if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
        {
            NRF_LOG_ERROR("Failed to stop advertising: 0x%08X", err_code);
        }
        else
        {
            NRF_LOG_INFO("Advertising stopped for sleep mode");
        }
    }
    
    // Stop the accelerometer timer
    err_code = app_timer_stop(m_accel_timer_id);
    APP_ERROR_CHECK(err_code);
    
    // Restart the accelerometer timer with 60-second interval
    err_code = app_timer_start(m_accel_timer_id, SLEEP_ACCEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    
    // Set sleep mode flag
    is_sleep_mode = true;
    NRF_LOG_INFO("Entered sleep mode: sending accelerometer data every 60 seconds");
    NRF_LOG_FLUSH();
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected. Conn Handle: %d, Reason: 0x%02X",
                         p_ble_evt->evt.gap_evt.conn_handle,
                         p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            m_sequence_counter = 0;
            err_code = app_timer_stop(m_accel_timer_id);
            APP_ERROR_CHECK(err_code);
            err_code = app_timer_stop(m_tpc_timer_id);
            APP_ERROR_CHECK(err_code);
            err_code = sd_ble_gap_rssi_stop(m_conn_handle);
            if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
            {
                NRF_LOG_ERROR("Failed to stop RSSI measurement: 0x%08X", err_code);
            }
            tpc_enabled = false;
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected. Conn Handle: %d", p_ble_evt->evt.gap_evt.conn_handle);
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            m_sequence_counter = 0;
            NRF_LOG_INFO("Accelerometer CCCD Handle: %d", m_accel.accel_data_handles.cccd_handle);
            // Start RSSI measurement
            err_code = sd_ble_gap_rssi_start(m_conn_handle, 0, 0);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("Failed to start RSSI measurement: 0x%08X", err_code);
            }
            tpc_enabled = false; // Will be enabled on CCCD write
            break;

        case BLE_GATTS_EVT_WRITE:
            NRF_LOG_INFO("GATT Write. Handle: %d, Len: %d, Data: 0x%02X%02X",
                         p_ble_evt->evt.gatts_evt.params.write.handle,
                         p_ble_evt->evt.gatts_evt.params.write.len,
                         p_ble_evt->evt.gatts_evt.params.write.len >= 2 ? p_ble_evt->evt.gatts_evt.params.write.data[1] : 0,
                         p_ble_evt->evt.gatts_evt.params.write.len >= 1 ? p_ble_evt->evt.gatts_evt.params.write.data[0] : 0);
            if (p_ble_evt->evt.gatts_evt.params.write.handle == m_accel.accel_data_handles.cccd_handle &&
                p_ble_evt->evt.gatts_evt.params.write.len >= 2 &&
                p_ble_evt->evt.gatts_evt.params.write.data[0] == 0x01)
            {
                NRF_LOG_INFO("Notifications enabled by client.");
                err_code = app_timer_start(m_accel_timer_id, ACCEL_MEAS_INTERVAL, NULL);
                APP_ERROR_CHECK(err_code);
                err_code = app_timer_start(m_tpc_timer_id, TPC_INTERVAL, NULL);
                APP_ERROR_CHECK(err_code);
                tpc_enabled = true;
            }
            else
            {
                NRF_LOG_INFO("CCCD write mismatch. Expected handle: %d, Data: 0x0001",
                             m_accel.accel_data_handles.cccd_handle);
            }
            break;

        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            NRF_LOG_INFO("MTU Exchange Request. Client MTU: %d",
                         p_ble_evt->evt.gatts_evt.params.exchange_mtu_request.client_rx_mtu);
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle, 65);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("MTU exchange reply failed: 0x%08X", err_code);
                if (err_code == NRF_ERROR_INVALID_STATE)
                {
                    NRF_LOG_WARNING("Invalid state for MTU exchange, using default MTU.");
                }
                else
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys = {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            NRF_LOG_DEBUG("Unhandled BLE Event: 0x%04X", p_ble_evt->header.evt_id);
            break;
    }
}
static void wake_mode_enter(void)
{
    ret_code_t err_code;
    
    // Set BSP indication to idle (or advertising if not connected)
    err_code = bsp_indication_set(m_conn_handle == BLE_CONN_HANDLE_INVALID ? BSP_INDICATE_ADVERTISING : BSP_INDICATE_CONNECTED);
    APP_ERROR_CHECK(err_code);
    
    // Restart advertising if not connected
    if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
        {
            NRF_LOG_ERROR("Failed to restart advertising: 0x%08X", err_code);
        }
        else
        {
            NRF_LOG_INFO("Advertising restarted");
        }
    }
    
    // Stop the accelerometer timer
    err_code = app_timer_stop(m_accel_timer_id);
    APP_ERROR_CHECK(err_code);
    
    // Restart the accelerometer timer with normal 500ms interval
    err_code = app_timer_start(m_accel_timer_id, ACCEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    
    // Clear sleep mode flag
    is_sleep_mode = false;
    NRF_LOG_INFO("Exited sleep mode: resumed normal operation (500ms intervals)");
    NRF_LOG_FLUSH();
}
static void ble_stack_init(void)
{
    ret_code_t err_code;
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;
        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;
        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;
        case BSP_EVENT_KEY_0: // Button 1 triggers sleep mode
            NRF_LOG_INFO("Button 1 pressed, entering sleep mode.");
            sleep_mode_enter();
            break;
        case BSP_EVENT_KEY_1: // Button 2 exits sleep mode
            if (is_sleep_mode)
            {
                NRF_LOG_INFO("Button 2 pressed, exiting sleep mode.");
                wake_mode_enter();
            }
            else
            {
                NRF_LOG_INFO("Button 2 pressed, not in sleep mode.");
            }
            break;
        default:
            break;
    }
}

static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);
    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));
    sec_param.bond = SEC_PARAM_BOND;
    sec_param.mitm = SEC_PARAM_MITM;
    sec_param.lesc = SEC_PARAM_LESC;
    sec_param.keypress = SEC_PARAM_KEYPRESS;
    sec_param.io_caps = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob = SEC_PARAM_OOB;
    sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc = 1;
    sec_param.kdist_own.id = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id = 1;
    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);
    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

static void advertising_init(void)
{
    ret_code_t err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));
    init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = true;
    init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids = m_adv_uuids;
    init.config.ble_adv_fast_enabled = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);
    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;
    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

int main(void)
{
    bool erase_bonds;

    // Initialize logging
    log_init();
    
    // Log to confirm application start
    NRF_LOG_INFO("Device started");
    
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    peer_manager_init();

    // Initialize TPC
    p_received_average = P_TARGET;
    Pt = 0.0f; // Start with 0 dBm

    twi_master_init();
    if (!mpu6050_init())
    {
        NRF_LOG_INFO("MPU6050 initialization failed!");
    }
    else
    {
        NRF_LOG_INFO("MPU6050 initialized successfully.");
    }

    NRF_LOG_INFO("Accel X, Accel Y, Accel Z");
    int16_t acc_x = 0, acc_y = 0, acc_z = 0;
    bool acc_ok = MPU6050_ReadAcc(&acc_x, &acc_y, &acc_z);
    if (acc_ok)
    {
        NRF_LOG_INFO("%d, %d, %d", acc_x, acc_y, acc_z);
    }
    else
    {
        NRF_LOG_ERROR("Read failed - Accel: %d", acc_ok);
    }

    NRF_LOG_FLUSH();
    nrf_delay_ms(1000);

    NRF_LOG_INFO("Accelerometer example started.");
    advertising_start(erase_bonds);

    for (;;)
    {
        if (is_sleep_mode && m_conn_handle == BLE_CONN_HANDLE_INVALID)
        {
            // In sleep mode and disconnected, idle in low-power state
            nrf_pwr_mgmt_run();
        }
        else
        {
            idle_state_handle();
        }
    }
}