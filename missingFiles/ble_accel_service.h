#ifndef BLE_ACCEL_SERVICE_H__
#define BLE_ACCEL_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_ble_gatt.h"
#include "nrf_sdh_ble.h"

// Custom Service UUID: 12345678-1234-5678-1234-56789abcdef0
#define BLE_UUID_ACCEL_SERVICE 0x1234
#define BLE_ACCEL_BASE_UUID {0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 0x00, 0x00}

// Characteristic UUID: Accelerometer Data
#define BLE_UUID_ACCEL_DATA_CHAR 0x1235

// Observer priority for the service
#define BLE_ACCEL_BLE_OBSERVER_PRIO 2

// Structure to hold accelerometer measurement
typedef struct
{
    uint16_t sequence_counter; // For packet loss tracking
    int16_t accel_x;          // X-axis
    int16_t accel_y;          // Y-axis
    int16_t accel_z;          // Z-axis
    uint16_t reserved;        // For future use or padding
} ble_accel_meas_t;

// Service structure
typedef struct ble_accel_s
{
    uint16_t service_handle;
    ble_gatts_char_handles_t accel_data_handles;
    uint8_t uuid_type;
    uint16_t conn_handle;
} ble_accel_t;

// Macro for defining an accel service instance
#define BLE_ACCEL_DEF(_name) \
    static ble_accel_t _name; \
    NRF_SDH_BLE_OBSERVER(_name ## _obs, BLE_ACCEL_BLE_OBSERVER_PRIO, ble_accel_on_ble_evt, &_name)

// Function declarations
void ble_accel_service_init(ble_accel_t * p_accel);
uint32_t ble_accel_data_send(ble_accel_t * p_accel, ble_accel_meas_t * p_meas);
void ble_accel_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

#endif // BLE_ACCEL_SERVICE_H__s