/**
 * RaceChrono DIY BLE Service
 *
 * Implements the official RaceChrono CAN-over-BLE protocol:
 * - Service UUID: 0x1FF8
 * - Characteristic 0x0001: CAN Notify (Notify)
 * - Characteristic 0x0002: CAN Filter (Write)
 * - Characteristic 0x0003: Telemetry Notify (Notify) — unified telemetry snapshot
 *
 * MTU negotiation: requests 247-byte MTU on connection to accommodate
 * telemetry snapshots (24 bytes payload + ATT header).
 * Connection parameters: requests 15-20ms interval for low-latency telemetry.
 */

#ifndef BLE_SERVICE_H
#define BLE_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

/* RaceChrono BLE service constants */
#define RC_SVC_UUID16          0x1FF8
#define RC_CAN_NOTIFY_UUID16   0x0001
#define RC_CAN_FILTER_UUID16   0x0002
#define RC_TELEMETRY_UUID16    0x0003

/* Maximum BLE notification payload (MTU 247 - 3 byte ATT header) */
#define BLE_MAX_NOTIFY_PAYLOAD  244

/**
 * Initialize the RaceChrono BLE service
 *
 * Sets up GATT server, advertising, and characteristics
 *
 * @return true on success, false on failure
 */
bool ble_service_init(void);

/**
 * Stop BLE advertising and shut down service
 */
void ble_service_stop(void);

/**
 * Send a CAN frame to connected RaceChrono device
 *
 * @param can_id 32-bit CAN identifier (extended format)
 * @param data Pointer to payload data
 * @param len Length of payload (max 8 bytes)
 * @return true if sent successfully, false if no connection or error
 */
bool ble_send_can_frame(uint32_t can_id, const uint8_t *data, uint8_t len);

/**
 * Send a telemetry snapshot to connected RaceChrono device
 *
 * Encodes vehicle dynamics and tire temps as a structured notification.
 * Payload format (24 bytes):
 *   [0-3]   timestamp_ms (LE uint32)
 *   [4-5]   vehicle_speed_kmh (LE uint16, ×10)
 *   [6-7]   engine_rpm (LE uint16, ×10)
 *   [8]     throttle_pct (uint8, 0-100)
 *   [9]     brake_pressure_bar (uint8, ×10)
 *   [10-11] steering_angle_deg (LE int16, ×10)
 *   [12]    gear (int8, offset +128)
 *   [13-16] avg_temp_lf, rf, lr, rr (uint8, °C)
 *   [17]    max_temp (uint8, °C)
 *   [18-21] delta_temp_lf, rf, lr, rr (uint8, °C)
 *   [22-23] tirex/exlap update counts (LE uint16 each, capped)
 *
 * @param data Pointer to telemetry data buffer
 * @param len Length of telemetry data (max BLE_MAX_NOTIFY_PAYLOAD bytes)
 * @return true if sent successfully, false if no connection or error
 */
bool ble_send_telemetry_snapshot(const uint8_t *data, uint8_t len);

/**
 * Check if a RaceChrono device is connected
 * 
 * @return true if connected, false otherwise
 */
bool ble_is_connected(void);

/**
 * Get the current connection handle
 * 
 * @return connection handle, or 0 if not connected
 */
uint16_t ble_get_conn_handle(void);

/**
 * Get the current negotiated ATT MTU
 *
 * @return MTU in bytes (default 23 if not negotiated)
 */
uint16_t ble_get_mtu(void);

#endif /* BLE_SERVICE_H */
