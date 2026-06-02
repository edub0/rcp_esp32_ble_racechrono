/**
 * 
 * RaceChrono DIY BLE Service
 * 
 * Implements the official RaceChrono CAN-over-BLE protocol:
 * - Service UUID: 0x1FF8
 * - Characteristic 0x0001: CAN Notify (Read + Notify)
 * - Characteristic 0x0002: CAN Filter (Write)
 */

#ifndef BLE_SERVICE_H
#define BLE_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

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

#endif /* BLE_SERVICE_H */
