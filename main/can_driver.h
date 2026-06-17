/**
 * CAN Driver for ESP32-CAN-X2
 * 
 * Initializes TWAI controller on CAN1 (GPIO6=RX, GPIO7=TX) at 1 Mbps
 * Uses ESP-IDF v6.0.1 TWAI API (driver/twai.h)
 */

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "driver/twai.h"
#include <stdbool.h>
#include "esp_err.h"

/* CAN Configuration */
#define CAN_TX_PIN    7
#define CAN_RX_PIN    6
#define CAN_BITRATE   1000000  /* 1 Mbps */

/**
 * Initialize the TWAI CAN driver
 * 
 * @return ESP_OK on success, ESP_ERR on failure
 */
esp_err_t can_driver_init(void);

/**
 * Stop the TWAI CAN driver
 * 
 * @return ESP_OK on success, ESP_ERR on failure
 */
esp_err_t can_driver_stop(void);

/**
 * Receive a CAN frame with timeout
 * 
 * @param msg Pointer to TWAI message structure to fill
 * @param ticks_to_wait FreeRTOS ticks to wait (portMAX_DELAY for infinite)
 * @return ESP_OK on success, ESP_ERR on failure
 */
esp_err_t can_driver_receive(twai_message_t *msg, TickType_t ticks_to_wait);

/**
 * Transmit a CAN frame with timeout
 *
 * @param msg Pointer to TWAI message structure to send
 * @param ticks_to_wait FreeRTOS ticks to wait
 * @return ESP_OK on success, ESP_ERR on failure
 */
esp_err_t can_driver_transmit(const twai_message_t *msg, TickType_t ticks_to_wait);

/**
 * Check if CAN driver is initialized and running
 * 
 * @return true if running, false otherwise
 */
bool can_driver_is_running(void);

#endif /* CAN_DRIVER_H */
