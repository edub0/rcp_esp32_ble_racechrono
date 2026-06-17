/**
 * CAN Driver Implementation for ESP32-CAN-X2
 * 
 * Uses ESP32-S3 native TWAI controller on CAN1
 * - TX: GPIO7
 * - RX: GPIO6
 * - Bitrate: 1 Mbps
 * - Mode: Normal (accept all frames initially)
 * 
 * ESP-IDF v6.0.1 TWAI API (driver/twai.h)
 */

#include "can_driver.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "can_driver";
static bool s_can_running = false;

esp_err_t can_driver_init(void)
{
    ESP_LOGI(TAG, "Initializing TWAI CAN driver");
    ESP_LOGI(TAG, "TX GPIO=%d, RX GPIO=%d, Bitrate=%d", CAN_TX_PIN, CAN_RX_PIN, CAN_BITRATE);

    /* General configuration */
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);

    /* Timing configuration for 1 Mbps */
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();

    /* Filter configuration - accept all frames initially */
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    /* Install TWAI driver */
    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Start TWAI driver */
    ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(ret));
        twai_driver_uninstall();
        return ret;
    }

    s_can_running = true;
    ESP_LOGI(TAG, "TWAI CAN driver initialized successfully");
    return ESP_OK;
}

esp_err_t can_driver_stop(void)
{
    if (!s_can_running) {
        return ESP_OK;
    }

    esp_err_t ret = twai_stop();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop TWAI driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = twai_driver_uninstall();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete TWAI driver: %s", esp_err_to_name(ret));
        return ret;
    }

    s_can_running = false;
    ESP_LOGI(TAG, "TWAI CAN driver stopped");
    return ESP_OK;
}

esp_err_t can_driver_receive(twai_message_t *msg, TickType_t ticks_to_wait)
{
    if (!s_can_running) {
        ESP_LOGW(TAG, "CAN driver not running");
        return ESP_ERR_INVALID_STATE;
    }

    return twai_receive(msg, ticks_to_wait);
}

esp_err_t can_driver_transmit(const twai_message_t *msg, TickType_t ticks_to_wait)
{
    if (!s_can_running) {
        ESP_LOGW(TAG, "CAN driver not running");
        return ESP_ERR_INVALID_STATE;
    }

    return twai_transmit(msg, ticks_to_wait);
}

bool can_driver_is_running(void)
{
    return s_can_running;
}
