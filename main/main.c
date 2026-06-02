/**
 * TireX → RaceChrono BLE Bridge
 * 
 * Main entry point for ESP32-CAN-X2 firmware
 * 
 * Architecture:
 *   TireX CAN Bus → CAN1/TWAI → CAN RX Task → Filter → BLE TX Queue → RaceChrono
 * 
 * Tasks:
 *   - main_task: Initialize CAN and BLE, start CAN RX task
 *   - can_rx_task: Receive CAN frames, apply filter, forward via BLE
 */

#include "can_driver.h"
#include "ble_service.h"
#include "filter.h"
#include "tirex_decoder.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char *TAG = "main";

/* CAN RX Task Stack Size */
#define CAN_RX_TASK_STACK_SIZE  (4096)
#define CAN_RX_TASK_PRIORITY    (5)

/* Debug: Print TireX data every N frames */
#define TIREX_PRINT_INTERVAL_MS  1000

/* Global TireX data for diagnostics */
static tirex_data_t s_tirex_data;
static uint32_t s_last_print_ms = 0;
static uint32_t s_frame_count = 0;

static void disable_console_logging_if_configured(void)
{
#if CONFIG_TIREX_DISABLE_ALL_CONSOLE_LOGGING
    esp_log_level_set("*", ESP_LOG_NONE);
#endif
}

/**
 * CAN RX Task
 * 
 * Continuously receives CAN frames, applies RaceChrono filter,
 * and forwards matching frames via BLE.
 */
static void can_rx_task(void *pvParameters)
{
    twai_message_t msg;
    
    ESP_LOGI(TAG, "CAN RX task started");
    
    while (true) {
        /* Receive CAN frame with timeout */
        esp_err_t ret = can_driver_receive(&msg, pdMS_TO_TICKS(1000));
        
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "CAN receive error: %s", esp_err_to_name(ret));
            continue;
        }
        
        /* Only process extended frames (29-bit IDs) */
        if (msg.flags & TWAI_MSG_FLAG_EXTD) {
            uint32_t can_id = msg.identifier;
            s_frame_count++;
            
            /* Decode for debug */
            tirex_decoder_process(can_id, msg.data, &s_tirex_data);
            
            /* Print diagnostics periodically */
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            if (now_ms - s_last_print_ms >= TIREX_PRINT_INTERVAL_MS) {
                tirex_decoder_print(&s_tirex_data);
                s_last_print_ms = now_ms;
            }
            
            /* Apply RaceChrono filter */
            if (filter_should_forward(can_id)) {
                /* Forward via BLE */
                if (!ble_send_can_frame(can_id, msg.data, msg.data_length_code)) {
                    ESP_LOGW(TAG, "BLE send failed (not connected?)");
                }
            }
        }
    }
}

/**
 * Application entry point
 */
void app_main(void)
{
    disable_console_logging_if_configured();

#if !CONFIG_TIREX_DISABLE_ALL_CONSOLE_LOGGING
    ESP_LOGI(TAG, "=== TireX → RaceChrono BLE Bridge ===");
    ESP_LOGI(TAG, "Firmware version: 1.0.0");
#endif
    
    /* Initialize NVS - required before BLE/WiFi */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    
    /* Initialize modules */
    filter_init();
    tirex_decoder_init();
    
    /* Initialize CAN driver */
    esp_err_t ret_can = can_driver_init();
    if (ret_can != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CAN driver: %s", esp_err_to_name(ret_can));
        return;
    }
    
    /* Initialize BLE service */
    if (!ble_service_init()) {
#if !CONFIG_TIREX_DISABLE_ALL_CONSOLE_LOGGING
        ESP_LOGE(TAG, "Failed to initialize BLE service");
#endif
        can_driver_stop();
        return;
    }
    
    /* Start CAN RX task */
    ret = xTaskCreate(can_rx_task, "can_rx_task", 
                      CAN_RX_TASK_STACK_SIZE, NULL,
                      CAN_RX_TASK_PRIORITY, NULL);
    if (ret != pdPASS) {
#if !CONFIG_TIREX_DISABLE_ALL_CONSOLE_LOGGING
        ESP_LOGE(TAG, "Failed to create CAN RX task");
#endif
        ble_service_stop();
        can_driver_stop();
        return;
    }
    
#if !CONFIG_TIREX_DISABLE_ALL_CONSOLE_LOGGING
    ESP_LOGI(TAG, "System initialized successfully");
    ESP_LOGI(TAG, "Waiting for RaceChrono connection...");
#endif
    
    /* Main loop - monitor system status */
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        /* Periodic status */
#if !CONFIG_TIREX_DISABLE_ALL_CONSOLE_LOGGING
        ESP_LOGI(TAG, "Status: CAN=%s, BLE=%s, Frames=%lu",
                 can_driver_is_running() ? "OK" : "ERR",
                 ble_is_connected() ? "Connected" : "Advertising",
                 s_frame_count);
#endif
    }
}
