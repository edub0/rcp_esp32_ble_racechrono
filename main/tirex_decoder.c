/**
 * TireX CAN Decoder Implementation
 */

#include "tirex_decoder.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

#if CONFIG_TIREX_DECODER_CONSOLE_LOGGING
static const char *TAG = "tirex_decoder";
#endif

/**
 * Decode a single tire's temperature data
 * 
 * @param data Pointer to 4 bytes of raw temperature data
 * @param temps Pointer to tire_temps_t to fill
 */
static void decode_tire_temps(const uint8_t *data, tire_temps_t *temps)
{
    temps->temp1 = data[0] * 0.5f;
    temps->temp2 = data[1] * 0.5f;
    temps->temp3 = data[2] * 0.5f;
    temps->temp4 = data[3] * 0.5f;
}

bool tirex_decoder_init(void)
{
#if CONFIG_TIREX_DECODER_CONSOLE_LOGGING
    ESP_LOGI(TAG, "TireX decoder initialized");
#endif
    return true;
}

bool tirex_decoder_process(uint32_t can_id, const uint8_t *data, tirex_data_t *tirex)
{
    if (data == NULL || tirex == NULL) {
        return false;
    }
    
    tirex->timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
    
    switch (can_id) {
    case TIREX_LF_ID:
        decode_tire_temps(data, &tirex->lf);
#if CONFIG_TIREX_DECODER_CONSOLE_LOGGING
        ESP_LOGI(TAG, "LF: %.1f %.1f %.1f %.1f", 
                 tirex->lf.temp1, tirex->lf.temp2, 
                 tirex->lf.temp3, tirex->lf.temp4);
#endif
        return true;
        
    case TIREX_RF_ID:
        decode_tire_temps(data, &tirex->rf);
#if CONFIG_TIREX_DECODER_CONSOLE_LOGGING
        ESP_LOGI(TAG, "RF: %.1f %.1f %.1f %.1f", 
                 tirex->rf.temp1, tirex->rf.temp2, 
                 tirex->rf.temp3, tirex->rf.temp4);
#endif
        return true;
        
    case TIREX_LR_ID:
        decode_tire_temps(data, &tirex->lr);
#if CONFIG_TIREX_DECODER_CONSOLE_LOGGING
        ESP_LOGI(TAG, "LR: %.1f %.1f %.1f %.1f", 
                 tirex->lr.temp1, tirex->lr.temp2, 
                 tirex->lr.temp3, tirex->lr.temp4);
#endif
        return true;
        
    case TIREX_RR_ID:
        decode_tire_temps(data, &tirex->rr);
#if CONFIG_TIREX_DECODER_CONSOLE_LOGGING
        ESP_LOGI(TAG, "RR: %.1f %.1f %.1f %.1f", 
                 tirex->rr.temp1, tirex->rr.temp2, 
                 tirex->rr.temp3, tirex->rr.temp4);
#endif
        return true;
        
    default:
        return false;
    }
}

void tirex_decoder_print(const tirex_data_t *tirex)
{
    if (tirex == NULL) {
        return;
    }
    
#if CONFIG_TIREX_DECODER_CONSOLE_LOGGING
    ESP_LOGI(TAG, "=== TireX Data (t=%lu ms) ===", tirex->timestamp_ms);
    ESP_LOGI(TAG, "LF: %.1f %.1f %.1f %.1f", 
             tirex->lf.temp1, tirex->lf.temp2, 
             tirex->lf.temp3, tirex->lf.temp4);
    ESP_LOGI(TAG, "RF: %.1f %.1f %.1f %.1f", 
             tirex->rf.temp1, tirex->rf.temp2, 
             tirex->rf.temp3, tirex->rf.temp4);
    ESP_LOGI(TAG, "LR: %.1f %.1f %.1f %.1f", 
             tirex->lr.temp1, tirex->lr.temp2, 
             tirex->lr.temp3, tirex->lr.temp4);
    ESP_LOGI(TAG, "RR: %.1f %.1f %.1f %.1f", 
             tirex->rr.temp1, tirex->rr.temp2, 
             tirex->rr.temp3, tirex->rr.temp4);
    ESP_LOGI(TAG, "===========================");
#endif
}
