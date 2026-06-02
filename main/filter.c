/**
 * RaceChrono CAN Filter Implementation
 */

#include "filter.h"
#include "sdkconfig.h"
#include "tirex_decoder.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "filter";

/* Filter state */
static bool s_allow_all = false;
static filter_rule_t s_rules[MAX_FILTER_RULES];
static uint8_t s_num_rules = 0;

#if !CONFIG_TIREX_FORWARD_ALL_PIDS_OVER_BLE
static bool is_tirex_pid(uint32_t can_id)
{
    return can_id == TIREX_LF_ID ||
           can_id == TIREX_RF_ID ||
           can_id == TIREX_LR_ID ||
           can_id == TIREX_RR_ID;
}
#endif

bool filter_init(void)
{
    ESP_LOGI(TAG, "Initializing filter module");
    
    /* Default: deny all */
    s_allow_all = false;
    s_num_rules = 0;
    memset(s_rules, 0, sizeof(s_rules));
    
    ESP_LOGI(TAG, "Filter initialized (default: deny all)");
#if CONFIG_TIREX_FORWARD_ALL_PIDS_OVER_BLE
    ESP_LOGI(TAG, "Forwarding mode: ALL CAN PIDs");
#else
    ESP_LOGI(TAG, "Forwarding mode: TireX PIDs only unless RaceChrono filter allows more");
#endif
    return true;
}

void filter_reset(void)
{
    s_allow_all = false;
    s_num_rules = 0;
    memset(s_rules, 0, sizeof(s_rules));
    ESP_LOGI(TAG, "Filters reset to deny all");
}

void filter_process(const uint8_t *data, uint16_t len)
{
    if (len < 1) {
        ESP_LOGW(TAG, "Filter command too short");
        return;
    }
    
    uint8_t cmd = data[0];
    
    switch (cmd) {
    case 0: /* Deny All */
        ESP_LOGI(TAG, "Filter: Deny All");
        s_allow_all = false;
        s_num_rules = 0;
        memset(s_rules, 0, sizeof(s_rules));
        break;
        
    case 1: /* Allow All */
        ESP_LOGI(TAG, "Filter: Allow All");
        s_allow_all = true;
        s_num_rules = 0;
        memset(s_rules, 0, sizeof(s_rules));
        break;
        
    case 2: /* Allow Specific PID */
        if (len < 7) {
            ESP_LOGW(TAG, "Filter command 2 too short (need 7 bytes, got %d)", len);
            return;
        }
        
        /* Parse interval (bytes 1-2, little-endian) */
        uint16_t interval = (data[1] | (data[2] << 8));
        
        /* Parse CAN ID (bytes 3-6, little-endian) */
        uint32_t can_id = (data[3] | (data[4] << 8) | (data[5] << 16) | (data[6] << 24));
        
        ESP_LOGI(TAG, "Filter: Allow PID 0x%08X, interval=%d ms", can_id, interval);
        
        /* Check if rule already exists */
        bool found = false;
        for (uint8_t i = 0; i < s_num_rules; i++) {
            if (s_rules[i].pid == can_id) {
                s_rules[i].interval_ms = interval;
                found = true;
                break;
            }
        }
        
        /* Add new rule if not found and space available */
        if (!found && s_num_rules < MAX_FILTER_RULES) {
            s_rules[s_num_rules].pid = can_id;
            s_rules[s_num_rules].interval_ms = interval;
            s_rules[s_num_rules].last_sent_ms = 0;
            s_num_rules++;
        } else if (!found) {
            ESP_LOGW(TAG, "Filter table full, cannot add rule for 0x%08X", can_id);
        }
        break;
        
    default:
        ESP_LOGW(TAG, "Unknown filter command: %d", cmd);
        break;
    }
}

bool filter_should_forward(uint32_t can_id)
{
#if CONFIG_TIREX_FORWARD_ALL_PIDS_OVER_BLE
    (void)can_id;
    return true;
#else
    /* Always forward TireX frames when raw passthrough is not enabled. */
    if (is_tirex_pid(can_id)) {
        return true;
    }

    /* If allow all, forward everything */
    if (s_allow_all) {
        return true;
    }
    
    /* Check against filter rules */
    for (uint8_t i = 0; i < s_num_rules; i++) {
        if (s_rules[i].pid == can_id) {
            /* Check interval */
            uint32_t now = esp_timer_get_time() / 1000; /* ms */
            
            if (s_rules[i].last_sent_ms == 0 || 
                (now - s_rules[i].last_sent_ms) >= s_rules[i].interval_ms) {
                s_rules[i].last_sent_ms = now;
                return true;
            }
            return false; /* Interval not elapsed */
        }
    }
    
    /* Not in filter list, deny */
    return false;
#endif
}
