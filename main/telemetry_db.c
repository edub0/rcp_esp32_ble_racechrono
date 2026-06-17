/**
 * Telemetry Database Implementation
 * 
 * Thread-safe central telemetry hub using FreeRTOS mutex.
 */

#include "telemetry_db.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

static const char *TAG = "telemetry_db";

/* Global telemetry state */
static telemetry_t s_telemetry;
static SemaphoreHandle_t s_telemetry_mutex = NULL;

/* Update rate tracking */
static uint32_t s_update_count = 0;
static uint32_t s_update_window_start_ms = 0;
static float s_update_rate_hz = 0.0f;

/**
 * Calculate average and delta for a set of 4 temperature readings
 */
static void calc_tire_stats(const float temps[4], float *avg, float *delta)
{
    float sum = 0.0f;
    float min = temps[0];
    float max = temps[0];
    
    for (int i = 0; i < 4; i++) {
        sum += temps[i];
        if (temps[i] < min) min = temps[i];
        if (temps[i] > max) max = temps[i];
    }
    
    *avg = sum / 4.0f;
    *delta = max - min;
}

bool telemetry_db_init(void)
{
    ESP_LOGI(TAG, "Initializing telemetry database");
    
    /* Create mutex for thread-safe access */
    s_telemetry_mutex = xSemaphoreCreateMutex();
    if (s_telemetry_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create telemetry mutex");
        return false;
    }
    
    /* Reset telemetry data */
    telemetry_db_reset();
    
    ESP_LOGI(TAG, "Telemetry database initialized");
    return true;
}

bool telemetry_db_update_tirex(const float lf[4], const float rf[4],
                               const float lr[4], const float rr[4])
{
    if (lf == NULL || rf == NULL || lr == NULL || rr == NULL) {
        return false;
    }
    
    if (xSemaphoreTake(s_telemetry_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire telemetry mutex");
        return false;
    }
    
    /* Copy temperature data */
    memcpy(s_telemetry.tire_temp_lf, lf, 4 * sizeof(float));
    memcpy(s_telemetry.tire_temp_rf, rf, 4 * sizeof(float));
    memcpy(s_telemetry.tire_temp_lr, lr, 4 * sizeof(float));
    memcpy(s_telemetry.tire_temp_rr, rr, 4 * sizeof(float));
    
    /* Calculate statistics */
    calc_tire_stats(lf, &s_telemetry.avg_temp_lf, &s_telemetry.delta_temp_lf);
    calc_tire_stats(rf, &s_telemetry.avg_temp_rf, &s_telemetry.delta_temp_rf);
    calc_tire_stats(lr, &s_telemetry.avg_temp_lr, &s_telemetry.delta_temp_lr);
    calc_tire_stats(rr, &s_telemetry.avg_temp_rr, &s_telemetry.delta_temp_rr);
    
    /* Find max temperature across all sensors */
    s_telemetry.max_temp = lf[0];
    const float *all_temps[4] = {lf, rf, lr, rr};
    for (int t = 0; t < 4; t++) {
        for (int s = 0; s < 4; s++) {
            if (all_temps[t][s] > s_telemetry.max_temp) {
                s_telemetry.max_temp = all_temps[t][s];
            }
        }
    }
    
    /* Update metadata */
    s_telemetry.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_telemetry.last_tirex_ms = s_telemetry.timestamp_ms;
    s_telemetry.tirex_update_count++;
    s_telemetry.has_tirex_data = true;
    s_update_count++;
    
    xSemaphoreGive(s_telemetry_mutex);
    return true;
}

bool telemetry_db_update_tirex_config(const tirex_sensor_snapshot_t *sensor)
{
    if (sensor == NULL) {
        return false;
    }

    if (xSemaphoreTake(s_telemetry_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire telemetry mutex");
        return false;
    }

    telemetry_tirex_sensor_t *dst = NULL;
    for (uint8_t i = 0; i < s_telemetry.tirex_sensor_count; i++) {
        if (s_telemetry.tirex_sensors[i].base_id == sensor->base_id) {
            dst = &s_telemetry.tirex_sensors[i];
            break;
        }
    }

    if (dst == NULL) {
        if (s_telemetry.tirex_sensor_count >= TIREX_MAX_SENSORS) {
            s_telemetry.tirex_sensor_count = TIREX_MAX_SENSORS - 1;
        }
        dst = &s_telemetry.tirex_sensors[s_telemetry.tirex_sensor_count++];
    }

    memset(dst, 0, sizeof(*dst));
    dst->present = sensor->present;
    dst->base_id = sensor->base_id;
    strncpy(dst->name, sensor->name, TIREX_NAME_LEN);
    dst->name[TIREX_NAME_LEN] = '\0';
    dst->corner = sensor->corner;
    dst->flip_orientation = sensor->flip_orientation;
    dst->full_frame_mode = sensor->full_frame_mode;
    dst->full_frame_trace_enabled = sensor->full_frame_trace_enabled;
    dst->zone_count = sensor->zone_count;
    dst->sample_rate_code = sensor->sample_rate_code;
    dst->observed_position = sensor->observed_position;
    dst->observed_zone_count = sensor->observed_zone_count;
    dst->observed_full_frame_mode = sensor->observed_full_frame_mode;
    dst->first_seen_ms = sensor->first_seen_ms;
    dst->last_seen_ms = sensor->last_seen_ms;
    dst->last_announce_ms = sensor->last_announce_ms;
    dst->last_stats_ms = sensor->last_stats_ms;
    dst->last_config_ms = sensor->last_config_ms;
    dst->last_apply_ms = sensor->last_apply_ms;
    dst->awaiting_config_confirm = sensor->awaiting_config_confirm;
    dst->config_requested = sensor->config_requested;
    dst->has_zone_data = sensor->has_zone_data;
    dst->has_full_frame_data = sensor->has_full_frame_data;
    dst->full_frame_packet_count = sensor->full_frame_packet_count;
    dst->last_full_frame_ms = sensor->last_full_frame_ms;

    s_telemetry.tirex_config_update_count++;
    s_telemetry.last_tirex_config_ms = (uint32_t)(esp_timer_get_time() / 1000);

    xSemaphoreGive(s_telemetry_mutex);
    return true;
}

bool telemetry_db_update_exlap(float speed, float rpm, float throttle,
                               float brake, float steering, float yaw,
                               int8_t gear,
                               float ws_fl, float ws_fr, float ws_rl, float ws_rr,
                               float tire_pressure_fl, float tire_pressure_fr,
                               float tire_pressure_rl, float tire_pressure_rr)
{
    if (xSemaphoreTake(s_telemetry_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire telemetry mutex");
        return false;
    }
    
    /* Update vehicle dynamics */
    s_telemetry.vehicle_speed_kmh = speed;
    s_telemetry.engine_rpm = rpm;
    s_telemetry.throttle_pct = throttle;
    s_telemetry.brake_pressure_bar = brake;
    s_telemetry.steering_angle_deg = steering;
    s_telemetry.yaw_rate = yaw;
    s_telemetry.gear = gear;
    
    /* Update wheel speeds */
    s_telemetry.wheel_speed_fl = ws_fl;
    s_telemetry.wheel_speed_fr = ws_fr;
    s_telemetry.wheel_speed_rl = ws_rl;
    s_telemetry.wheel_speed_rr = ws_rr;

    /* Update tire pressures */
    s_telemetry.tire_pressure_lf = tire_pressure_fl;
    s_telemetry.tire_pressure_rf = tire_pressure_fr;
    s_telemetry.tire_pressure_lr = tire_pressure_rl;
    s_telemetry.tire_pressure_rr = tire_pressure_rr;
    
    /* Update metadata */
    s_telemetry.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_telemetry.last_exlap_ms = s_telemetry.timestamp_ms;
    s_telemetry.exlap_update_count++;
    s_telemetry.has_exlap_data = true;
    s_update_count++;
    
    xSemaphoreGive(s_telemetry_mutex);
    return true;
}

bool telemetry_db_get_snapshot(telemetry_t *telemetry)
{
    if (telemetry == NULL) {
        return false;
    }
    
    if (xSemaphoreTake(s_telemetry_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire telemetry mutex");
        return false;
    }
    
    /* Copy telemetry data */
    memcpy(telemetry, &s_telemetry, sizeof(telemetry_t));
    
    xSemaphoreGive(s_telemetry_mutex);
    return true;
}

bool telemetry_db_to_json(char *json_buf, size_t json_buf_len)
{
    if (json_buf == NULL || json_buf_len == 0) {
        return false;
    }
    
    telemetry_t t;
    if (!telemetry_db_get_snapshot(&t)) {
        return false;
    }
    
    /* Build JSON string */
    int written = snprintf(json_buf, json_buf_len,
        "{"
        "\"timestamp\":%lu,"
        "\"speed\":%.1f,\"rpm\":%.0f,\"throttle\":%.1f,\"brake\":%.2f,"
        "\"steering\":%.1f,\"yaw\":%.2f,\"gear\":%d,"
        "\"ws_fl\":%.1f,\"ws_fr\":%.1f,\"ws_rl\":%.1f,\"ws_rr\":%.1f,"
        "\"tp_fl\":%.2f,\"tp_fr\":%.2f,\"tp_rl\":%.2f,\"tp_rr\":%.2f,"
        "\"temp_lf\":[%.1f,%.1f,%.1f,%.1f],"
        "\"temp_rf\":[%.1f,%.1f,%.1f,%.1f],"
        "\"temp_lr\":[%.1f,%.1f,%.1f,%.1f],"
        "\"temp_rr\":[%.1f,%.1f,%.1f,%.1f],"
        "\"avg_lf\":%.1f,\"avg_rf\":%.1f,\"avg_lr\":%.1f,\"avg_rr\":%.1f,"
        "\"max_temp\":%.1f,"
        "\"delta_lf\":%.1f,\"delta_rf\":%.1f,\"delta_lr\":%.1f,\"delta_rr\":%.1f,"
        "\"tirex_updates\":%lu,\"exlap_updates\":%lu,"
        "\"has_tirex\":%d,\"has_exlap\":%d"
        "}",
        (unsigned long)t.timestamp_ms,
        t.vehicle_speed_kmh, t.engine_rpm, t.throttle_pct, t.brake_pressure_bar,
        t.steering_angle_deg, t.yaw_rate, t.gear,
        t.wheel_speed_fl, t.wheel_speed_fr, t.wheel_speed_rl, t.wheel_speed_rr,
        t.tire_pressure_lf, t.tire_pressure_rf, t.tire_pressure_lr, t.tire_pressure_rr,
        t.tire_temp_lf[0], t.tire_temp_lf[1], t.tire_temp_lf[2], t.tire_temp_lf[3],
        t.tire_temp_rf[0], t.tire_temp_rf[1], t.tire_temp_rf[2], t.tire_temp_rf[3],
        t.tire_temp_lr[0], t.tire_temp_lr[1], t.tire_temp_lr[2], t.tire_temp_lr[3],
        t.tire_temp_rr[0], t.tire_temp_rr[1], t.tire_temp_rr[2], t.tire_temp_rr[3],
        t.avg_temp_lf, t.avg_temp_rf, t.avg_temp_lr, t.avg_temp_rr,
        t.max_temp,
        t.delta_temp_lf, t.delta_temp_rf, t.delta_temp_lr, t.delta_temp_rr,
        (unsigned long)t.tirex_update_count, (unsigned long)t.exlap_update_count,
        t.has_tirex_data ? 1 : 0, t.has_exlap_data ? 1 : 0
    );
    
    if (written < 0 || (size_t)written >= json_buf_len) {
        ESP_LOGW(TAG, "JSON buffer too small (need %d, have %zu)", written, json_buf_len);
        return false;
    }
    
    return true;
}

void telemetry_db_reset(void)
{
    if (xSemaphoreTake(s_telemetry_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire telemetry mutex for reset");
        return;
    }
    
    memset(&s_telemetry, 0, sizeof(telemetry_t));
    s_update_count = 0;
    s_update_window_start_ms = 0;
    s_update_rate_hz = 0.0f;

    xSemaphoreGive(s_telemetry_mutex);
}

float telemetry_db_get_update_rate(void)
{
    float rate;

    if (xSemaphoreTake(s_telemetry_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return 0.0f;
    }

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    
    if (s_update_window_start_ms == 0) {
        s_update_window_start_ms = now_ms;
        rate = 0.0f;
    } else {
        uint32_t elapsed_ms = now_ms - s_update_window_start_ms;
        if (elapsed_ms < 1000) {
            rate = 0.0f;
        } else {
            rate = (float)s_update_count / (elapsed_ms / 1000.0f);
            s_update_count = 0;
            s_update_window_start_ms = now_ms;
        }
    }

    xSemaphoreGive(s_telemetry_mutex);
    return rate;
}
