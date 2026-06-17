/**
 * EXLAP Parser Implementation
 *
 * Lightweight parser for EXLAP `Dat` telemetry messages.
 * It keeps a small streaming buffer so fragmented TCP reads can be
 * reassembled without dynamic allocation.
 */

#include "exlap_parser.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

static const char *TAG = "exlap_parser";

#define EXLAP_MAX_TAG_LEN  32
#define EXLAP_MAX_VAL_LEN   64
#define EXLAP_PARSE_BUF_SIZE 1024

static char s_parse_buf[EXLAP_PARSE_BUF_SIZE];
static size_t s_parse_buf_len = 0;
static uint32_t s_packet_count = 0;

#if CONFIG_EXLAP_PARSER_DEBUG_LOGGING
#define EXLAP_DBG(...) ESP_LOGD(TAG, __VA_ARGS__)
#else
#define EXLAP_DBG(...) ((void)0)
#endif

static const char *find_str_in_range(const char *buf, size_t buf_len,
                                     const char *needle)
{
    size_t needle_len = strlen(needle);
    if (needle_len == 0 || needle_len > buf_len) {
        return NULL;
    }

    for (size_t i = 0; i <= buf_len - needle_len; i++) {
        if (memcmp(&buf[i], needle, needle_len) == 0) {
            return &buf[i];
        }
    }

    return NULL;
}

static const char *find_char_in_range(const char *buf, size_t buf_len, char needle)
{
    for (size_t i = 0; i < buf_len; i++) {
        if (buf[i] == needle) {
            return &buf[i];
        }
    }

    return NULL;
}

static size_t suffix_prefix_overlap(const char *buf, size_t buf_len, const char *pattern)
{
    size_t pattern_len = strlen(pattern);
    size_t max_overlap = (buf_len < pattern_len) ? buf_len : pattern_len;

    for (size_t overlap = max_overlap; overlap > 0; overlap--) {
        if (memcmp(&buf[buf_len - overlap], pattern, overlap) == 0) {
            return overlap;
        }
    }

    return 0;
}

static bool extract_attr_value_in_range(const char *buf, size_t buf_len,
                                        const char *attr_name,
                                        char *value_out, size_t value_out_len)
{
    char pattern[EXLAP_MAX_TAG_LEN + 4];
    int written = snprintf(pattern, sizeof(pattern), "%s=\"", attr_name);
    if (written < 0 || (size_t)written >= sizeof(pattern)) {
        return false;
    }

    const char *start = find_str_in_range(buf, buf_len, pattern);
    if (start == NULL) {
        return false;
    }

    start += strlen(pattern);
    size_t remaining = buf_len - (size_t)(start - buf);
    const char *end = find_char_in_range(start, remaining, '"');
    if (end == NULL) {
        return false;
    }

    size_t copy_len = (size_t)(end - start);
    if (copy_len >= value_out_len) {
        copy_len = value_out_len - 1;
    }

    memcpy(value_out, start, copy_len);
    value_out[copy_len] = '\0';
    return true;
}

static bool extract_named_field_value(const char *body, size_t body_len,
                                      const char *field_name,
                                      char *value_out, size_t value_out_len)
{
    char pattern[EXLAP_MAX_TAG_LEN + 8];
    int written = snprintf(pattern, sizeof(pattern), "name=\"%s\"", field_name);
    if (written < 0 || (size_t)written >= sizeof(pattern)) {
        return false;
    }

    const char *name_pos = find_str_in_range(body, body_len, pattern);
    if (name_pos == NULL) {
        return false;
    }

    const char *tag_start = name_pos;
    while (tag_start > body && *tag_start != '<') {
        tag_start--;
    }
    if (*tag_start != '<') {
        return false;
    }

    size_t remaining = body_len - (size_t)(tag_start - body);
    const char *tag_end = find_char_in_range(tag_start, remaining, '>');
    if (tag_end == NULL) {
        return false;
    }

    size_t tag_len = (size_t)(tag_end - tag_start) + 1;
    return extract_attr_value_in_range(tag_start, tag_len, "val", value_out, value_out_len);
}

static bool extract_named_field_float(const char *body, size_t body_len,
                                      const char *field_name, float *value_out)
{
    char value_buf[EXLAP_MAX_VAL_LEN];
    if (!extract_named_field_value(body, body_len, field_name, value_buf, sizeof(value_buf))) {
        return false;
    }

    char *end = NULL;
    float value = strtof(value_buf, &end);
    if (end == value_buf) {
        return false;
    }

    *value_out = value;
    return true;
}

static bool state_allows_update(const char *body, size_t body_len)
{
    char state[EXLAP_MAX_VAL_LEN];
    if (!extract_named_field_value(body, body_len, "state", state, sizeof(state))) {
        return true;
    }

    return (strcasecmp(state, "valid") == 0 ||
            strcasecmp(state, "ok") == 0 ||
            strcasecmp(state, "active") == 0);
}

static void clear_validity_flags(exlap_telemetry_t *telemetry)
{
    telemetry->has_vehicle_speed = false;
    telemetry->has_engine_speed = false;
    telemetry->has_accelerator_pos = false;
    telemetry->has_brake_pressure = false;
    telemetry->has_steering_angle = false;
    telemetry->has_yaw_rate = false;
    telemetry->has_wheel_speed_fl = false;
    telemetry->has_wheel_speed_fr = false;
    telemetry->has_wheel_speed_rl = false;
    telemetry->has_wheel_speed_rr = false;
    telemetry->has_tire_pressure_fl = false;
    telemetry->has_tire_pressure_fr = false;
    telemetry->has_tire_pressure_rl = false;
    telemetry->has_tire_pressure_rr = false;
    telemetry->has_gear = false;
}

static int8_t parse_gear_string(const char *gear_str)
{
    if (gear_str == NULL || gear_str[0] == '\0') {
        return 0;
    }

    if (strcasecmp(gear_str, "NoGear") == 0 ||
        strcasecmp(gear_str, "Neutral") == 0 ||
        strcasecmp(gear_str, "N") == 0) {
        return 0;
    }

    if (strcasecmp(gear_str, "Reverse") == 0 ||
        strcasecmp(gear_str, "R") == 0 ||
        strcasecmp(gear_str, "GearR") == 0) {
        return -1;
    }

    if (strncasecmp(gear_str, "Gear", 4) == 0) {
        const char *digits = gear_str + 4;
        if (*digits != '\0') {
            long gear = strtol(digits, NULL, 10);
            if (gear > 0 && gear < 127) {
                return (int8_t)gear;
            }
        }
    }

    return (int8_t)strtol(gear_str, NULL, 10);
}

static void parse_dat_message(const char *dat_start, size_t dat_len,
                              exlap_telemetry_t *telemetry)
{
    const char *open_end = find_char_in_range(dat_start, dat_len, '>');
    if (open_end == NULL) {
        return;
    }

    size_t open_len = (size_t)(open_end - dat_start) + 1;
    char url[EXLAP_MAX_TAG_LEN];
    if (!extract_attr_value_in_range(dat_start, open_len, "url", url, sizeof(url))) {
        EXLAP_DBG("Dat without url attribute");
        return;
    }

    const char *body_start = open_end + 1;
    size_t body_len = dat_len - open_len;
    const char *close_tag = find_str_in_range(body_start, body_len, "</Dat>");
    if (close_tag == NULL) {
        return;
    }

    body_len = (size_t)(close_tag - body_start);

    EXLAP_DBG("Dat url=%s len=%zu", url, body_len);

    if (strcmp(url, "vehicleSpeed") == 0) {
        float speed = 0.0f;
        if (state_allows_update(body_start, body_len) &&
            extract_named_field_float(body_start, body_len, "speed", &speed)) {
            char unit[EXLAP_MAX_VAL_LEN];
            if (extract_named_field_value(body_start, body_len, "unit", unit, sizeof(unit)) &&
                (strcasecmp(unit, "mph") == 0 || strcasecmp(unit, "mi/h") == 0)) {
                speed *= 1.609344f;
            }

            telemetry->vehicle_speed = speed;
            telemetry->has_vehicle_speed = true;
            EXLAP_DBG("  vehicleSpeed=%.2f km/h", telemetry->vehicle_speed);
        }
        return;
    }

    if (strcmp(url, "engineSpeed") == 0) {
        float rpm = 0.0f;
        if (state_allows_update(body_start, body_len) &&
            extract_named_field_float(body_start, body_len, "engineSpeed", &rpm)) {
            telemetry->engine_speed = rpm;
            telemetry->has_engine_speed = true;
            EXLAP_DBG("  engineSpeed=%.0f", telemetry->engine_speed);
        }
        return;
    }

    if (strcmp(url, "acceleratorPosition") == 0) {
        float throttle = 0.0f;
        if (state_allows_update(body_start, body_len) &&
            extract_named_field_float(body_start, body_len, "acceleratorPosition", &throttle)) {
            if (throttle > -1.5f && throttle < 1.5f) {
                throttle *= 100.0f;
            }
            telemetry->accelerator_pos = throttle;
            telemetry->has_accelerator_pos = true;
            EXLAP_DBG("  acceleratorPosition=%.1f%%", telemetry->accelerator_pos);
        }
        return;
    }

    if (strcmp(url, "brakePressure") == 0) {
        float brake_psi = 0.0f;
        if (state_allows_update(body_start, body_len) &&
            extract_named_field_float(body_start, body_len, "brakePressure", &brake_psi)) {
            telemetry->brake_pressure = brake_psi * 0.0689475729f;
            telemetry->has_brake_pressure = true;
            EXLAP_DBG("  brakePressure=%.3f bar", telemetry->brake_pressure);
        }
        return;
    }

    if (strcmp(url, "wheelAngle") == 0) {
        float wheel_angle = 0.0f;
        if (state_allows_update(body_start, body_len) &&
            extract_named_field_float(body_start, body_len, "wheelAngle", &wheel_angle)) {
            telemetry->steering_angle = wheel_angle;
            telemetry->has_steering_angle = true;
            EXLAP_DBG("  wheelAngle=%.1f deg", telemetry->steering_angle);
        }
        return;
    }

    if (strcmp(url, "yawRate") == 0) {
        float yaw_rate = 0.0f;
        if (state_allows_update(body_start, body_len) &&
            extract_named_field_float(body_start, body_len, "yawRate", &yaw_rate)) {
            telemetry->yaw_rate = yaw_rate;
            telemetry->has_yaw_rate = true;
            EXLAP_DBG("  yawRate=%.2f", telemetry->yaw_rate);
        }
        return;
    }

    if (strcmp(url, "espTyreVelocities") == 0) {
        float wheel_speed = 0.0f;

        if (state_allows_update(body_start, body_len) &&
            extract_named_field_float(body_start, body_len, "frontLeft", &wheel_speed)) {
            telemetry->wheel_speed_fl = wheel_speed;
            telemetry->has_wheel_speed_fl = true;
        }
        if (state_allows_update(body_start, body_len) &&
            extract_named_field_float(body_start, body_len, "frontRight", &wheel_speed)) {
            telemetry->wheel_speed_fr = wheel_speed;
            telemetry->has_wheel_speed_fr = true;
        }
        if (state_allows_update(body_start, body_len) &&
            extract_named_field_float(body_start, body_len, "rearLeft", &wheel_speed)) {
            telemetry->wheel_speed_rl = wheel_speed;
            telemetry->has_wheel_speed_rl = true;
        }
        if (state_allows_update(body_start, body_len) &&
            extract_named_field_float(body_start, body_len, "rearRight", &wheel_speed)) {
            telemetry->wheel_speed_rr = wheel_speed;
            telemetry->has_wheel_speed_rr = true;
        }

        if (telemetry->has_wheel_speed_fl || telemetry->has_wheel_speed_fr ||
            telemetry->has_wheel_speed_rl || telemetry->has_wheel_speed_rr) {
            EXLAP_DBG("  espTyreVelocities parsed");
        }
        return;
    }

    if (strcmp(url, "tyrePressures") == 0) {
        float pressure = 0.0f;
        float pressure_scale = 1.0f;
        char unit[EXLAP_MAX_VAL_LEN];

        if (extract_named_field_value(body_start, body_len, "pressureUnit", unit, sizeof(unit)) &&
            strcasecmp(unit, "psi") == 0) {
            pressure_scale = 0.0689475729f;
        }

        if (state_allows_update(body_start, body_len) &&
            extract_named_field_float(body_start, body_len, "pressureFrontLeft", &pressure)) {
            telemetry->tire_pressure_fl = pressure * pressure_scale;
            telemetry->has_tire_pressure_fl = true;
        }
        if (state_allows_update(body_start, body_len) &&
            extract_named_field_float(body_start, body_len, "pressureFrontRight", &pressure)) {
            telemetry->tire_pressure_fr = pressure * pressure_scale;
            telemetry->has_tire_pressure_fr = true;
        }
        if (state_allows_update(body_start, body_len) &&
            extract_named_field_float(body_start, body_len, "pressureRearLeft", &pressure)) {
            telemetry->tire_pressure_rl = pressure * pressure_scale;
            telemetry->has_tire_pressure_rl = true;
        }
        if (state_allows_update(body_start, body_len) &&
            extract_named_field_float(body_start, body_len, "pressureRearRight", &pressure)) {
            telemetry->tire_pressure_rr = pressure * pressure_scale;
            telemetry->has_tire_pressure_rr = true;
        }

        if (telemetry->has_tire_pressure_fl || telemetry->has_tire_pressure_fr ||
            telemetry->has_tire_pressure_rl || telemetry->has_tire_pressure_rr) {
            EXLAP_DBG("  tyrePressures parsed");
        }
        return;
    }

    if (strcmp(url, "currentGear") == 0) {
        char gear_str[EXLAP_MAX_VAL_LEN];
        if (state_allows_update(body_start, body_len) &&
            extract_named_field_value(body_start, body_len, "currentGear", gear_str, sizeof(gear_str))) {
            telemetry->gear = parse_gear_string(gear_str);
            telemetry->has_gear = true;
            EXLAP_DBG("  currentGear=%s -> %d", gear_str, telemetry->gear);
        }
        return;
    }

    EXLAP_DBG("  unhandled Dat url=%s", url);
}

bool exlap_parser_init(void)
{
    s_parse_buf_len = 0;
    memset(s_parse_buf, 0, sizeof(s_parse_buf));
    s_packet_count = 0;
    ESP_LOGI(TAG, "EXLAP parser initialized");
    return true;
}

bool exlap_parser_parse(const uint8_t *data, size_t len,
                        exlap_telemetry_t *telemetry)
{
    if (data == NULL || telemetry == NULL) {
        ESP_LOGE(TAG, "Invalid arguments to parser");
        return false;
    }

    if (len == 0) {
        return false;
    }

    memset(telemetry, 0, sizeof(*telemetry));
    clear_validity_flags(telemetry);

    if (s_parse_buf_len + len > sizeof(s_parse_buf)) {
        size_t keep = sizeof(s_parse_buf) / 4;
        if (keep > s_parse_buf_len) {
            keep = s_parse_buf_len;
        }
        ESP_LOGW(TAG, "Parse buffer overflow, discarding %zu bytes",
                 s_parse_buf_len - keep);
        memmove(s_parse_buf, &s_parse_buf[s_parse_buf_len - keep], keep);
        s_parse_buf_len = keep;
    }

    memcpy(&s_parse_buf[s_parse_buf_len], data, len);
    s_parse_buf_len += len;

    const char *scan = s_parse_buf;
    const char *buffer_end = s_parse_buf + s_parse_buf_len;
    bool found_message = false;
    size_t consumed = 0;

    while (scan < buffer_end) {
        const char *dat_start = find_str_in_range(scan, (size_t)(buffer_end - scan), "<Dat");
        if (dat_start == NULL) {
            break;
        }

        const char *open_end = find_char_in_range(dat_start, (size_t)(buffer_end - dat_start), '>');
        if (open_end == NULL) {
            consumed = (size_t)(dat_start - s_parse_buf);
            break;
        }

        const char *close_tag = find_str_in_range(open_end + 1,
                                                 (size_t)(buffer_end - (open_end + 1)),
                                                 "</Dat>");
        if (close_tag == NULL) {
            consumed = (size_t)(dat_start - s_parse_buf);
            break;
        }

        size_t dat_len = (size_t)(close_tag + strlen("</Dat>") - dat_start);
        parse_dat_message(dat_start, dat_len, telemetry);

        telemetry->timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
        s_packet_count++;
        telemetry->packet_count = s_packet_count;
        found_message = true;

        EXLAP_DBG("Parsed EXLAP Dat #%lu", (unsigned long)s_packet_count);

        scan = close_tag + strlen("</Dat>");
        consumed = (size_t)(scan - s_parse_buf);
    }

    if (consumed > 0) {
        size_t remaining = s_parse_buf_len - consumed;
        if (remaining > 0) {
            memmove(s_parse_buf, &s_parse_buf[consumed], remaining);
        }
        s_parse_buf_len = remaining;
    } else if (!found_message) {
        size_t keep_open = suffix_prefix_overlap(s_parse_buf, s_parse_buf_len, "<Dat");
        size_t keep_close = suffix_prefix_overlap(s_parse_buf, s_parse_buf_len, "</Dat>");
        size_t keep = (keep_open > keep_close) ? keep_open : keep_close;

        if (keep == 0) {
            s_parse_buf_len = 0;
        } else if (keep < s_parse_buf_len) {
            memmove(s_parse_buf, &s_parse_buf[s_parse_buf_len - keep], keep);
            s_parse_buf_len = keep;
        }
    }

    return found_message;
}

void exlap_parser_reset(void)
{
    s_parse_buf_len = 0;
    memset(s_parse_buf, 0, sizeof(s_parse_buf));
    ESP_LOGI(TAG, "Parser state reset");
}

void exlap_parser_print(const exlap_telemetry_t *telemetry)
{
    if (telemetry == NULL) {
        return;
    }

    ESP_LOGI(TAG, "=== EXLAP Telemetry (t=%lu ms, pkt=%lu) ===",
             (unsigned long)telemetry->timestamp_ms,
             (unsigned long)telemetry->packet_count);

    if (telemetry->has_vehicle_speed) {
        ESP_LOGI(TAG, "Speed: %.1f km/h", telemetry->vehicle_speed);
    }
    if (telemetry->has_engine_speed) {
        ESP_LOGI(TAG, "RPM: %.0f", telemetry->engine_speed);
    }
    if (telemetry->has_accelerator_pos) {
        ESP_LOGI(TAG, "Throttle: %.1f%%", telemetry->accelerator_pos);
    }
    if (telemetry->has_brake_pressure) {
        ESP_LOGI(TAG, "Brake: %.3f bar", telemetry->brake_pressure);
    }
    if (telemetry->has_steering_angle) {
        ESP_LOGI(TAG, "Steering: %.1f deg", telemetry->steering_angle);
    }
    if (telemetry->has_yaw_rate) {
        ESP_LOGI(TAG, "Yaw: %.2f deg/s", telemetry->yaw_rate);
    }
    if (telemetry->has_gear) {
        ESP_LOGI(TAG, "Gear: %d", telemetry->gear);
    }
    if (telemetry->has_wheel_speed_fl || telemetry->has_wheel_speed_fr ||
        telemetry->has_wheel_speed_rl || telemetry->has_wheel_speed_rr) {
        ESP_LOGI(TAG, "Wheel FL: %.1f | FR: %.1f | RL: %.1f | RR: %.1f",
                 telemetry->wheel_speed_fl,
                 telemetry->wheel_speed_fr,
                 telemetry->wheel_speed_rl,
                 telemetry->wheel_speed_rr);
    }

    ESP_LOGI(TAG, "=========================================");
}
