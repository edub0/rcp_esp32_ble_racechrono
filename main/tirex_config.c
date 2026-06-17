/**
 * TireX Sensor Configuration Manager
 */

#include "tirex_config.h"

#include "can_driver.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "telemetry_db.h"

#include <string.h>
#include <inttypes.h>

static const char *TAG = "tirex_cfg";

#define TIREX_CFG_MAGIC       0x54525843u /* "TRXC" */
#define TIREX_CFG_VERSION_V1  1u
#define TIREX_CFG_VERSION     2u
#define TIREX_CFG_KEY         "profiles"
#define TIREX_FULL_FRAME_COLS 16u
#define TIREX_FULL_FRAME_ROWS (TIREX_FULL_FRAME_PIXELS / TIREX_FULL_FRAME_COLS)
#define TIREX_FULL_FRAME_CHANNEL_COLS 8u
#define TIREX_FULL_FRAME_PACKETS (TIREX_FULL_FRAME_CHANNELS / 8u)
#define TIREX_FULL_FRAME_GAP_RESET_MS 250u

typedef struct {
    uint32_t base_id;
    char name[TIREX_NAME_LEN + 1];
    tirex_corner_t corner;
    bool flip_orientation;
    bool full_frame_mode;
    bool full_frame_trace_enabled;
    uint8_t zone_count;
    uint8_t sample_rate_code;
} tirex_nvs_profile_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    tirex_nvs_profile_t profiles[TIREX_MAX_SENSORS];
} tirex_nvs_store_t;

typedef struct {
    uint32_t base_id;
    char name[TIREX_NAME_LEN + 1];
    tirex_corner_t corner;
    bool flip_orientation;
    bool full_frame_mode;
    uint8_t zone_count;
    uint8_t sample_rate_code;
} tirex_nvs_profile_v1_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    tirex_nvs_profile_v1_t profiles[TIREX_MAX_SENSORS];
} tirex_nvs_store_v1_t;

static tirex_sensor_snapshot_t s_sensors[TIREX_MAX_SENSORS];
static tirex_sensor_snapshot_t s_sort_tmp;
static tirex_sensor_snapshot_t s_confirmed_snapshot;
static SemaphoreHandle_t s_mutex = NULL;

static const char *corner_names[] = {
    "left_front",
    "right_front",
    "left_rear",
    "right_rear",
};

static void log_full_frame_mapcheck(const tirex_sensor_snapshot_t *sensor, uint32_t frame_index)
{
    float total_sum = 0.0f;
    float left_edge_sum = 0.0f;
    float right_edge_sum = 0.0f;
    float top_edge_sum = 0.0f;
    float bottom_edge_sum = 0.0f;
    float center_sum = 0.0f;
    float pair_diff_sum = 0.0f;
    uint32_t total_count = 0;
    uint32_t left_edge_count = 0;
    uint32_t right_edge_count = 0;
    uint32_t top_edge_count = 0;
    uint32_t bottom_edge_count = 0;
    uint32_t center_count = 0;
    uint32_t pair_diff_count = 0;

    for (uint32_t row = 0; row < TIREX_FULL_FRAME_ROWS; row++) {
        for (uint32_t col = 0; col < TIREX_FULL_FRAME_COLS; col++) {
            uint32_t idx = row * TIREX_FULL_FRAME_COLS + col;
            float v = sensor->full_frame_pixels[idx];
            total_sum += v;
            total_count++;

            if (col < 2) {
                left_edge_sum += v;
                left_edge_count++;
            }
            if (col >= (TIREX_FULL_FRAME_COLS - 2u)) {
                right_edge_sum += v;
                right_edge_count++;
            }
            if (row < 2) {
                top_edge_sum += v;
                top_edge_count++;
            }
            if (row >= (TIREX_FULL_FRAME_ROWS - 2u)) {
                bottom_edge_sum += v;
                bottom_edge_count++;
            }
            if (row >= 4 && row <= 7 && col >= 6 && col <= 9) {
                center_sum += v;
                center_count++;
            }

            if ((col & 1u) == 0u && (col + 1u) < TIREX_FULL_FRAME_COLS) {
                float diff = v - sensor->full_frame_pixels[idx + 1u];
                pair_diff_sum += (diff < 0.0f) ? -diff : diff;
                pair_diff_count++;
            }
        }
    }

    if (total_count == 0 || left_edge_count == 0 || right_edge_count == 0 ||
        top_edge_count == 0 || bottom_edge_count == 0 || center_count == 0 ||
        pair_diff_count == 0) {
        return;
    }

    float total_avg = total_sum / (float)total_count;
    float left_avg = left_edge_sum / (float)left_edge_count;
    float right_avg = right_edge_sum / (float)right_edge_count;
    float top_avg = top_edge_sum / (float)top_edge_count;
    float bottom_avg = bottom_edge_sum / (float)bottom_edge_count;
    float center_avg = center_sum / (float)center_count;
    float pair_avg_diff = pair_diff_sum / (float)pair_diff_count;

    ESP_LOGI(TAG,
             "TireX full-frame mapcheck: base=0x%08" PRIX32 " frame=%" PRIu32 " avg=%.1f left=%.1f right=%.1f top=%.1f bottom=%.1f center=%.1f edge-center=%.1f LR=%.1f TB=%.1f pair-diff=%.2f",
             sensor->base_id,
             frame_index,
             total_avg,
             left_avg,
             right_avg,
             top_avg,
             bottom_avg,
             center_avg,
             center_avg - ((left_avg + right_avg + top_avg + bottom_avg) * 0.25f),
             left_avg - right_avg,
             top_avg - bottom_avg,
             pair_avg_diff);
}

static void set_default_sensor(tirex_sensor_snapshot_t *sensor)
{
    memset(sensor, 0, sizeof(*sensor));
    sensor->corner = TIREX_CORNER_UNKNOWN;
    sensor->zone_count = 4;
    sensor->sample_rate_code = 16;
    sensor->full_frame_mode = false;
    sensor->flip_orientation = false;
    sensor->full_frame_trace_enabled = false;
    sensor->config_requested = false;
    sensor->last_config_request_ms = 0;
    sensor->last_trace_ms = 0;
    sensor->full_frame_packet_count = 0;
    sensor->full_frame_complete_count = 0;
    sensor->full_frame_discarded_count = 0;
    sensor->last_full_frame_ms = 0;
    sensor->last_full_frame_packet_ms = 0;
}

static const char *default_name_for_sensor(const tirex_sensor_snapshot_t *sensor, char *buf, size_t len)
{
    if (sensor->name[0] != '\0') {
        return sensor->name;
    }

    switch (sensor->corner) {
    case TIREX_CORNER_LEFT_FRONT:  return "left_front";
    case TIREX_CORNER_RIGHT_FRONT: return "right_front";
    case TIREX_CORNER_LEFT_REAR:   return "left_rear";
    case TIREX_CORNER_RIGHT_REAR:  return "right_rear";
    default:
        snprintf(buf, len, "sensor_0x%04" PRIX32, sensor->base_id);
        return buf;
    }
}

static tirex_corner_t position_to_corner(uint8_t position)
{
    switch (position) {
    case 0: return TIREX_CORNER_LEFT_FRONT;
    case 1: return TIREX_CORNER_RIGHT_FRONT;
    case 2: return TIREX_CORNER_LEFT_REAR;
    case 3: return TIREX_CORNER_RIGHT_REAR;
    default: return TIREX_CORNER_UNKNOWN;
    }
}

static uint8_t corner_to_position(tirex_corner_t corner)
{
    switch (corner) {
    case TIREX_CORNER_LEFT_FRONT:
        return 0;
    case TIREX_CORNER_RIGHT_FRONT:
        return 1;
    case TIREX_CORNER_LEFT_REAR:
        return 2;
    case TIREX_CORNER_RIGHT_REAR:
        return 3;
    default:
        return 0;
    }
}

static int find_sensor_index_by_base_id(uint32_t base_id)
{
    for (int i = 0; i < TIREX_MAX_SENSORS; i++) {
        if (s_sensors[i].base_id == base_id && base_id != 0) {
            return i;
        }
    }
    return -1;
}

static int find_free_sensor_index(void)
{
    for (int i = 0; i < TIREX_MAX_SENSORS; i++) {
        if (s_sensors[i].base_id == 0) {
            return i;
        }
    }
    return -1;
}

static int get_or_create_sensor_index(uint32_t base_id)
{
    int idx = find_sensor_index_by_base_id(base_id);
    if (idx >= 0) {
        return idx;
    }

    idx = find_free_sensor_index();
    if (idx < 0 || idx >= TIREX_MAX_SENSORS) {
        idx = 0;
        for (int i = 1; i < TIREX_MAX_SENSORS; i++) {
            if (s_sensors[i].last_seen_ms < s_sensors[idx].last_seen_ms) {
                idx = i;
            }
        }
        ESP_LOGW(TAG, "TireX sensor table full, reusing slot %d for base 0x%08" PRIX32, idx, base_id);
    }

    set_default_sensor(&s_sensors[idx]);
    s_sensors[idx].base_id = base_id;
    s_sensors[idx].present = true;
    s_sensors[idx].first_seen_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_sensors[idx].last_seen_ms = s_sensors[idx].first_seen_ms;
    ESP_LOGI(TAG, "Discovered TireX sensor base=0x%08" PRIX32 " slot=%d", base_id, idx);
    return idx;
}

static void sort_sensors_by_base_id(void)
{
    for (int i = 0; i < TIREX_MAX_SENSORS - 1; i++) {
        for (int j = i + 1; j < TIREX_MAX_SENSORS; j++) {
            if (s_sensors[j].base_id != 0 &&
                (s_sensors[i].base_id == 0 || s_sensors[j].base_id < s_sensors[i].base_id)) {
                s_sort_tmp = s_sensors[i];
                s_sensors[i] = s_sensors[j];
                s_sensors[j] = s_sort_tmp;
            }
        }
    }
}

static void load_defaults(void)
{
    memset(s_sensors, 0, sizeof(s_sensors));
    for (int i = 0; i < TIREX_MAX_SENSORS; i++) {
        set_default_sensor(&s_sensors[i]);
    }
}

static bool persist_profiles(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(TIREX_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS for save: %s", esp_err_to_name(ret));
        return false;
    }

    tirex_nvs_store_t store = {
        .magic = TIREX_CFG_MAGIC,
        .version = TIREX_CFG_VERSION,
    };
    for (int i = 0; i < TIREX_MAX_SENSORS; i++) {
        store.profiles[i].base_id = s_sensors[i].base_id;
        strncpy(store.profiles[i].name, s_sensors[i].name, TIREX_NAME_LEN);
        store.profiles[i].name[TIREX_NAME_LEN] = '\0';
        store.profiles[i].corner = s_sensors[i].corner;
        store.profiles[i].flip_orientation = s_sensors[i].flip_orientation;
        store.profiles[i].full_frame_mode = s_sensors[i].full_frame_mode;
        store.profiles[i].full_frame_trace_enabled = s_sensors[i].full_frame_trace_enabled;
        store.profiles[i].zone_count = s_sensors[i].zone_count;
        store.profiles[i].sample_rate_code = s_sensors[i].sample_rate_code;
    }

    ret = nvs_set_blob(handle, TIREX_CFG_KEY, &store, sizeof(store));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to store TireX profiles: %s", esp_err_to_name(ret));
        nvs_close(handle);
        return false;
    }

    ret = nvs_commit(handle);
    nvs_close(handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to commit TireX profiles: %s", esp_err_to_name(ret));
        return false;
    }

    return true;
}

static bool load_profiles(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(TIREX_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        return false;
    }

    size_t len = 0;
    ret = nvs_get_blob(handle, TIREX_CFG_KEY, NULL, &len);
    if (ret != ESP_OK || (len != sizeof(tirex_nvs_store_t) && len != sizeof(tirex_nvs_store_v1_t))) {
        nvs_close(handle);
        return false;
    }

    tirex_nvs_store_t store_v2;
    tirex_nvs_store_v1_t store_v1;
    void *store_ptr = (len == sizeof(tirex_nvs_store_t)) ? (void *)&store_v2 : (void *)&store_v1;
    ret = nvs_get_blob(handle, TIREX_CFG_KEY, store_ptr, &len);
    nvs_close(handle);
    if (ret != ESP_OK) {
        return false;
    }

    load_defaults();
    if (len == sizeof(tirex_nvs_store_t)) {
        if (store_v2.magic != TIREX_CFG_MAGIC || store_v2.version != TIREX_CFG_VERSION) {
            return false;
        }
        for (int i = 0; i < TIREX_MAX_SENSORS; i++) {
            s_sensors[i].base_id = store_v2.profiles[i].base_id;
            s_sensors[i].present = (store_v2.profiles[i].base_id != 0);
            strncpy(s_sensors[i].name, store_v2.profiles[i].name, TIREX_NAME_LEN);
            s_sensors[i].name[TIREX_NAME_LEN] = '\0';
            s_sensors[i].corner = store_v2.profiles[i].corner;
            s_sensors[i].flip_orientation = store_v2.profiles[i].flip_orientation;
            s_sensors[i].full_frame_mode = store_v2.profiles[i].full_frame_mode;
            s_sensors[i].full_frame_trace_enabled = store_v2.profiles[i].full_frame_trace_enabled;
            s_sensors[i].zone_count = store_v2.profiles[i].zone_count;
            s_sensors[i].sample_rate_code = store_v2.profiles[i].sample_rate_code;
        }
    } else {
        if (store_v1.magic != TIREX_CFG_MAGIC || store_v1.version != TIREX_CFG_VERSION_V1) {
            return false;
        }
        for (int i = 0; i < TIREX_MAX_SENSORS; i++) {
            s_sensors[i].base_id = store_v1.profiles[i].base_id;
            s_sensors[i].present = (store_v1.profiles[i].base_id != 0);
            strncpy(s_sensors[i].name, store_v1.profiles[i].name, TIREX_NAME_LEN);
            s_sensors[i].name[TIREX_NAME_LEN] = '\0';
            s_sensors[i].corner = store_v1.profiles[i].corner;
            s_sensors[i].flip_orientation = store_v1.profiles[i].flip_orientation;
            s_sensors[i].full_frame_mode = store_v1.profiles[i].full_frame_mode;
            s_sensors[i].full_frame_trace_enabled = false;
            s_sensors[i].zone_count = store_v1.profiles[i].zone_count;
            s_sensors[i].sample_rate_code = store_v1.profiles[i].sample_rate_code;
        }
    }
    return true;
}

static void update_observed_zone_order(tirex_sensor_snapshot_t *sensor)
{
    if (!sensor->has_zone_data) {
        return;
    }

    if (!sensor->flip_orientation) {
        return;
    }

    float tmp[TIREX_MAX_ZONES];
    memcpy(tmp, sensor->zone_temps, sizeof(tmp));
    for (uint8_t i = 0; i < sensor->observed_zone_count && i < TIREX_MAX_ZONES; i++) {
        sensor->zone_temps[i] = tmp[(sensor->observed_zone_count - 1) - i];
    }
}

static void reset_full_frame_assembly(tirex_sensor_snapshot_t *sensor, bool arm_sync)
{
    if (sensor == NULL) {
        return;
    }

    memset(sensor->full_frame_assembly, 0, sizeof(sensor->full_frame_assembly));
    sensor->full_frame_packet_index = 0;
    sensor->full_frame_packets_collected = 0;
    sensor->last_full_frame_packet_ms = 0;
    sensor->full_frame_sync_armed = arm_sync;
    sensor->full_frame_synced = false;
}

static bool tirex_config_matches_requested(const tirex_sensor_snapshot_t *sensor)
{
    if (!sensor->awaiting_config_confirm) {
        return false;
    }

    return sensor->observed_position == sensor->requested_position &&
           sensor->observed_zone_count == sensor->requested_zone_count &&
           sensor->observed_full_frame_mode == sensor->requested_full_frame_mode &&
           sensor->sample_rate_code == sensor->requested_sample_rate_code;
}

static void publish_confirmed_tirex_sensor(const tirex_sensor_snapshot_t *sensor)
{
    if (sensor != NULL) {
        telemetry_db_update_tirex_config(sensor);
    }
}

static void parse_temperature_broadcast(tirex_sensor_snapshot_t *sensor,
                                        uint32_t can_id,
                                        const uint8_t *data,
                                        uint8_t dlc)
{
    uint8_t base_offset = (uint8_t)(can_id - sensor->base_id);
    if (base_offset == 0x20) {
        uint8_t zone_count = dlc;
        if (zone_count > TIREX_MAX_ZONES) {
            zone_count = TIREX_MAX_ZONES;
        }
        sensor->observed_zone_count = zone_count;
        sensor->zone_count = sensor->zone_count == 0 ? zone_count : sensor->zone_count;
        for (uint8_t i = 0; i < zone_count; i++) {
            sensor->zone_temps[i] = data[i] * 0.5f;
        }
        sensor->has_zone_data = true;
    } else if (base_offset == 0x21) {
        uint8_t start = 8;
        uint8_t zone_count = start + dlc;
        if (zone_count > TIREX_MAX_ZONES) {
            zone_count = TIREX_MAX_ZONES;
        }
        for (uint8_t i = 0; i < dlc && (start + i) < TIREX_MAX_ZONES; i++) {
            sensor->zone_temps[start + i] = data[i] * 0.5f;
        }
        sensor->observed_zone_count = zone_count;
        sensor->has_zone_data = true;
    }
}

static void parse_full_frame(tirex_sensor_snapshot_t *sensor,
                             const uint8_t *data,
                             uint8_t dlc)
{
    if (dlc != 8) {
        sensor->full_frame_discarded_count++;
        reset_full_frame_assembly(sensor, sensor->full_frame_sync_armed);
        return;
    }

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (sensor->last_full_frame_packet_ms != 0 &&
        (now_ms - sensor->last_full_frame_packet_ms) > TIREX_FULL_FRAME_GAP_RESET_MS) {
        sensor->full_frame_discarded_count++;
        reset_full_frame_assembly(sensor, sensor->full_frame_sync_armed);
    }

    /*
     * TireX reports an 8 x 12 channel grid in 12 consecutive CAN packets.
     * Each reported channel represents two adjacent horizontal pixels in the
     * documented 16 x 12 physical array.
     */
    uint8_t packet_slot = sensor->full_frame_packet_index;
    uint16_t start = (uint16_t)(packet_slot * 8u);
    float min_v = (float)data[0];
    float max_v = (float)data[0];
    float sum_v = 0.0f;

    for (uint8_t i = 0; i < dlc; i++) {
        float v = (float)data[i];
        sensor->full_frame_assembly[start + i] = data[i];
        sum_v += v;
        if (v < min_v) {
            min_v = v;
        }
        if (v > max_v) {
            max_v = v;
        }
    }
    sensor->last_full_frame_packet_ms = now_ms;
    sensor->full_frame_mode = true;
    sensor->observed_full_frame_mode = true;
    sensor->full_frame_packet_index++;
    sensor->full_frame_packets_collected++;
    sensor->full_frame_packet_count++;

    if (sensor->full_frame_trace_enabled) {
        char payload_hex[3 * 8 + 1] = {0};
        size_t hex_off = 0;
        for (uint8_t i = 0; i < dlc && i < 8; i++) {
            hex_off += (size_t)snprintf(&payload_hex[hex_off], sizeof(payload_hex) - hex_off,
                                        "%s%02X", (i == 0) ? "" : " ", data[i]);
            if (hex_off >= sizeof(payload_hex)) {
                break;
            }
        }
        float avg_v = sum_v / (float)dlc;
        ESP_LOGI(TAG,
                 "TireX full-frame trace: base=0x%08" PRIX32 " pkt=%" PRIu32 " slot=%u start=%u dlc=%u bytes=[%s] min=%.1f max=%.1f avg=%.1f span=%.1f",
                 sensor->base_id,
                 sensor->full_frame_packet_count,
                 packet_slot,
                 start,
                 dlc,
                 payload_hex,
                 min_v,
                 max_v,
                 avg_v,
                 max_v - min_v);
    }

    if (sensor->full_frame_packets_collected == TIREX_FULL_FRAME_PACKETS) {
        if (sensor->full_frame_sync_armed) {
            for (uint16_t row = 0; row < TIREX_FULL_FRAME_ROWS; row++) {
                for (uint16_t col = 0; col < TIREX_FULL_FRAME_CHANNEL_COLS; col++) {
                    uint16_t channel_idx = row * TIREX_FULL_FRAME_CHANNEL_COLS + col;
                    uint16_t pixel_idx = row * TIREX_FULL_FRAME_COLS + (col * 2u);
                    float v = (float)sensor->full_frame_assembly[channel_idx];
                    sensor->full_frame_channels[channel_idx] = v;
                    sensor->full_frame_pixels[pixel_idx] = v;
                    sensor->full_frame_pixels[pixel_idx + 1u] = v;
                }
            }
            sensor->has_full_frame_data = true;
            sensor->full_frame_synced = true;
            sensor->last_full_frame_ms = now_ms;
            sensor->full_frame_complete_count++;
            log_full_frame_mapcheck(sensor, sensor->full_frame_complete_count);
        } else {
            sensor->full_frame_discarded_count++;
        }

        memset(sensor->full_frame_assembly, 0, sizeof(sensor->full_frame_assembly));
        sensor->full_frame_packet_index = 0;
        sensor->full_frame_packets_collected = 0;
    }
}

static bool parse_announcement_or_config(tirex_sensor_snapshot_t *sensor,
                                         bool is_config,
                                         const uint8_t *data,
                                         uint8_t dlc)
{
    if (dlc < 3) {
        return false;
    }

    sensor->sample_rate_code = data[0];
    sensor->zone_count = data[1];
    sensor->observed_zone_count = data[1];
    sensor->observed_position = data[2];
    sensor->corner = position_to_corner(data[2]);
    if (is_config && dlc >= 4) {
        sensor->observed_full_frame_mode = (data[3] != 0);
        sensor->full_frame_mode = sensor->observed_full_frame_mode;
    }
    sensor->last_announce_ms = (uint32_t)(esp_timer_get_time() / 1000);
    return true;
}

static void parse_stats(tirex_sensor_snapshot_t *sensor, const uint8_t *data, uint8_t dlc)
{
    if (dlc == 0) {
        return;
    }

    (void)data;
    sensor->last_stats_ms = (uint32_t)(esp_timer_get_time() / 1000);
}

static uint8_t effective_position_for_sensor(const tirex_sensor_snapshot_t *sensor)
{
    return corner_to_position(sensor->corner);
}

static bool send_tirex_config_frame(uint32_t base_id, const tirex_sensor_snapshot_t *sensor)
{
    twai_message_t msg = {0};
    uint32_t cfg_id = base_id + 3;

    msg.identifier = cfg_id;
    msg.extd = 1;
    msg.rtr = 0;
    msg.ss = 0;
    msg.dlc_non_comp = 0;
    msg.data_length_code = 8;

    msg.data[0] = sensor->sample_rate_code;
    msg.data[1] = sensor->zone_count;
    msg.data[2] = effective_position_for_sensor(sensor);
    msg.data[3] = sensor->full_frame_mode ? 1 : 0;
    msg.data[4] = 0;
    msg.data[5] = 0;
    msg.data[6] = 0;
    msg.data[7] = 0;

    esp_err_t ret = can_driver_transmit(&msg, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send TireX config frame 0x%08" PRIX32 ": %s",
                 cfg_id, esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "Sent TireX config frame base=0x%08" PRIX32 " pos=%u zones=%u full_frame=%u rate=%u",
             base_id, msg.data[2], msg.data[1], msg.data[3], msg.data[0]);
    return true;
}

static const char *tirex_frame_kind_str(uint8_t offset)
{
    switch (offset) {
    case 0x00:
        return "announce/config";
    case 0x02:
        return "stats";
    case 0x03:
        return "config";
    case 0x20:
    case 0x21:
        return "temp-broadcast";
    case 0x40:
        return "full-frame";
    default:
        return "unknown";
    }
}

bool tirex_config_init(void)
{
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
        if (s_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create TireX config mutex");
            return false;
        }
    }

    load_defaults();
    if (load_profiles()) {
        ESP_LOGI(TAG, "Loaded TireX profiles from NVS");
    } else {
        ESP_LOGI(TAG, "No stored TireX profiles found; using defaults");
    }

    return true;
}

void tirex_config_reset(void)
{
    if (s_mutex == NULL || xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

    load_defaults();
    xSemaphoreGive(s_mutex);
}

bool tirex_config_get_snapshot(tirex_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }

    if (s_mutex == NULL || xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    snapshot->sensor_count = 0;
    memset(snapshot->sensors, 0, sizeof(snapshot->sensors));

    sort_sensors_by_base_id();
    for (int i = 0; i < TIREX_MAX_SENSORS; i++) {
        if (s_sensors[i].base_id == 0) {
            continue;
        }
        snapshot->sensors[snapshot->sensor_count] = s_sensors[i];
        if (snapshot->sensors[snapshot->sensor_count].corner == TIREX_CORNER_UNKNOWN) {
            snapshot->sensors[snapshot->sensor_count].corner = position_to_corner(
                snapshot->sensors[snapshot->sensor_count].observed_position);
        }
        if (snapshot->sensors[snapshot->sensor_count].name[0] == '\0') {
            char tmp[32];
            const char *fallback = default_name_for_sensor(&snapshot->sensors[snapshot->sensor_count], tmp, sizeof(tmp));
            strncpy(snapshot->sensors[snapshot->sensor_count].name, fallback, TIREX_NAME_LEN);
            snapshot->sensors[snapshot->sensor_count].name[TIREX_NAME_LEN] = '\0';
        }
        update_observed_zone_order(&snapshot->sensors[snapshot->sensor_count]);
        snapshot->sensor_count++;
    }

    xSemaphoreGive(s_mutex);
    return true;
}

bool tirex_config_get_live_frame(uint32_t base_id,
                                 uint8_t channels[TIREX_FULL_FRAME_CHANNELS],
                                 uint32_t *frame_count,
                                 uint32_t *last_frame_ms,
                                 bool *synced)
{
    if (channels == NULL || frame_count == NULL || last_frame_ms == NULL || synced == NULL) {
        return false;
    }

    if (s_mutex == NULL || xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return false;
    }

    int idx = find_sensor_index_by_base_id(base_id);
    if (idx < 0 || !s_sensors[idx].has_full_frame_data) {
        xSemaphoreGive(s_mutex);
        return false;
    }

    for (uint16_t i = 0; i < TIREX_FULL_FRAME_CHANNELS; i++) {
        float value = s_sensors[idx].full_frame_channels[i];
        if (value < 0.0f) {
            value = 0.0f;
        } else if (value > 255.0f) {
            value = 255.0f;
        }
        channels[i] = (uint8_t)(value + 0.5f);
    }
    *frame_count = s_sensors[idx].full_frame_complete_count;
    *last_frame_ms = s_sensors[idx].last_full_frame_ms;
    *synced = s_sensors[idx].full_frame_synced;

    xSemaphoreGive(s_mutex);
    return true;
}

bool tirex_config_get_profile(uint32_t base_id, tirex_sensor_profile_t *profile)
{
    if (profile == NULL) {
        return false;
    }

    if (s_mutex == NULL || xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    int idx = find_sensor_index_by_base_id(base_id);
    if (idx < 0) {
        xSemaphoreGive(s_mutex);
        return false;
    }

    strncpy(profile->name, s_sensors[idx].name, sizeof(profile->name) - 1);
    profile->corner = s_sensors[idx].corner;
    profile->flip_orientation = s_sensors[idx].flip_orientation;
    profile->full_frame_mode = s_sensors[idx].full_frame_mode;
    profile->full_frame_trace_enabled = s_sensors[idx].full_frame_trace_enabled;
    profile->zone_count = s_sensors[idx].zone_count;
    profile->sample_rate_code = s_sensors[idx].sample_rate_code;

    xSemaphoreGive(s_mutex);
    return true;
}

bool tirex_config_update_profile(uint32_t base_id, const tirex_sensor_profile_t *profile)
{
    if (profile == NULL || base_id == 0) {
        return false;
    }

    if (s_mutex == NULL || xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    int idx = get_or_create_sensor_index(base_id);
    s_sensors[idx].present = true;
    s_sensors[idx].base_id = base_id;
    strncpy(s_sensors[idx].name, profile->name, TIREX_NAME_LEN);
    s_sensors[idx].name[TIREX_NAME_LEN] = '\0';
    s_sensors[idx].corner = profile->corner;
    s_sensors[idx].flip_orientation = profile->flip_orientation;
    s_sensors[idx].full_frame_mode = profile->full_frame_mode;
    s_sensors[idx].full_frame_trace_enabled = profile->full_frame_trace_enabled;
    s_sensors[idx].zone_count = profile->zone_count;
    s_sensors[idx].sample_rate_code = profile->sample_rate_code;
    s_sensors[idx].last_apply_ms = (uint32_t)(esp_timer_get_time() / 1000);

    bool ok = persist_profiles();
    if (ok) {
        ESP_LOGI(TAG,
                 "Updated TireX profile base=0x%08" PRIX32 " name=%s corner=%s flip=%d zones=%u full_frame=%d trace=%d rate=%u",
                 base_id,
                 s_sensors[idx].name,
                 tirex_corner_to_str(s_sensors[idx].corner),
                 s_sensors[idx].flip_orientation ? 1 : 0,
                 s_sensors[idx].zone_count,
                 s_sensors[idx].full_frame_mode ? 1 : 0,
                 s_sensors[idx].full_frame_trace_enabled ? 1 : 0,
                 s_sensors[idx].sample_rate_code);
    }
    xSemaphoreGive(s_mutex);
    return ok;
}

bool tirex_config_apply(uint32_t base_id)
{
    if (s_mutex == NULL || xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    int idx = find_sensor_index_by_base_id(base_id);
    if (idx < 0) {
        xSemaphoreGive(s_mutex);
        return false;
    }

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_sensors[idx].awaiting_config_confirm = true;
    s_sensors[idx].requested_position = effective_position_for_sensor(&s_sensors[idx]);
    s_sensors[idx].requested_zone_count = s_sensors[idx].zone_count;
    s_sensors[idx].requested_full_frame_mode = s_sensors[idx].full_frame_mode;
    s_sensors[idx].requested_sample_rate_code = s_sensors[idx].sample_rate_code;
    s_sensors[idx].last_config_request_ms = now_ms;

    bool ok = send_tirex_config_frame(base_id, &s_sensors[idx]);
    if (ok) {
        s_sensors[idx].last_apply_ms = now_ms;
        ok = persist_profiles();
    } else {
        s_sensors[idx].awaiting_config_confirm = false;
    }

    xSemaphoreGive(s_mutex);

    if (ok) {
        (void)tirex_config_request_current(base_id);
    }

    return ok;
}

bool tirex_config_request_current(uint32_t base_id)
{
    twai_message_t msg = {0};
    msg.identifier = base_id + 3;
    msg.extd = 1;
    msg.rtr = 0;
    msg.ss = 0;
    msg.dlc_non_comp = 0;
    msg.data_length_code = 0;

    esp_err_t ret = can_driver_transmit(&msg, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to request TireX config for 0x%08" PRIX32 ": %s",
                 base_id, esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "Requested TireX config for base 0x%08" PRIX32, base_id);
    return true;
}

bool tirex_config_update_from_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    if (data == NULL) {
        return false;
    }

    uint8_t offset = (uint8_t)(can_id & 0xFF);
    if (offset != 0x00 && offset != 0x02 && offset != 0x03 && offset != 0x20 &&
        offset != 0x21 && offset != 0x40) {
        return false;
    }

    uint32_t base_id = can_id - offset;
    if (s_mutex == NULL || xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    int idx = get_or_create_sensor_index(base_id);
    tirex_sensor_snapshot_t *sensor = &s_sensors[idx];
    bool request_current = false;
    bool publish_snapshot = false;

    sensor->present = true;
    sensor->last_seen_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t now_ms = sensor->last_seen_ms;
    if (sensor->first_seen_ms == 0) {
        sensor->first_seen_ms = sensor->last_seen_ms;
    }

    if ((now_ms - sensor->last_trace_ms) >= 1000u || sensor->last_trace_ms == 0) {
        sensor->last_trace_ms = now_ms;
        ESP_LOGI(TAG,
                 "TireX trace: raw=0x%08" PRIX32 " base=0x%08" PRIX32 " offset=0x%02X kind=%s dlc=%u present=%d seen=%lu corner=%s observed_pos=%u obs_zones=%u obs_full=%u",
                 can_id,
                 base_id,
                 offset,
                 tirex_frame_kind_str(offset),
                 dlc,
                 sensor->present ? 1 : 0,
                 (unsigned long)sensor->last_seen_ms,
                 tirex_corner_to_str(sensor->corner),
                 (unsigned)sensor->observed_position,
                 (unsigned)sensor->observed_zone_count,
                 sensor->observed_full_frame_mode ? 1 : 0);
    }

    if (!sensor->config_requested &&
        (now_ms - sensor->last_config_request_ms) >= 5000u) {
        sensor->config_requested = true;
        sensor->last_config_request_ms = now_ms;
        request_current = true;
    }

    switch (offset) {
    case 0x00:
        (void)parse_announcement_or_config(sensor, false, data, dlc);
        break;
    case 0x02:
        parse_stats(sensor, data, dlc);
        break;
    case 0x03:
        (void)parse_announcement_or_config(sensor, true, data, dlc);
        sensor->last_config_ms = sensor->last_seen_ms;
        break;
    case 0x20:
    case 0x21:
        parse_temperature_broadcast(sensor, can_id, data, dlc);
        break;
    case 0x40:
        parse_full_frame(sensor, data, dlc);
        break;
    default:
        break;
    }

    if (offset == 0x00 || offset == 0x03) {
        if (sensor->awaiting_config_confirm) {
            if (tirex_config_matches_requested(sensor)) {
                sensor->awaiting_config_confirm = false;
                reset_full_frame_assembly(sensor, sensor->requested_full_frame_mode);
                if (sensor->requested_full_frame_mode) {
                    sensor->has_full_frame_data = false;
                }
                s_confirmed_snapshot = *sensor;
                publish_snapshot = true;
            }
        } else if (sensor->config_requested) {
            sensor->config_requested = false;
            s_confirmed_snapshot = *sensor;
            publish_snapshot = true;
        }
    }

    if (sensor->name[0] == '\0') {
        switch (sensor->corner) {
        case TIREX_CORNER_LEFT_FRONT:  strncpy(sensor->name, "left_front", sizeof(sensor->name) - 1); break;
        case TIREX_CORNER_RIGHT_FRONT: strncpy(sensor->name, "right_front", sizeof(sensor->name) - 1); break;
        case TIREX_CORNER_LEFT_REAR:   strncpy(sensor->name, "left_rear", sizeof(sensor->name) - 1); break;
        case TIREX_CORNER_RIGHT_REAR:  strncpy(sensor->name, "right_rear", sizeof(sensor->name) - 1); break;
        default: break;
        }
    }

    xSemaphoreGive(s_mutex);

    if (request_current) {
        (void)tirex_config_request_current(base_id);
    }

    if (publish_snapshot) {
        publish_confirmed_tirex_sensor(&s_confirmed_snapshot);
    }

    return true;
}

const char *tirex_corner_to_str(tirex_corner_t corner)
{
    switch (corner) {
    case TIREX_CORNER_LEFT_FRONT:  return corner_names[0];
    case TIREX_CORNER_RIGHT_FRONT: return corner_names[1];
    case TIREX_CORNER_LEFT_REAR:   return corner_names[2];
    case TIREX_CORNER_RIGHT_REAR:  return corner_names[3];
    case TIREX_CORNER_UNKNOWN:
    default:
        return "unknown";
    }
}
