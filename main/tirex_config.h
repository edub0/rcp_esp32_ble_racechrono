/**
 * TireX Sensor Configuration Manager
 *
 * Tracks discovered TireX sensors, their user-facing names/corner mappings,
 * and the current on-bus operating mode for each sensor.
 */

#ifndef TIREX_CONFIG_H
#define TIREX_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define TIREX_NVS_NAMESPACE    "tirex_cfg"
#define TIREX_MAX_SENSORS      4
#define TIREX_NAME_LEN         32
#define TIREX_FULL_FRAME_PIXELS 192
#define TIREX_FULL_FRAME_CHANNELS 96
#define TIREX_MAX_ZONES        16

typedef enum {
    TIREX_CORNER_LEFT_FRONT = 0,
    TIREX_CORNER_RIGHT_FRONT = 1,
    TIREX_CORNER_LEFT_REAR = 2,
    TIREX_CORNER_RIGHT_REAR = 3,
    TIREX_CORNER_UNKNOWN = 255,
} tirex_corner_t;

typedef struct {
    bool present;
    uint32_t base_id;
    char name[TIREX_NAME_LEN + 1];
    tirex_corner_t corner;
    bool flip_orientation;
    bool full_frame_mode;
    bool full_frame_trace_enabled;
    uint8_t zone_count;
    uint8_t sample_rate_code;
    uint8_t observed_position;
    uint8_t observed_zone_count;
    bool observed_full_frame_mode;
    uint32_t first_seen_ms;
    uint32_t last_seen_ms;
    uint32_t last_announce_ms;
    uint32_t last_stats_ms;
    uint32_t last_config_ms;
    uint32_t last_apply_ms;
    uint32_t last_config_request_ms;
    uint32_t last_trace_ms;
    bool awaiting_config_confirm;
    uint8_t requested_position;
    uint8_t requested_zone_count;
    bool requested_full_frame_mode;
    uint8_t requested_sample_rate_code;
    bool config_requested;
    bool has_zone_data;
    float zone_temps[TIREX_MAX_ZONES];
    bool has_full_frame_data;
    float full_frame_pixels[TIREX_FULL_FRAME_PIXELS];
    float full_frame_channels[TIREX_FULL_FRAME_CHANNELS];
    uint8_t full_frame_assembly[TIREX_FULL_FRAME_CHANNELS];
    uint8_t full_frame_packet_index;
    uint8_t full_frame_packets_collected;
    bool full_frame_sync_armed;
    bool full_frame_synced;
    uint32_t full_frame_packet_count;
    uint32_t full_frame_complete_count;
    uint32_t full_frame_discarded_count;
    uint32_t last_full_frame_ms;
    uint32_t last_full_frame_packet_ms;
} tirex_sensor_snapshot_t;

typedef struct {
    uint8_t sensor_count;
    tirex_sensor_snapshot_t sensors[TIREX_MAX_SENSORS];
} tirex_snapshot_t;

typedef struct {
    char name[TIREX_NAME_LEN + 1];
    tirex_corner_t corner;
    bool flip_orientation;
    bool full_frame_mode;
    bool full_frame_trace_enabled;
    uint8_t zone_count;
    uint8_t sample_rate_code;
} tirex_sensor_profile_t;

bool tirex_config_init(void);
void tirex_config_reset(void);

bool tirex_config_get_snapshot(tirex_snapshot_t *snapshot);
bool tirex_config_get_live_frame(uint32_t base_id,
                                 uint8_t channels[TIREX_FULL_FRAME_CHANNELS],
                                 uint32_t *frame_count,
                                 uint32_t *last_frame_ms,
                                 bool *synced);
bool tirex_config_get_profile(uint32_t base_id, tirex_sensor_profile_t *profile);
bool tirex_config_update_profile(uint32_t base_id, const tirex_sensor_profile_t *profile);
bool tirex_config_apply(uint32_t base_id);

bool tirex_config_update_from_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc);
bool tirex_config_request_current(uint32_t base_id);

const char *tirex_corner_to_str(tirex_corner_t corner);

#endif /* TIREX_CONFIG_H */
