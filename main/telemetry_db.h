/**
 * Telemetry Database
 * 
 * Central hub that aggregates telemetry data from all sources:
 * - TireX (CAN bus): Tire temperatures, pressures
 * - EXLAP (TCP): Vehicle speed, RPM, throttle, brake, steering, etc.
 * 
 * Provides a unified interface for:
 * - BLE forwarding to RaceChrono
 * - Web dashboard display
 * - Future: logging, alerts
 * 
 * Thread-safe: uses mutex for concurrent access from CAN task (Core 1)
 * and EXLAP task (Core 0).
 */

#ifndef TELEMETRY_DB_H
#define TELEMETRY_DB_H

#include "tirex_config.h"

#include <stdint.h>
#include <stdbool.h>

/* Maximum telemetry update rate */
#define TELEMETRY_MAX_HZ 50

/* Update interval for BLE forwarding */
#define TELEMETRY_BLE_FORWARD_MS 200

/* Update interval for web dashboard */
#define TELEMETRY_WEB_UPDATE_MS 500

/**
 * Telemetry data source
 */
typedef enum {
    TELEMETRY_SRC_TIREX = 0,
    TELEMETRY_SRC_EXLAP = 1,
    TELEMETRY_SRC_COMBINED = 2,
} telemetry_src_t;

/**
 * TireX configuration summary mirrored into the telemetry database.
 * This keeps downstream consumers aware of the latest on-bus operating mode
 * without carrying the full frame matrix in the shared runtime snapshot.
 */
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
    bool awaiting_config_confirm;
    bool config_requested;
    bool has_zone_data;
    bool has_full_frame_data;
    uint32_t full_frame_packet_count;
    uint32_t last_full_frame_ms;
} telemetry_tirex_sensor_t;

/**
 * Complete vehicle telemetry snapshot
 */
typedef struct {
    /* Timestamp of last update (ms since boot) */
    uint32_t timestamp_ms;
    
    /* === Vehicle Dynamics (from EXLAP) === */
    float vehicle_speed_kmh;     /* km/h */
    float engine_rpm;            /* RPM */
    float throttle_pct;          /* 0-100% */
    float brake_pressure_bar;    /* bar */
    float steering_angle_deg;    /* degrees (-180 to +180) */
    float yaw_rate;              /* deg/s */
    int8_t gear;                 /* 0=Neutral, 1-8=Gears, -1=Reverse */
    
    /* Wheel speeds (from EXLAP) */
    float wheel_speed_fl;        /* km/h */
    float wheel_speed_fr;        /* km/h */
    float wheel_speed_rl;        /* km/h */
    float wheel_speed_rr;        /* km/h */
    
    /* === Tire Temperatures (from TireX) === */
    /* Each tire has 4 sensor readings */
    float tire_temp_lf[4];       /* °C */
    float tire_temp_rf[4];       /* °C */
    float tire_temp_lr[4];       /* °C */
    float tire_temp_rr[4];       /* °C */
    
    /* === Tire Pressures (from TireX, if available) === */
    float tire_pressure_lf;      /* bar */
    float tire_pressure_rf;      /* bar */
    float tire_pressure_lr;      /* bar */
    float tire_pressure_rr;      /* bar */
    
    /* === Derived Values === */
    float avg_temp_lf;           /* °C */
    float avg_temp_rf;           /* °C */
    float avg_temp_lr;           /* °C */
    float avg_temp_rr;           /* °C */
    float max_temp;              /* °C — highest of all sensors */
    float delta_temp_lf;         /* °C — max-min differential */
    float delta_temp_rf;         /* °C */
    float delta_temp_lr;         /* °C */
    float delta_temp_rr;         /* °C */
    
    /* === Statistics === */
    uint32_t tirex_update_count; /* Number of TireX updates */
    uint32_t exlap_update_count; /* Number of EXLAP updates */
    uint32_t last_tirex_ms;      /* ms since last TireX update */
    uint32_t last_exlap_ms;      /* ms since last EXLAP update */

    /* === TireX configuration summary (mirrored from tirex_config) === */
    uint8_t tirex_sensor_count;
    telemetry_tirex_sensor_t tirex_sensors[TIREX_MAX_SENSORS];
    uint32_t tirex_config_update_count;
    uint32_t last_tirex_config_ms;

    /* Data validity flags */
    bool has_tirex_data;
    bool has_exlap_data;
} telemetry_t;

/**
 * Initialize the telemetry database
 * 
 * @return true on success
 */
bool telemetry_db_init(void);

/**
 * Update telemetry from TireX data
 * 
 * @param temps Pointer to tirex_data_t with temperature readings
 * @return true on success
 */
bool telemetry_db_update_tirex(const float lf[4], const float rf[4],
                               const float lr[4], const float rr[4]);

/**
 * Mirror a confirmed TireX sensor configuration into the telemetry database.
 * This is called after the sensor confirms a config change on the bus.
 */
bool telemetry_db_update_tirex_config(const tirex_sensor_snapshot_t *sensor);

/**
 * Update telemetry from EXLAP data
 * 
 * @param telemetry Pointer to exlap_telemetry_t with vehicle data
 * @return true on success
 */
bool telemetry_db_update_exlap(float speed, float rpm, float throttle,
                               float brake, float steering, float yaw,
                               int8_t gear,
                               float ws_fl, float ws_fr, float ws_rl, float ws_rr,
                               float tire_pressure_fl, float tire_pressure_fr,
                               float tire_pressure_rl, float tire_pressure_rr);

/**
 * Get the current telemetry snapshot (thread-safe)
 * 
 * @param telemetry Pointer to telemetry_t to fill
 * @return true on success
 */
bool telemetry_db_get_snapshot(telemetry_t *telemetry);

/**
 * Format telemetry as JSON string for web dashboard
 * 
 * @param json_buf Buffer to receive JSON string
 * @param json_buf_len Max buffer length
 * @return true on success
 */
bool telemetry_db_to_json(char *json_buf, size_t json_buf_len);

/**
 * Reset all telemetry data
 */
void telemetry_db_reset(void);

/**
 * Get telemetry update rate (Hz)
 * 
 * @return Updates per second (0 if no updates)
 */
float telemetry_db_get_update_rate(void);

#endif /* TELEMETRY_DB_H */
