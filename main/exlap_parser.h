/**
 * EXLAP Parser
 * 
 * Parses XML-style EXLAP telemetry packets into structured data.
 * - Lightweight custom parser (no dynamic allocation)
 * - Streaming packet processing
 * - Maps EXLAP signals to internal telemetry channels
 * - Configurable debug logging via CONFIG_EXLAP_PARSER_DEBUG_LOGGING
 * 
 * EXLAP protocol uses XML-style tags like:
 *   <vehicleSpeed>102.4</vehicleSpeed>
 *   <engineSpeed>4300</engineSpeed>
 */

#ifndef EXLAP_PARSER_H
#define EXLAP_PARSER_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/**
 * Parsed EXLAP telemetry data
 * 
 * Each field has a corresponding validity flag so the telemetry DB
 * knows which values were actually present in the packet and which
 * are stale from a previous parse. The parser consumes EXLAP `Dat`
 * payloads and may aggregate multiple completed `Dat` messages from
 * one receive buffer.
 */
typedef struct {
    /* Vehicle dynamics */
    float vehicle_speed;        /* km/h */
    float engine_speed;         /* RPM */
    float accelerator_pos;      /* % (0-100) */
    float brake_pressure;       /* bar */
    float steering_angle;       /* degrees */
    float yaw_rate;             /* deg/s */
    
    /* Wheel speeds */
    float wheel_speed_fl;       /* km/h */
    float wheel_speed_fr;       /* km/h */
    float wheel_speed_rl;       /* km/h */
    float wheel_speed_rr;       /* km/h */

    /* Tire pressures */
    float tire_pressure_fl;     /* bar */
    float tire_pressure_fr;     /* bar */
    float tire_pressure_rl;     /* bar */
    float tire_pressure_rr;     /* bar */
    
    /* Transmission */
    int8_t gear;                /* 0=neutral, 1-8=gears, -1=reverse */
    
    /* Per-field validity — set when tag is present in packet */
    bool has_vehicle_speed;
    bool has_engine_speed;
    bool has_accelerator_pos;
    bool has_brake_pressure;
    bool has_steering_angle;
    bool has_yaw_rate;
    bool has_wheel_speed_fl;
    bool has_wheel_speed_fr;
    bool has_wheel_speed_rl;
    bool has_wheel_speed_rr;
    bool has_tire_pressure_fl;
    bool has_tire_pressure_fr;
    bool has_tire_pressure_rl;
    bool has_tire_pressure_rr;
    bool has_gear;
    
    /* Metadata */
    uint32_t timestamp_ms;      /* Parse timestamp */
    uint32_t packet_count;      /* Number of packets parsed */
} exlap_telemetry_t;

/**
 * Initialize the EXLAP parser
 * 
 * @return true on success
 */
bool exlap_parser_init(void);

/**
 * Parse an EXLAP telemetry packet
 * 
 * Processes EXLAP XML-style `Dat` telemetry payloads and updates the
 * telemetry struct.
 * Uses a lightweight state-machine parser with no dynamic allocation.
 * 
 * Only fields whose tags appear in the packet are updated; validity
 * flags indicate which values are fresh.
 * 
 * @param data      Pointer to packet data
 * @param len       Length of packet data
 * @param telemetry Pointer to exlap_telemetry_t to update
 * @return true if a complete packet was parsed, false otherwise
 */
bool exlap_parser_parse(const uint8_t *data, size_t len,
                        exlap_telemetry_t *telemetry);

/**
 * Reset parser state
 * 
 * Clears any buffered partial data. Call this on TCP reconnect or
 * after a parse error to avoid carrying stale fragments into the
 * next message.
 */
void exlap_parser_reset(void);

/**
 * Print parsed telemetry to serial console
 * 
 * @param telemetry Pointer to exlap_telemetry_t
 */
void exlap_parser_print(const exlap_telemetry_t *telemetry);

#endif /* EXLAP_PARSER_H */
