/**
 * TireX CAN Decoder
 * 
 * Decodes TireX temperature data from CAN frames for debug/diagnostics
 * 
 * TireX Protocol:
 * - 0xA120: LF Tire Temps (4 sensors)
 * - 0xA220: RF Tire Temps (4 sensors)
 * - 0xA320: LR Tire Temps (4 sensors)
 * - 0xA420: RR Tire Temps (4 sensors)
 * 
 * Temperature (°C) = Raw Value × 0.5
 */

#ifndef TIREX_DECODER_H
#define TIREX_DECODER_H

#include <stdint.h>

/* TireX CAN IDs */
#define TIREX_LF_ID   0xA120
#define TIREX_RF_ID   0xA220
#define TIREX_LR_ID   0xA320
#define TIREX_RR_ID   0xA420

/**
 * Tire temperature data structure
 */
typedef struct {
    float temp1; /* °C */
    float temp2; /* °C */
    float temp3; /* °C */
    float temp4; /* °C */
} tire_temps_t;

/**
 * Full TireX data structure
 */
typedef struct {
    tire_temps_t lf; /* Left Front */
    tire_temps_t rf; /* Right Front */
    tire_temps_t lr; /* Left Rear */
    tire_temps_t rr; /* Right Rear */
    uint32_t timestamp_ms; /* Timestamp of last update */
} tirex_data_t;

/**
 * Initialize the TireX decoder
 * 
 * @return true on success
 */
bool tirex_decoder_init(void);

/**
 * Decode a TireX CAN frame
 * 
 * @param can_id CAN identifier
 * @param data Pointer to 8-byte payload
 * @param tirex Pointer to tirex_data_t to update
 * @return true if frame was decoded, false if unknown ID
 */
bool tirex_decoder_process(uint32_t can_id, const uint8_t *data, tirex_data_t *tirex);

/**
 * Print decoded TireX data to serial console
 * 
 * @param tirex Pointer to tirex_data_t
 */
void tirex_decoder_print(const tirex_data_t *tirex);

#endif /* TIREX_DECODER_H */
