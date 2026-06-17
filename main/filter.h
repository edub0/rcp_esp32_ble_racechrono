/**
 * RaceChrono CAN Filter Implementation
 * 
 * Handles filter commands from RaceChrono:
 * - Command 0: Deny All
 * - Command 1: Allow All
 * - Command 2: Allow Specific PID (with interval and CAN ID)
 */

#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>
#include <stdbool.h>

/* Maximum number of filter rules.
 *
 * RaceChrono can subscribe to more than 16 distinct PIDs during a live
 * session, so keep some headroom above the observed client request set.
 */
#define MAX_FILTER_RULES  32

/**
 * Filter rule structure
 */
typedef struct {
    uint32_t pid;         /* CAN ID to filter */
    uint16_t interval_ms; /* Transmission interval in ms */
    uint32_t last_sent_ms; /* Timestamp of last transmission */
} filter_rule_t;

/**
 * Initialize the filter module
 * 
 * @return true on success
 */
bool filter_init(void);

/**
 * Process a filter command from RaceChrono
 * 
 * @param data Pointer to command data
 * @param len Length of command data
 */
void filter_process(const uint8_t *data, uint16_t len);

/**
 * Check if a CAN frame should be forwarded
 * 
 * @param can_id 32-bit CAN identifier
 * @return true if frame should be forwarded, false otherwise
 */
bool filter_should_forward(uint32_t can_id);

/**
 * Reset all filters to default state (deny all)
 */
void filter_reset(void);

#endif /* FILTER_H */
