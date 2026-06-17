/**
 * EXLAP Client
 * 
 * Manages TCP connection to vehicle EXLAP telemetry service.
 * - Connect/disconnect to vehicle IP:port
 * - Authenticate with the EXLAP challenge/response flow
 * - Subscribe to telemetry channels
 * - Send heartbeat requests to keep the session alive
 * - Receive and queue incoming telemetry packets
 * 
 * Runs as a FreeRTOS task on Core 0.
 */

#ifndef EXLAP_CLIENT_H
#define EXLAP_CLIENT_H

#include "freertos/FreeRTOS.h"
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/* EXLAP client NVS namespace */
#define EXLAP_NVS_NAMESPACE "exlap_cfg"

/* Maximum IP address string length */
#define EXLAP_MAX_IP_LEN 16

/* Maximum auth credential lengths */
#define EXLAP_MAX_USER_LEN 32
#define EXLAP_MAX_PASS_LEN 64

/* Maximum telemetry packet buffer size */
#define EXLAP_MAX_PACKET_SIZE 1024

/* Receive queue depth */
#define EXLAP_QUEUE_DEPTH 64

/**
 * EXLAP connection states
 */
typedef enum {
    EXLAP_STATE_DISCONNECTED,
    EXLAP_STATE_CONNECTING,
    EXLAP_STATE_AUTHENTICATING,
    EXLAP_STATE_SUBSCRIBING,
    EXLAP_STATE_AUTHENTICATED,
} exlap_state_t;

/**
 * EXLAP configuration
 */
typedef struct {
    char vehicle_ip[EXLAP_MAX_IP_LEN + 1];
    uint16_t port;
    bool enabled;
    char username[EXLAP_MAX_USER_LEN + 1];
    char password[EXLAP_MAX_PASS_LEN + 1];
} exlap_config_t;

/**
 * EXLAP status information
 */
typedef struct {
    exlap_state_t state;
    bool authenticated;
    uint32_t packets_received;
    uint32_t packets_dropped;
    uint32_t bytes_received;
    uint32_t last_packet_ms; /* Timestamp of last received packet */
    uint32_t last_auth_ms;
    uint32_t last_heartbeat_ms;
} exlap_status_t;

/**
 * Initialize the EXLAP client module
 * 
 * Sets up NVS defaults, creates receive queue.
 * Does NOT connect yet — call exlap_client_connect().
 * 
 * @return true on success, false on failure
 */
bool exlap_client_init(void);

/**
 * Stop the EXLAP client
 * 
 * Disconnects TCP, deletes task, frees resources.
 */
void exlap_client_stop(void);

/**
 * Connect to the vehicle EXLAP service
 * 
 * Uses stored configuration or provided parameters.
 * 
 * @param ip     Vehicle IP address (NULL to use stored config)
 * @param port   Vehicle EXLAP port (0 to use stored config)
 * @return true if connection initiated, false on error
 */
bool exlap_client_connect(const char *ip, uint16_t port);

/**
 * Disconnect from the vehicle EXLAP service
 */
void exlap_client_disconnect(void);

/**
 * Start the EXLAP client task
 * 
 * Creates the FreeRTOS task for TCP receive/parsing.
 * 
 * @return true on success, false on failure
 */
bool exlap_client_start_task(void);

/**
 * Receive a telemetry packet from the EXLAP receive queue
 * 
 * @param data      Buffer to receive packet data
 * @param len       Pointer to max buffer size (in) / received size (out)
 * @param timeout   FreeRTOS ticks to wait
 * @return true if packet received, false on timeout/error
 */
bool exlap_client_recv_packet(uint8_t *data, size_t *len, TickType_t timeout);

/**
 * Get current EXLAP status
 * 
 * @param status Pointer to exlap_status_t to fill
 */
void exlap_client_get_status(exlap_status_t *status);

/**
 * Check if EXLAP is connected
 * 
 * @return true if connected, false otherwise
 */
bool exlap_client_is_connected(void);

/**
 * Save EXLAP configuration to NVS
 * 
 * @param config Pointer to exlap_config_t to save
 * @return true on success, false on failure
 */
bool exlap_client_save_config(const exlap_config_t *config);

/**
 * Load EXLAP configuration from NVS
 * 
 * @param config Pointer to exlap_config_t to fill
 * @return true if config found, false if defaults used
 */
bool exlap_client_load_config(exlap_config_t *config);

#endif /* EXLAP_CLIENT_H */
