/**
 * WiFi Manager
 * 
 * Manages WiFi STA connection for EXLAP gateway.
 * - Stores SSID/password in NVS
 * - Auto-connect on startup
 * - Reconnect on disconnection
 * - Reports signal strength and connection state
 * 
 * State Machine:
 *   DISCONNECTED → CONNECTING → CONNECTED
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_wifi_types.h"
#include <stdbool.h>
#include <stdint.h>

/* WiFi Manager NVS namespace */
#define WIFI_NVS_NAMESPACE "wifi_cfg"

/* Maximum SSID/password length */
#define WIFI_MAX_SSID_LEN   32
#define WIFI_MAX_PASS_LEN   64

/**
 * WiFi connection states
 */
typedef enum {
    WIFI_STATE_DISCONNECTED,
    WIFI_STATE_CONNECTING,
    WIFI_STATE_CONNECTED,
} wifi_state_t;

/**
 * WiFi status information
 */
typedef struct {
    wifi_state_t state;
    int8_t rssi;                       /* Signal strength in dBm */
    uint8_t channel;                   /* Connected channel */
    wifi_auth_mode_t auth_mode;        /* Authentication mode */
    char ssid[WIFI_MAX_SSID_LEN + 1];  /* Current SSID */
} wifi_status_t;

/**
 * Initialize the WiFi manager
 * 
 * Sets up NVS, WiFi event handler, and attempts auto-connect
 * if credentials are stored. When the troubleshooting AP/UI build
 * flag is enabled, the gateway SoftAP is also started so the local
 * web UI remains available.
 * 
 * @return true on success, false on failure
 */
bool wifi_manager_init(void);

/**
 * Stop WiFi and free resources
 */
void wifi_manager_stop(void);

/**
 * Set WiFi credentials and connect
 * 
 * @param ssid     WiFi network name
 * @param password WiFi password (NULL for open networks)
 * @return true if connection initiated, false on error
 */
bool wifi_manager_connect(const char *ssid, const char *password);

/**
 * Disconnect from current WiFi network
 */
void wifi_manager_disconnect(void);

/**
 * Get current WiFi status
 * 
 * @param status Pointer to wifi_status_t to fill
 */
void wifi_manager_get_status(wifi_status_t *status);

/**
 * Check if WiFi is connected
 * 
 * @return true if connected, false otherwise
 */
bool wifi_manager_is_connected(void);

/**
 * Get current signal strength
 * 
 * @return RSSI in dBm, or 0 if not connected
 */
int8_t wifi_manager_get_rssi(void);

/**
 * Save credentials to NVS for auto-connect on reboot
 * 
 * @param ssid     WiFi network name
 * @param password WiFi password
 * @return true on success, false on failure
 */
bool wifi_manager_save_credentials(const char *ssid, const char *password);

/**
 * Load saved credentials from NVS
 * 
 * @param ssid     Buffer for SSID (must be >= WIFI_MAX_SSID_LEN + 1)
 * @param password Buffer for password (must be >= WIFI_MAX_PASS_LEN + 1)
 * @return true if credentials found, false if not
 */
bool wifi_manager_load_credentials(char *ssid, char *password);

#endif /* WIFI_MANAGER_H */
