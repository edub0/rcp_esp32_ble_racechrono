/**
 * WiFi Manager Implementation
 * 
 * Uses ESP-IDF WiFi API with event callback for state management.
 * Credentials persisted in NVS under "wifi_cfg" namespace.
 */

#include "wifi_manager.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>

static const char *TAG = "wifi_manager";

#if CONFIG_EXLAP_TESTING_AP_UI
/* True while the gateway AP is intentionally paused to give the station
 * radio a cleaner shot at joining the vehicle network. */
static bool s_ap_paused_for_join = false;
#endif

/* WiFi connection state */
static wifi_state_t s_wifi_state = WIFI_STATE_DISCONNECTED;
static int8_t s_rssi = 0;
static uint8_t s_channel = 0;
static wifi_auth_mode_t s_auth_mode = WIFI_AUTH_OPEN;
static char s_ssid[WIFI_MAX_SSID_LEN + 1] = {0};

/* NVS keys */
#define NVS_KEY_SSID     "ssid"
#define NVS_KEY_PASSWORD "password"

/* WiFi event handler */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);

static void strip_wrapping_quotes(char *s)
{
    if (s == NULL) {
        return;
    }

    size_t len = strlen(s);
    if (len >= 6 && strncmp(s, "%27", 3) == 0 && strcmp(s + len - 3, "%27") == 0) {
        memmove(s, s + 3, len - 6);
        s[len - 6] = '\0';
        return;
    }

    if (len >= 6 && strncmp(s, "%22", 3) == 0 && strcmp(s + len - 3, "%22") == 0) {
        memmove(s, s + 3, len - 6);
        s[len - 6] = '\0';
        return;
    }

    if (len >= 2) {
        char first = s[0];
        char last = s[len - 1];
        if ((first == '"' && last == '"') || (first == '\'' && last == '\'')) {
            memmove(s, s + 1, len - 2);
            s[len - 2] = '\0';
        }
    }
}

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return -1;
}

static void url_decode_inplace(char *s)
{
    if (s == NULL) {
        return;
    }

    char *src = s;
    char *dst = s;

    while (*src != '\0') {
        if (*src == '%' && src[1] != '\0' && src[2] != '\0') {
            int hi = hex_value(src[1]);
            int lo = hex_value(src[2]);
            if (hi >= 0 && lo >= 0) {
                *dst++ = (char)((hi << 4) | lo);
                src += 3;
                continue;
            }
        } else if (*src == '+') {
            *dst++ = ' ';
            src++;
            continue;
        }

        *dst++ = *src++;
    }

    *dst = '\0';
}

static const char *wifi_mode_to_str(wifi_mode_t mode)
{
    switch (mode) {
    case WIFI_MODE_NULL: return "null";
    case WIFI_MODE_STA: return "sta";
    case WIFI_MODE_AP: return "ap";
    case WIFI_MODE_APSTA: return "apsta";
    default: return "unknown";
    }
}

static void wifi_log_radio_state(const char *context)
{
    wifi_mode_t mode = WIFI_MODE_NULL;
    uint8_t primary = 0;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;

    if (esp_wifi_get_mode(&mode) != ESP_OK) {
        ESP_LOGW(TAG, "%s: unable to read WiFi mode", context);
        return;
    }

    if (esp_wifi_get_channel(&primary, &second) != ESP_OK) {
        ESP_LOGW(TAG, "%s: mode=%s channel=unknown", context, wifi_mode_to_str(mode));
        return;
    }

    ESP_LOGI(TAG, "%s: mode=%s primary_channel=%u secondary=%d",
             context, wifi_mode_to_str(mode), primary, second);
}

static void wifi_start_access_point(void)
{
#if CONFIG_EXLAP_TESTING_AP_UI
    static const char *ap_ssid = "EXLAP-Gateway";
    static const char *ap_password = "exlapgw1";
    static const uint8_t ap_channel = 11;
    wifi_config_t ap_config = {0};

    strncpy((char *)ap_config.ap.ssid, ap_ssid, sizeof(ap_config.ap.ssid) - 1);
    strncpy((char *)ap_config.ap.password, ap_password, sizeof(ap_config.ap.password) - 1);
    ap_config.ap.channel = ap_channel;
    ap_config.ap.max_connection = 4;
    ap_config.ap.ssid_len = 0;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap_config.ap.pmf_cfg.capable = false;
    ap_config.ap.pmf_cfg.required = false;
    ap_config.ap.beacon_interval = 100;

    if (strlen(ap_password) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
        ap_config.ap.pmf_cfg.capable = false;
        ap_config.ap.pmf_cfg.required = false;
    }

    esp_err_t ret = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure softAP: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "SoftAP configured: ssid=%s channel=%u auth=%d pmf_capable=%d pmf_required=%d beacon=%u password=%s",
             ap_ssid,
             ap_channel,
             ap_config.ap.authmode,
             ap_config.ap.pmf_cfg.capable,
             ap_config.ap.pmf_cfg.required,
             ap_config.ap.beacon_interval,
             ap_password);
    wifi_log_radio_state("SoftAP configured");
#else
    ESP_LOGI(TAG, "Troubleshooting SoftAP/web UI disabled by build flag");
#endif
}

static void wifi_log_access_point_info(void)
{
#if CONFIG_EXLAP_TESTING_AP_UI
    esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    esp_netif_ip_info_t ip = {0};

    if (ap_netif != NULL && esp_netif_get_ip_info(ap_netif, &ip) == ESP_OK) {
        ESP_LOGI(TAG, "SoftAP ready: ssid=%s ip=" IPSTR, "EXLAP-Gateway", IP2STR(&ip.ip));
    } else {
        ESP_LOGI(TAG, "SoftAP ready: ssid=%s ip=192.168.4.1", "EXLAP-Gateway");
    }

    wifi_log_radio_state("SoftAP ready");
#endif
}

static bool wifi_pause_access_point_for_join(void)
{
#if CONFIG_EXLAP_TESTING_AP_UI
    if (s_ap_paused_for_join) {
        return true;
    }

    esp_err_t ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to pause SoftAP for STA join: %s", esp_err_to_name(ret));
        return false;
    }

    s_ap_paused_for_join = true;
    ESP_LOGI(TAG, "SoftAP paused for STA join");
    wifi_log_radio_state("AP paused");
    return true;
#else
    return true;
#endif
}

static bool wifi_resume_access_point_after_join(void)
{
#if CONFIG_EXLAP_TESTING_AP_UI
    if (!s_ap_paused_for_join) {
        return true;
    }

    esp_err_t ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to resume SoftAP after STA join: %s", esp_err_to_name(ret));
        return false;
    }

    wifi_start_access_point();
    wifi_log_access_point_info();
    s_ap_paused_for_join = false;
    ESP_LOGI(TAG, "SoftAP resumed after STA join");
    return true;
#else
    return true;
#endif
}

/**
 * Initialize WiFi internals (TCP/IP, WiFi driver, event handlers)
 */
static bool wifi_internal_init(void)
{
    /* Initialize TCP/IP */
    ESP_ERROR_CHECK(esp_netif_init());

    /* Create the default event loop used by WiFi/IP handlers */
    esp_err_t ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create default event loop: %s", esp_err_to_name(ret));
        return false;
    }

    /* Create TCP/IP interface */
    esp_netif_create_default_wifi_sta();
#if CONFIG_EXLAP_TESTING_AP_UI
    esp_netif_create_default_wifi_ap();
#endif

    /* WiFi config */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    /* Initialize WiFi driver */
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(ret));
        return false;
    }

#if CONFIG_EXLAP_TESTING_AP_UI
    ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
#else
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
#endif
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return false;
    }

    /* Keep the station radio fully awake during joins to reduce WPA2
     * handshake and coexistence timing issues on the vehicle AP. */
    ret = esp_wifi_set_ps(WIFI_PS_NONE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to disable WiFi power save: %s", esp_err_to_name(ret));
    }

#if CONFIG_EXLAP_TESTING_AP_UI
    wifi_start_access_point();
#endif

    /* Register event handlers */
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    /* Start WiFi */
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi start failed: %s", esp_err_to_name(ret));
        return false;
    }

    wifi_log_access_point_info();

    return true;
}

/**
 * WiFi event handler
 * 
 * Handles connection state transitions and IP acquisition
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "WiFi STA started");
            wifi_log_radio_state("STA start");
            break;

        case WIFI_EVENT_STA_CONNECTED: {
            wifi_event_sta_connected_t *event =
                (wifi_event_sta_connected_t *)event_data;

            s_channel = event->channel;
            s_auth_mode = event->authmode;
            strncpy(s_ssid, (const char *)event->ssid, WIFI_MAX_SSID_LEN);
            s_ssid[WIFI_MAX_SSID_LEN] = '\0';

            /* Get RSSI */
            int rssi_val = 0;
            esp_wifi_sta_get_rssi(&rssi_val);
            s_rssi = (int8_t)rssi_val;

            s_wifi_state = WIFI_STATE_CONNECTED;
            ESP_LOGI(TAG, "Connected to \"%s\" (channel=%d, auth=%d)",
                     s_ssid, s_channel, s_auth_mode);
            wifi_log_radio_state("STA connected");
            break;
        }

        case WIFI_EVENT_STA_DISCONNECTED:
        {
            wifi_event_sta_disconnected_t *event =
                (wifi_event_sta_disconnected_t *)event_data;
            s_wifi_state = WIFI_STATE_DISCONNECTED;
            s_rssi = 0;
            ESP_LOGW(TAG, "STA disconnected (reason=%d); manual reconnect required",
                     event ? event->reason : -1);
            wifi_log_radio_state("STA disconnected");
            if (!wifi_resume_access_point_after_join()) {
                ESP_LOGW(TAG, "Failed to restore SoftAP after STA disconnect");
            }
            break;
        }

        case WIFI_EVENT_AP_START:
            ESP_LOGI(TAG, "SoftAP started");
            wifi_log_radio_state("AP started");
            break;

        case WIFI_EVENT_AP_STOP:
            ESP_LOGI(TAG, "SoftAP stopped");
            break;

        case WIFI_EVENT_AP_STACONNECTED: {
            wifi_event_ap_staconnected_t *event =
                (wifi_event_ap_staconnected_t *)event_data;
            if (event != NULL) {
                ESP_LOGI(TAG, "SoftAP client connected: aid=%d, mac=%02x:%02x:%02x:%02x:%02x:%02x",
                         event->aid,
                         event->mac[0], event->mac[1], event->mac[2],
                         event->mac[3], event->mac[4], event->mac[5]);
            } else {
                ESP_LOGI(TAG, "SoftAP client connected");
            }
            wifi_log_radio_state("AP client connected");
            break;
        }

        case WIFI_EVENT_AP_STADISCONNECTED: {
            wifi_event_ap_stadisconnected_t *event =
                (wifi_event_ap_stadisconnected_t *)event_data;
            if (event != NULL) {
                ESP_LOGI(TAG, "SoftAP client disconnected: aid=%d, mac=%02x:%02x:%02x:%02x:%02x:%02x",
                         event->aid,
                         event->mac[0], event->mac[1], event->mac[2],
                         event->mac[3], event->mac[4], event->mac[5]);
            } else {
                ESP_LOGI(TAG, "SoftAP client disconnected");
            }
            wifi_log_radio_state("AP client disconnected");
            break;
        }

        case WIFI_EVENT_SCAN_DONE:
            break;

        default:
            break;
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
            ESP_LOGI(TAG, "Got IP: " IPSTR,
                     IP2STR(&event->ip_info.ip));
            if (!wifi_resume_access_point_after_join()) {
                ESP_LOGW(TAG, "Failed to restore SoftAP after STA got IP");
            }
        }
    }
}

bool wifi_manager_init(void)
{
    ESP_LOGI(TAG, "Initializing WiFi manager");

    if (!wifi_internal_init()) {
        return false;
    }

    /* Load saved credentials and auto-connect at boot so the vehicle WiFi
     * comes up without requiring a manual web UI join. */
    char ssid[WIFI_MAX_SSID_LEN + 1] = {0};
    char password[WIFI_MAX_PASS_LEN + 1] = {0};

    if (wifi_manager_load_credentials(ssid, password)) {
        ESP_LOGI(TAG, "Saved credentials found; auto-connecting on startup");
        if (!wifi_manager_connect(ssid, password)) {
            ESP_LOGW(TAG, "Auto-connect to saved WiFi failed");
        }
    } else {
        ESP_LOGI(TAG, "No saved credentials, WiFi idle");
    }

    return true;
}

void wifi_manager_stop(void)
{
    wifi_manager_disconnect();
    esp_wifi_stop();
    esp_wifi_deinit();
    ESP_LOGI(TAG, "WiFi stopped");
}

bool wifi_manager_connect(const char *ssid, const char *password)
{
    if (ssid == NULL || strlen(ssid) == 0) {
        ESP_LOGE(TAG, "Invalid SSID");
        return false;
    }

    if (strlen(ssid) > WIFI_MAX_SSID_LEN) {
        ESP_LOGE(TAG, "SSID too long (max %d)", WIFI_MAX_SSID_LEN);
        return false;
    }

    char clean_ssid[WIFI_MAX_SSID_LEN + 1] = {0};
    char clean_password[WIFI_MAX_PASS_LEN + 1] = {0};
    strncpy(clean_ssid, ssid, WIFI_MAX_SSID_LEN);
    clean_ssid[WIFI_MAX_SSID_LEN] = '\0';
    if (password != NULL) {
        strncpy(clean_password, password, WIFI_MAX_PASS_LEN);
        clean_password[WIFI_MAX_PASS_LEN] = '\0';
    }
    strip_wrapping_quotes(clean_ssid);
    strip_wrapping_quotes(clean_password);

    if (!wifi_pause_access_point_for_join()) {
        return false;
    }

    s_wifi_state = WIFI_STATE_CONNECTING;

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, clean_ssid, WIFI_MAX_SSID_LEN);
    wifi_config.sta.ssid[WIFI_MAX_SSID_LEN] = '\0';

    if (strlen(clean_password) > 0) {
        strncpy((char *)wifi_config.sta.password, clean_password, WIFI_MAX_PASS_LEN);
        wifi_config.sta.password[WIFI_MAX_PASS_LEN] = '\0';
    }
    wifi_config.sta.bssid_set = false;
    wifi_config.sta.listen_interval = 0;
    wifi_config.sta.pmf_cfg.capable = false;
    wifi_config.sta.pmf_cfg.required = false;

    /* Set WiFi configuration */
    esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(ret));
        s_wifi_state = WIFI_STATE_DISCONNECTED;
        wifi_resume_access_point_after_join();
        return false;
    }

    /* Connect */
    ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect: %s", esp_err_to_name(ret));
        s_wifi_state = WIFI_STATE_DISCONNECTED;
        wifi_resume_access_point_after_join();
        return false;
    }

    ESP_LOGI(TAG, "Connecting to \"%s\" with default AP selection...", clean_ssid);
    return true;
}

void wifi_manager_disconnect(void)
{
    if (s_wifi_state == WIFI_STATE_DISCONNECTED) {
#if CONFIG_EXLAP_TESTING_AP_UI
        if (s_ap_paused_for_join) {
            wifi_resume_access_point_after_join();
        }
#endif
        return;
    }

    esp_wifi_disconnect();
    s_wifi_state = WIFI_STATE_DISCONNECTED;
#if CONFIG_EXLAP_TESTING_AP_UI
    if (s_ap_paused_for_join) {
        wifi_resume_access_point_after_join();
    }
#endif
    ESP_LOGI(TAG, "WiFi disconnect requested");
}

void wifi_manager_get_status(wifi_status_t *status)
{
    if (status == NULL) return;

    status->state = s_wifi_state;
    status->rssi = s_rssi;
    status->channel = s_channel;
    status->auth_mode = s_auth_mode;
    strncpy(status->ssid, s_ssid, WIFI_MAX_SSID_LEN);
    status->ssid[WIFI_MAX_SSID_LEN] = '\0';
}

bool wifi_manager_is_connected(void)
{
    return (s_wifi_state == WIFI_STATE_CONNECTED);
}

int8_t wifi_manager_get_rssi(void)
{
    return s_rssi;
}

bool wifi_manager_save_credentials(const char *ssid, const char *password)
{
    if (ssid == NULL) return false;

    char clean_ssid[WIFI_MAX_SSID_LEN + 1] = {0};
    char clean_password[WIFI_MAX_PASS_LEN + 1] = {0};
    strncpy(clean_ssid, ssid, WIFI_MAX_SSID_LEN);
    clean_ssid[WIFI_MAX_SSID_LEN] = '\0';
    if (password != NULL) {
        strncpy(clean_password, password, WIFI_MAX_PASS_LEN);
        clean_password[WIFI_MAX_PASS_LEN] = '\0';
    }
    url_decode_inplace(clean_ssid);
    url_decode_inplace(clean_password);
    strip_wrapping_quotes(clean_ssid);
    strip_wrapping_quotes(clean_password);

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return false;
    }

    ret = nvs_set_str(handle, NVS_KEY_SSID, clean_ssid);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save SSID: %s", esp_err_to_name(ret));
        nvs_close(handle);
        return false;
    }

    if (strlen(clean_password) > 0) {
        ret = nvs_set_str(handle, NVS_KEY_PASSWORD, clean_password);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save password: %s", esp_err_to_name(ret));
            nvs_close(handle);
            return false;
        }
    } else {
        nvs_erase_key(handle, NVS_KEY_PASSWORD);
    }

    ret = nvs_commit(handle);
    nvs_close(handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "WiFi credentials saved to NVS");
    return true;
}

bool wifi_manager_load_credentials(char *ssid, char *password)
{
    if (ssid == NULL || password == NULL) return false;

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(WIFI_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No saved WiFi credentials found");
        return false;
    }

    size_t ssid_len = sizeof(char) * (WIFI_MAX_SSID_LEN + 1);
    size_t pass_len = sizeof(char) * (WIFI_MAX_PASS_LEN + 1);

    ret = nvs_get_str(handle, NVS_KEY_SSID, ssid, &ssid_len);
    if (ret != ESP_OK) {
        nvs_close(handle);
        return false;
    }

    /* Password is optional */
    nvs_get_str(handle, NVS_KEY_PASSWORD, password, &pass_len);

    /* NVS stores plain text credentials. Do not URL-decode here or a literal
     * '+' in the password would be converted into a space on load. */
    strip_wrapping_quotes(ssid);
    strip_wrapping_quotes(password);

    nvs_close(handle);
    ESP_LOGI(TAG, "Loaded credentials for SSID: \"%s\"", ssid);
    return true;
}
