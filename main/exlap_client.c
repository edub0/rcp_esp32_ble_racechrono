/**
 * EXLAP Client Implementation
 * 
 * TCP client for VAG vehicle EXLAP telemetry service.
 * Uses ESP-IDF socket API for non-blocking TCP receive.
 * Packets queued to FreeRTOS queue for parser task.
 */

#include "exlap_client.h"
#include "wifi_manager.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "mbedtls/base64.h"
#include "mbedtls/private/sha256.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "exlap_client";

/* NVS keys */
#define NVS_KEY_IP     "ip"
#define NVS_KEY_PORT   "port"
#define NVS_KEY_ENABLE "enabled"
#define NVS_KEY_USER   "user"
#define NVS_KEY_PASS   "password"

/* Default EXLAP port */
#define EXLAP_DEFAULT_PORT 1337

/* EXLAP auth / heartbeat settings */
#define EXLAP_HEARTBEAT_INTERVAL_MS 2000
#define EXLAP_RECV_TIMEOUT_MS 250
#define EXLAP_AUTH_MAX_RETRIES 3

/* TCP receive buffer size */
#define EXLAP_RX_BUF_SIZE 512

/* Reconnect delay in ms */
#define EXLAP_RECONNECT_DELAY_MS 5000

/* Task configuration */
#define EXLAP_TASK_STACK_SIZE 4096
#define EXLAP_TASK_PRIORITY   4
#define EXLAP_TASK_CORE       0

/* Internal state */
static exlap_state_t s_exlap_state = EXLAP_STATE_DISCONNECTED;
static int s_tcp_socket = -1;
static struct sockaddr_in s_server_addr;
static char s_current_ip[EXLAP_MAX_IP_LEN + 1] = {0};
static uint16_t s_current_port = EXLAP_DEFAULT_PORT;
static char s_current_user[EXLAP_MAX_USER_LEN + 1] = {0};
static char s_current_password[EXLAP_MAX_PASS_LEN + 1] = {0};

/* Packet queue: each item is a dynamically allocated buffer */
static QueueHandle_t s_packet_queue = NULL;

/* Statistics */
static uint32_t s_packets_received = 0;
static uint32_t s_packets_dropped = 0;
static uint32_t s_bytes_received = 0;
static uint32_t s_last_packet_ms = 0;
static uint32_t s_last_auth_ms = 0;
static uint32_t s_last_heartbeat_ms = 0;
static bool s_session_authenticated = false;
static uint32_t s_message_id = 98;

/* Packet buffer for queue items */
typedef struct {
    uint8_t data[EXLAP_MAX_PACKET_SIZE];
    size_t len;
} exlap_packet_t;

static void exlap_flush_packet_queue(void)
{
    if (s_packet_queue == NULL) {
        return;
    }

    exlap_packet_t *packet;
    while (xQueueReceive(s_packet_queue, &packet, 0) == pdPASS) {
        free(packet);
    }
}

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

static uint32_t exlap_next_message_id(void)
{
    if (s_message_id >= 999999998UL) {
        s_message_id = 98;
    }
    s_message_id++;
    return s_message_id;
}

static void exlap_generate_cnonce(char *out, size_t out_len)
{
    uint8_t raw[16];
    uint8_t encoded[32];
    size_t encoded_len = 0;

    for (size_t i = 0; i < sizeof(raw); i++) {
        raw[i] = (uint8_t)(esp_random() & 0xFF);
    }

    if (mbedtls_base64_encode(encoded, sizeof(encoded), &encoded_len,
                              raw, sizeof(raw)) != 0) {
        if (out_len > 0) {
            out[0] = '\0';
        }
        return;
    }

    if (encoded_len >= out_len) {
        encoded_len = out_len - 1;
    }

    memcpy(out, encoded, encoded_len);
    out[encoded_len] = '\0';
}

static bool exlap_build_digest(const char *user, const char *password,
                               const char *nonce, const char *cnonce,
                               char *out, size_t out_len)
{
    if (user == NULL || password == NULL || nonce == NULL || cnonce == NULL ||
        out == NULL || out_len == 0) {
        return false;
    }

    char input[256];
    int written = snprintf(input, sizeof(input), "%s:%s:%s:%s",
                           user, password, nonce, cnonce);
    if (written < 0 || (size_t)written >= sizeof(input)) {
        return false;
    }

    uint8_t digest[32];
    if (mbedtls_sha256((const unsigned char *)input, (size_t)written, digest, 0) != 0) {
        return false;
    }

    size_t encoded_len = 0;
    if (mbedtls_base64_encode((unsigned char *)out, out_len, &encoded_len,
                              digest, sizeof(digest)) != 0) {
        return false;
    }

    if (encoded_len >= out_len) {
        return false;
    }

    out[encoded_len] = '\0';
    return true;
}

static bool exlap_send_raw(const char *msg)
{
    if (msg == NULL || s_tcp_socket < 0) {
        return false;
    }

    size_t len = strlen(msg);
    size_t total = 0;

    while (total < len) {
        ssize_t sent = send(s_tcp_socket, msg + total, len - total, 0);
        if (sent < 0) {
            ESP_LOGE(TAG, "Failed to send EXLAP message: %s", strerror(errno));
            return false;
        }
        if (sent == 0) {
            ESP_LOGE(TAG, "Short EXLAP send");
            return false;
        }

        total += (size_t)sent;
    }

    return true;
}

static bool exlap_send_req_auth_challenge(void)
{
    char msg[128];
    uint32_t id = exlap_next_message_id();

    int written = snprintf(msg, sizeof(msg),
                           "<Req id=\"%lu\"><Authenticate phase=\"challenge\" useHash=\"sha256\"/></Req>",
                           (unsigned long)id);
    if (written < 0 || (size_t)written >= sizeof(msg)) {
        return false;
    }

    return exlap_send_raw(msg);
}

static bool exlap_send_req_auth_response(const char *nonce)
{
    char msg[256];
    char cnonce[32];
    char digest[64];
    uint32_t id = exlap_next_message_id();

    if (strlen(s_current_user) == 0 || strlen(s_current_password) == 0) {
        ESP_LOGE(TAG, "EXLAP auth credentials are not configured");
        return false;
    }

    exlap_generate_cnonce(cnonce, sizeof(cnonce));
    if (cnonce[0] == '\0') {
        ESP_LOGE(TAG, "Failed to generate EXLAP cnonce");
        return false;
    }

    if (!exlap_build_digest(s_current_user, s_current_password, nonce, cnonce,
                            digest, sizeof(digest))) {
        ESP_LOGE(TAG, "Failed to build EXLAP auth digest");
        return false;
    }

    int written = snprintf(msg, sizeof(msg),
                           "<Req id=\"%lu\"><Authenticate phase=\"response\" user=\"%s\" cnonce=\"%s\" digest=\"%s\"/></Req>",
                           (unsigned long)id, s_current_user, cnonce, digest);
    if (written < 0 || (size_t)written >= sizeof(msg)) {
        return false;
    }

    return exlap_send_raw(msg);
}

static bool exlap_send_heartbeat(void)
{
    char msg[96];
    uint32_t id = exlap_next_message_id();

    int written = snprintf(msg, sizeof(msg),
                           "<Req id=\"%lu\"><Alive/></Req>",
                           (unsigned long)id);
    if (written < 0 || (size_t)written >= sizeof(msg)) {
        return false;
    }

    if (!exlap_send_raw(msg)) {
        return false;
    }

    s_last_heartbeat_ms = (uint32_t)(esp_timer_get_time() / 1000);
    ESP_LOGD(TAG, "Sent EXLAP heartbeat");
    return true;
}

static bool exlap_send_subscribe(const char *url, uint16_t interval_ms, uint32_t *id_out)
{
    char msg[192];
    uint32_t id = exlap_next_message_id();

    int written = snprintf(msg, sizeof(msg),
                           "<Req id=\"%lu\"><Subscribe url=\"%s\" ival=\"%u\" content=\"true\" timeStamp=\"true\"/></Req>",
                           (unsigned long)id, url, interval_ms);
    if (written < 0 || (size_t)written >= sizeof(msg)) {
        return false;
    }

    if (id_out != NULL) {
        *id_out = id;
    }

    return exlap_send_raw(msg);
}

static bool exlap_send_subscription_set(bool skip_first)
{
    static const struct {
        const char *url;
        uint16_t ival_ms;
    } s_subscriptions[] = {
        {"vehicleSpeed", 100},
        {"engineSpeed", 100},
        {"acceleratorPosition", 100},
        {"brakePressure", 100},
        {"wheelAngle", 100},
        {"yawRate", 100},
        {"currentGear", 0},
        {"tyrePressures", 0},
        {"espTyreVelocities", 0},
    };

    ESP_LOGI(TAG, "Sending EXLAP subscriptions");
    size_t start = skip_first ? 1 : 0;
    for (size_t i = start; i < sizeof(s_subscriptions) / sizeof(s_subscriptions[0]); i++) {
        if (!exlap_send_subscribe(s_subscriptions[i].url, s_subscriptions[i].ival_ms, NULL)) {
            ESP_LOGE(TAG, "Failed to subscribe to %s", s_subscriptions[i].url);
            return false;
        }
    }

    return true;
}

static void exlap_apply_config(const exlap_config_t *config)
{
    if (config == NULL) {
        return;
    }

    if (strlen(config->vehicle_ip) > 0) {
        strncpy(s_current_ip, config->vehicle_ip, EXLAP_MAX_IP_LEN);
        s_current_ip[EXLAP_MAX_IP_LEN] = '\0';
    }

    if (config->port > 0) {
        s_current_port = config->port;
    }

    strncpy(s_current_user, config->username, EXLAP_MAX_USER_LEN);
    s_current_user[EXLAP_MAX_USER_LEN] = '\0';
    strip_wrapping_quotes(s_current_user);

    strncpy(s_current_password, config->password, EXLAP_MAX_PASS_LEN);
    s_current_password[EXLAP_MAX_PASS_LEN] = '\0';
    strip_wrapping_quotes(s_current_password);
}

static bool exlap_packet_contains_dat(const uint8_t *buf, size_t len)
{
    return (buf != NULL && len > 0 &&
            find_str_in_range((const char *)buf, len, "<Dat") != NULL);
}

static bool exlap_extract_challenge_nonce(const uint8_t *buf, size_t len,
                                          char *nonce, size_t nonce_len)
{
    if (buf == NULL || nonce == NULL || nonce_len == 0 || len == 0) {
        return false;
    }

    const char *text = (const char *)buf;
    const char *challenge = find_str_in_range(text, len, "Challenge");
    if (challenge == NULL) {
        return false;
    }

    const char *nonce_attr = find_str_in_range(challenge,
                                               len - (size_t)(challenge - text),
                                               "nonce=\"");
    if (nonce_attr == NULL) {
        return false;
    }

    nonce_attr += strlen("nonce=\"");
    const char *end = find_str_in_range(nonce_attr,
                                        len - (size_t)(nonce_attr - text), "\"");
    if (end == NULL) {
        return false;
    }

    size_t copy_len = (size_t)(end - nonce_attr);
    if (copy_len >= nonce_len) {
        copy_len = nonce_len - 1;
    }

    memcpy(nonce, nonce_attr, copy_len);
    nonce[copy_len] = '\0';
    return true;
}

static bool exlap_extract_rsp_id(const uint8_t *buf, size_t len, uint32_t *id_out)
{
    if (buf == NULL || id_out == NULL || len == 0) {
        return false;
    }

    const char *text = (const char *)buf;
    const char *rsp = find_str_in_range(text, len, "<Rsp");
    if (rsp == NULL) {
        return false;
    }

    const char *id_attr = find_str_in_range(rsp, len - (size_t)(rsp - text), "id=\"");
    if (id_attr == NULL) {
        return false;
    }

    id_attr += strlen("id=\"");
    const char *end = find_str_in_range(id_attr, len - (size_t)(id_attr - text), "\"");
    if (end == NULL) {
        return false;
    }

    char id_buf[16];
    size_t copy_len = (size_t)(end - id_attr);
    if (copy_len >= sizeof(id_buf)) {
        copy_len = sizeof(id_buf) - 1;
    }

    memcpy(id_buf, id_attr, copy_len);
    id_buf[copy_len] = '\0';

    char *parse_end = NULL;
    unsigned long id = strtoul(id_buf, &parse_end, 10);
    if (parse_end == id_buf) {
        return false;
    }

    *id_out = (uint32_t)id;
    return true;
}

static bool exlap_rsp_indicates_ok(const uint8_t *buf, size_t len)
{
    if (buf == NULL || len == 0) {
        return false;
    }

    const char *text = (const char *)buf;
    const char *rsp = find_str_in_range(text, len, "<Rsp");
    if (rsp == NULL) {
        return false;
    }

    const char *status = find_str_in_range(rsp, len - (size_t)(rsp - text), "status=\"");
    if (status == NULL) {
        return true;
    }

    status += strlen("status=\"");
    const char *end = find_str_in_range(status, len - (size_t)(status - text), "\"");
    if (end == NULL) {
        return false;
    }

    char status_buf[32];
    size_t copy_len = (size_t)(end - status);
    if (copy_len >= sizeof(status_buf)) {
        copy_len = sizeof(status_buf) - 1;
    }

    memcpy(status_buf, status, copy_len);
    status_buf[copy_len] = '\0';

    return (strcmp(status_buf, "ok") == 0);
}

static bool exlap_wait_for_rsp(uint32_t expected_id, TickType_t timeout)
{
    uint8_t rx_buf[EXLAP_RX_BUF_SIZE + 1];
    TickType_t waited = 0;

    while (waited < timeout) {
        int ret = recv(s_tcp_socket, rx_buf, EXLAP_RX_BUF_SIZE, 0);
        if (ret > 0) {
            rx_buf[ret] = '\0';

            uint32_t rsp_id = 0;
            if (exlap_extract_rsp_id(rx_buf, (size_t)ret, &rsp_id)) {
                if (rsp_id != expected_id) {
                    /* Unrelated response. Ignore it and keep waiting. */
                    continue;
                }

                if (!exlap_rsp_indicates_ok(rx_buf, (size_t)ret)) {
                    ESP_LOGE(TAG, "EXLAP response %lu was not ok",
                             (unsigned long)rsp_id);
                    return false;
                }

                return true;
            }

            if (strstr((const char *)rx_buf, "authenticationFailed") != NULL) {
                ESP_LOGE(TAG, "EXLAP authentication failed");
                return false;
            }
        } else if (ret == 0) {
            ESP_LOGW(TAG, "EXLAP peer closed connection during response wait");
            return false;
        } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
            ESP_LOGE(TAG, "EXLAP response recv error: %s", strerror(errno));
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
        waited += pdMS_TO_TICKS(50);
    }

    return false;
}

static bool exlap_establish_session(void)
{
    char rx_buf[EXLAP_RX_BUF_SIZE + 1];
    char nonce[128] = {0};

    s_exlap_state = EXLAP_STATE_AUTHENTICATING;
    s_session_authenticated = false;

    for (int attempt = 0; attempt < EXLAP_AUTH_MAX_RETRIES; attempt++) {
        ESP_LOGI(TAG, "Sending EXLAP auth challenge (attempt %d/%d)",
                 attempt + 1, EXLAP_AUTH_MAX_RETRIES);

        if (!exlap_send_req_auth_challenge()) {
            return false;
        }

        /* Match the reference client: send the challenge request, then
         * wait for the server nonce challenge response. */
        for (int wait_round = 0; wait_round < 8; wait_round++) {
            int ret = recv(s_tcp_socket, rx_buf, EXLAP_RX_BUF_SIZE, 0);

            if (ret > 0) {
                rx_buf[ret] = '\0';

                if (strstr(rx_buf, "authenticationFailed") != NULL) {
                    ESP_LOGE(TAG, "EXLAP authentication failed");
                    return false;
                }

                if (exlap_extract_challenge_nonce((const uint8_t *)rx_buf,
                                                  (size_t)ret, nonce, sizeof(nonce))) {
                    goto got_nonce;
                }
            } else if (ret == 0) {
                ESP_LOGW(TAG, "EXLAP peer closed connection during auth");
                return false;
            } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
                ESP_LOGE(TAG, "EXLAP auth recv error: %s", strerror(errno));
                return false;
            }
        }
    }

    ESP_LOGE(TAG, "Timed out waiting for EXLAP challenge nonce");
    return false;

got_nonce:
    ESP_LOGI(TAG, "Received EXLAP challenge nonce");

    if (!exlap_send_req_auth_response(nonce)) {
        return false;
    }

    s_last_auth_ms = (uint32_t)(esp_timer_get_time() / 1000);
    ESP_LOGI(TAG, "Sent EXLAP auth response; waiting for subscription Rsp");

    uint32_t first_sub_req_id = 0;
    if (!exlap_send_subscribe("vehicleSpeed", 100, &first_sub_req_id)) {
        ESP_LOGE(TAG, "Failed to send initial subscription");
        return false;
    }

    if (!exlap_wait_for_rsp(first_sub_req_id, pdMS_TO_TICKS(3000))) {
        ESP_LOGE(TAG, "No matching Rsp for EXLAP subscription request");
        return false;
    }

    s_session_authenticated = true;
    s_exlap_state = EXLAP_STATE_SUBSCRIBING;
    if (!exlap_send_subscription_set(true)) {
        return false;
    }

    s_exlap_state = EXLAP_STATE_AUTHENTICATED;
    s_last_heartbeat_ms = s_last_auth_ms;
    ESP_LOGI(TAG, "EXLAP session authenticated and subscriptions sent");
    return true;
}

/**
 * Connect TCP socket to vehicle EXLAP service
 */
static bool exlap_tcp_connect(const char *ip, uint16_t port)
{
    if (ip == NULL || strlen(ip) == 0) {
        ESP_LOGE(TAG, "No IP address specified");
        return false;
    }

    /* Check WiFi is connected */
    if (!wifi_manager_is_connected()) {
        ESP_LOGW(TAG, "WiFi not connected, cannot connect to EXLAP");
        return false;
    }

    /* Close existing socket */
    if (s_tcp_socket >= 0) {
        close(s_tcp_socket);
        s_tcp_socket = -1;
    }

    /* Clear any queued packets from the previous session */
    exlap_flush_packet_queue();

    /* Create TCP socket */
    s_tcp_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s_tcp_socket < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        return false;
    }

    /* Set socket options */
    int opt = 1;
    setsockopt(s_tcp_socket, SOL_SOCKET, SO_KEEPALIVE, &opt, sizeof(opt));

    /* Set receive timeout */
    struct timeval tv;
    tv.tv_sec = EXLAP_RECV_TIMEOUT_MS / 1000;
    tv.tv_usec = (EXLAP_RECV_TIMEOUT_MS % 1000) * 1000;
    setsockopt(s_tcp_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    /* Configure server address */
    memset(&s_server_addr, 0, sizeof(s_server_addr));
    s_server_addr.sin_family = AF_INET;
    s_server_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &s_server_addr.sin_addr);

    ESP_LOGI(TAG, "Connecting to EXLAP at %s:%d ...", ip, port);

    /* Connect */
    int ret = connect(s_tcp_socket, (struct sockaddr *)&s_server_addr,
                      sizeof(s_server_addr));
    if (ret < 0) {
        ESP_LOGE(TAG, "TCP connect failed: %s", strerror(errno));
        close(s_tcp_socket);
        s_tcp_socket = -1;
        return false;
    }

    strncpy(s_current_ip, ip, EXLAP_MAX_IP_LEN);
    s_current_ip[EXLAP_MAX_IP_LEN] = '\0';
    s_current_port = port;

    ESP_LOGI(TAG, "Connected to EXLAP at %s:%d", ip, port);
    return true;
}

/**
 * EXLAP TCP receive task
 * 
 * Continuously reads from the TCP socket and queues packets
 * for the parser. Handles reconnection on disconnect.
 */
static void exlap_task(void *pvParameters)
{
    uint8_t rx_buf[EXLAP_RX_BUF_SIZE + 1];
    exlap_packet_t *packet;
    int ret;

    ESP_LOGI(TAG, "EXLAP task started");

    while (true) {
        /* Wait for WiFi before starting or restarting a session */
        if (!wifi_manager_is_connected()) {
            if (s_tcp_socket >= 0) {
                exlap_client_disconnect();
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (s_exlap_state == EXLAP_STATE_DISCONNECTED) {
            exlap_config_t config;
            if (exlap_client_load_config(&config)) {
                exlap_apply_config(&config);
            }

            if (!config.enabled) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            if (strlen(s_current_ip) == 0) {
                ESP_LOGW(TAG, "No EXLAP IP configured, retrying");
                vTaskDelay(pdMS_TO_TICKS(EXLAP_RECONNECT_DELAY_MS));
                continue;
            }

            s_exlap_state = EXLAP_STATE_CONNECTING;
            if (!exlap_tcp_connect(s_current_ip, s_current_port)) {
                ESP_LOGW(TAG, "EXLAP connect failed, retrying in %lu ms",
                         EXLAP_RECONNECT_DELAY_MS);
                s_exlap_state = EXLAP_STATE_DISCONNECTED;
                vTaskDelay(pdMS_TO_TICKS(EXLAP_RECONNECT_DELAY_MS));
                continue;
            }

            if (!exlap_establish_session()) {
                ESP_LOGW(TAG, "EXLAP session setup failed, retrying");
                exlap_client_disconnect();
                vTaskDelay(pdMS_TO_TICKS(EXLAP_RECONNECT_DELAY_MS));
                continue;
            }
        }

        /* Read from TCP socket */
        ret = recv(s_tcp_socket, rx_buf, EXLAP_RX_BUF_SIZE, 0);

        if (ret > 0) {
            /* Data received */
            s_bytes_received += ret;
            rx_buf[ret] = '\0';

            if (strstr((const char *)rx_buf, "authenticationFailed") != NULL) {
                ESP_LOGE(TAG, "Server reported authentication failure");
                exlap_client_disconnect();
                vTaskDelay(pdMS_TO_TICKS(EXLAP_RECONNECT_DELAY_MS));
                continue;
            }

            if (!exlap_packet_contains_dat(rx_buf, (size_t)ret)) {
                /* Non-telemetry EXLAP control packet. Ignore it. */
            } else {
                /* Allocate queue packet */
                packet = (exlap_packet_t *)malloc(sizeof(exlap_packet_t));
                if (packet == NULL) {
                    ESP_LOGE(TAG, "Failed to allocate packet buffer");
                    s_packets_dropped++;
                    continue;
                }

                memcpy(packet->data, rx_buf, (size_t)ret);
                packet->len = (size_t)ret;

                /* Send to queue */
                /* The queue stores packet pointers, so enqueue the address of
                 * the pointer variable rather than the packet contents. */
                if (xQueueSend(s_packet_queue, &packet, pdMS_TO_TICKS(100)) != pdPASS) {
                    ESP_LOGW(TAG, "Packet queue full, dropping packet");
                    free(packet);
                    s_packets_dropped++;
                } else {
                    s_packets_received++;
                    s_last_packet_ms = (uint32_t)(esp_timer_get_time() / 1000);
                }
            }

        } else if (ret == 0) {
            /* Peer closed connection */
            ESP_LOGW(TAG, "EXLAP peer closed connection");
            exlap_client_disconnect();

        } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
            /* Error */
            ESP_LOGE(TAG, "EXLAP recv error: %s", strerror(errno));
            exlap_client_disconnect();

            /* Reconnect delay */
            vTaskDelay(pdMS_TO_TICKS(EXLAP_RECONNECT_DELAY_MS));
        }

        /* Heartbeat while session is live */
        if (s_exlap_state == EXLAP_STATE_AUTHENTICATED) {
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            if ((now_ms - s_last_heartbeat_ms) >= EXLAP_HEARTBEAT_INTERVAL_MS) {
                if (!exlap_send_heartbeat()) {
                    ESP_LOGW(TAG, "EXLAP heartbeat failed");
                    exlap_client_disconnect();
                    vTaskDelay(pdMS_TO_TICKS(EXLAP_RECONNECT_DELAY_MS));
                }
            }
        }
    }
}

/* ========== Public API ========== */

bool exlap_client_init(void)
{
    ESP_LOGI(TAG, "Initializing EXLAP client");

    /* Create packet queue */
    if (s_packet_queue != NULL) {
        vQueueDelete(s_packet_queue);
    }

    s_packet_queue = xQueueCreate(EXLAP_QUEUE_DEPTH, sizeof(exlap_packet_t *));
    if (s_packet_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create packet queue");
        return false;
    }

    /* Reset statistics */
    s_packets_received = 0;
    s_packets_dropped = 0;
    s_bytes_received = 0;
    s_last_packet_ms = 0;

    /* Reset state */
    s_exlap_state = EXLAP_STATE_DISCONNECTED;
    s_tcp_socket = -1;
    s_session_authenticated = false;
    s_last_auth_ms = 0;
    s_last_heartbeat_ms = 0;
    s_current_ip[0] = '\0';
    s_current_user[0] = '\0';
    s_current_password[0] = '\0';
    s_message_id = 98;

    ESP_LOGI(TAG, "EXLAP client initialized");
    return true;
}

void exlap_client_stop(void)
{
    ESP_LOGI(TAG, "Stopping EXLAP client");

    s_exlap_state = EXLAP_STATE_DISCONNECTED;
    s_session_authenticated = false;

    if (s_tcp_socket >= 0) {
        close(s_tcp_socket);
        s_tcp_socket = -1;
    }

    if (s_packet_queue != NULL) {
        /* Drain queue */
        exlap_packet_t *packet;
        while (xQueueReceive(s_packet_queue, &packet, 0) == pdPASS) {
            free(packet);
        }
        vQueueDelete(s_packet_queue);
        s_packet_queue = NULL;
    }
}

bool exlap_client_connect(const char *ip, uint16_t port)
{
    exlap_config_t config;
    if (exlap_client_load_config(&config)) {
        exlap_apply_config(&config);
    }

    if (ip != NULL && strlen(ip) > 0) {
        strncpy(s_current_ip, ip, EXLAP_MAX_IP_LEN);
        s_current_ip[EXLAP_MAX_IP_LEN] = '\0';
    }
    if (port > 0) {
        s_current_port = port;
    }

    if (strlen(s_current_ip) == 0) {
        ESP_LOGE(TAG, "No IP address configured");
        return false;
    }

    s_exlap_state = EXLAP_STATE_CONNECTING;
    return exlap_tcp_connect(s_current_ip, s_current_port);
}

void exlap_client_disconnect(void)
{
    s_exlap_state = EXLAP_STATE_DISCONNECTED;
    s_session_authenticated = false;
    s_last_auth_ms = 0;
    s_last_heartbeat_ms = 0;

    if (s_tcp_socket >= 0) {
        close(s_tcp_socket);
        s_tcp_socket = -1;
    }

    /* Drop queued packets from the disconnected session */
    exlap_flush_packet_queue();

    ESP_LOGI(TAG, "Disconnected from EXLAP");
}

bool exlap_client_start_task(void)
{
    TaskHandle_t task_handle;

    BaseType_t ret = xTaskCreatePinnedToCore(exlap_task,
                                             "exlap_task",
                                             EXLAP_TASK_STACK_SIZE,
                                             NULL,
                                             EXLAP_TASK_PRIORITY,
                                             &task_handle,
                                             EXLAP_TASK_CORE);

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create EXLAP task");
        return false;
    }

    ESP_LOGI(TAG, "EXLAP task started on Core %d", EXLAP_TASK_CORE);
    return true;
}

bool exlap_client_recv_packet(uint8_t *data, size_t *len, TickType_t timeout)
{
    if (data == NULL || len == NULL || s_packet_queue == NULL) {
        return false;
    }

    exlap_packet_t *packet;
    if (xQueueReceive(s_packet_queue, &packet, timeout) != pdPASS) {
        return false;
    }

    size_t copy_len = (*len < packet->len) ? *len : packet->len;
    memcpy(data, packet->data, copy_len);
    *len = copy_len;

    free(packet);
    return true;
}

void exlap_client_get_status(exlap_status_t *status)
{
    if (status == NULL) return;

    status->state = s_exlap_state;
    status->authenticated = s_session_authenticated;
    status->packets_received = s_packets_received;
    status->packets_dropped = s_packets_dropped;
    status->bytes_received = s_bytes_received;
    status->last_packet_ms = s_last_packet_ms;
    status->last_auth_ms = s_last_auth_ms;
    status->last_heartbeat_ms = s_last_heartbeat_ms;
}

bool exlap_client_is_connected(void)
{
    return (s_exlap_state == EXLAP_STATE_AUTHENTICATED);
}

bool exlap_client_save_config(const exlap_config_t *config)
{
    if (config == NULL) return false;

    exlap_config_t clean_config = *config;
    url_decode_inplace(clean_config.vehicle_ip);
    url_decode_inplace(clean_config.username);
    url_decode_inplace(clean_config.password);
    strip_wrapping_quotes(clean_config.vehicle_ip);
    strip_wrapping_quotes(clean_config.username);
    strip_wrapping_quotes(clean_config.password);

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(EXLAP_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return false;
    }

    if (strlen(clean_config.vehicle_ip) > 0) {
        ret = nvs_set_str(handle, NVS_KEY_IP, clean_config.vehicle_ip);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save IP: %s", esp_err_to_name(ret));
            nvs_close(handle);
            return false;
        }
    }

    ret = nvs_set_u16(handle, NVS_KEY_PORT, config->port);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save port: %s", esp_err_to_name(ret));
        nvs_close(handle);
        return false;
    }

    ret = nvs_set_u8(handle, NVS_KEY_ENABLE, config->enabled ? 1 : 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save enabled: %s", esp_err_to_name(ret));
        nvs_close(handle);
        return false;
    }

    if (strlen(clean_config.username) > 0) {
        ret = nvs_set_str(handle, NVS_KEY_USER, clean_config.username);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save username: %s", esp_err_to_name(ret));
            nvs_close(handle);
            return false;
        }
    }

    if (strlen(clean_config.password) > 0) {
        ret = nvs_set_str(handle, NVS_KEY_PASS, clean_config.password);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save password: %s", esp_err_to_name(ret));
            nvs_close(handle);
            return false;
        }
    }

    ret = nvs_commit(handle);
    nvs_close(handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "EXLAP config saved to NVS");
    return true;
}

bool exlap_client_load_config(exlap_config_t *config)
{
    if (config == NULL) return false;

    /* Set defaults */
    memset(config, 0, sizeof(exlap_config_t));
    config->port = EXLAP_DEFAULT_PORT;
    config->enabled = false;
    config->username[0] = '\0';
    config->password[0] = '\0';

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(EXLAP_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No saved EXLAP config, using defaults");
        return false;
    }

    size_t ip_len = sizeof(config->vehicle_ip);
    ret = nvs_get_str(handle, NVS_KEY_IP, config->vehicle_ip, &ip_len);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No saved IP address");
    } else {
        strip_wrapping_quotes(config->vehicle_ip);
    }

    uint16_t port;
    ret = nvs_get_u16(handle, NVS_KEY_PORT, &port);
    if (ret == ESP_OK) {
        config->port = port;
    }

    uint8_t enabled;
    ret = nvs_get_u8(handle, NVS_KEY_ENABLE, &enabled);
    if (ret == ESP_OK) {
        config->enabled = (enabled != 0);
    }

    size_t user_len = sizeof(config->username);
    ret = nvs_get_str(handle, NVS_KEY_USER, config->username, &user_len);
    if (ret != ESP_OK) {
        config->username[0] = '\0';
    } else {
        strip_wrapping_quotes(config->username);
    }

    size_t pass_len = sizeof(config->password);
    ret = nvs_get_str(handle, NVS_KEY_PASS, config->password, &pass_len);
    if (ret != ESP_OK) {
        config->password[0] = '\0';
    } else {
        strip_wrapping_quotes(config->password);
    }

    nvs_close(handle);
    ESP_LOGI(TAG, "Loaded EXLAP config: ip=%s, port=%d, enabled=%d, user=%s",
             config->vehicle_ip, config->port, config->enabled, config->username);
    return true;
}
