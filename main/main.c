/**
 * TireX → RaceChrono BLE Bridge + EXLAP Gateway
 * 
 * Main entry point for ESP32-CAN-X2 firmware
 * 
 * Architecture:
 *   TireX CAN Bus → CAN1/TWAI → CAN RX Task → Filter → BLE TX Queue → RaceChrono
 *   Vehicle EXLAP → WiFi TCP → EXLAP Client → Parser → Telemetry DB → BLE/Web UI
 * 
 * Tasks:
 *   - can_rx_task: Receive CAN frames, apply filter, forward via BLE (Core 1)
 *   - exlap_task: TCP receive from vehicle EXLAP service (Core 0)
 *   - exlap_recv_task: Parse EXLAP telemetry packets (Core 0)
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>
#include <stdio.h>
#include "can_driver.h"
#include "ble_service.h"
#include "filter.h"
#include "tirex_decoder.h"
#include "tirex_config.h"
#include "wifi_manager.h"
#include "exlap_client.h"
#include "exlap_parser.h"
#include "telemetry_db.h"
#include "web_ui.h"

#if CONFIG_EXLAP_ENABLED
#define EXLAP_PARSER_PRINT_INTERVAL_MS  2000
#endif

static const char *TAG = "main";

/* CAN RX Task Stack Size */
#define CAN_RX_TASK_STACK_SIZE  (4096)
#define CAN_RX_TASK_PRIORITY    (5)

/* Debug: Print TireX data every N ms */
#define TIREX_PRINT_INTERVAL_MS  1000

/* Telemetry forwarding task config */
#define TELEMETRY_TASK_STACK_SIZE 4096
#define TELEMETRY_TASK_PRIORITY   4
#define TELEMETRY_FORWARD_MS      200  /* 5 Hz */
#define TELEMETRY_JSON_LOG_MS     10000 /* 0.1 Hz */

/* Experimental RaceChrono CAN payload for EXLAP steering angle.
 * RaceChrono should be configured to watch this extended CAN ID and
 * decode bytes 0-1 as a signed little-endian steering angle in centi-degrees.
 */
#define EXPERIMENTAL_STEERING_CAN_ID      0x18FF5A01
#define EXPERIMENTAL_STEERING_SCALE       100.0f

/* Global frame counter for diagnostics */
static uint32_t s_frame_count = 0;
#if CONFIG_EXLAP_ENABLED
static uint32_t s_last_experimental_steering_log_ms = 0;
#endif

#if CONFIG_EXLAP_ENABLED
/* Periodic EXLAP diagnostics print interval */
#define EXLAP_PRINT_INTERVAL_MS  2000
#endif /* CONFIG_EXLAP_ENABLED */

/**
 * CAN RX Task
 * 
 * Continuously receives CAN frames, applies RaceChrono filter,
 * and forwards matching frames via BLE.
 */
static void can_rx_task(void *pvParameters)
{
    twai_message_t msg;
    tirex_data_t tirex;
    uint32_t last_print_ms = 0;

    ESP_LOGI(TAG, "CAN RX task started on Core %d", xPortGetCoreID());

    /* Zero-initialize local TireX accumulator */
    memset(&tirex, 0, sizeof(tirex));

    while (true) {
        /* Receive CAN frame with timeout */
        esp_err_t ret = can_driver_receive(&msg, pdMS_TO_TICKS(1000));

        if (ret != ESP_OK) {
            continue;
        }

        /* Only process extended frames (29-bit IDs) */
        if (msg.flags & TWAI_MSG_FLAG_EXTD) {
            uint32_t can_id = msg.identifier;
            s_frame_count++;

            /* Track TireX configuration state and live sensor inventory. */
            tirex_config_update_from_frame(can_id, msg.data, msg.data_length_code);

            /* Decode TireX data into local accumulator */
            tirex_decoder_process(can_id, msg.data, &tirex);

            /* Push decoded TireX data to telemetry database */
            float lf_temps[4] = {tirex.lf.temp1, tirex.lf.temp2,
                                 tirex.lf.temp3, tirex.lf.temp4};
            float rf_temps[4] = {tirex.rf.temp1, tirex.rf.temp2,
                                 tirex.rf.temp3, tirex.rf.temp4};
            float lr_temps[4] = {tirex.lr.temp1, tirex.lr.temp2,
                                 tirex.lr.temp3, tirex.lr.temp4};
            float rr_temps[4] = {tirex.rr.temp1, tirex.rr.temp2,
                                 tirex.rr.temp3, tirex.rr.temp4};
            telemetry_db_update_tirex(lf_temps, rf_temps, lr_temps, rr_temps);

            /* Print diagnostics periodically — read from DB snapshot */
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            if (now_ms - last_print_ms >= TIREX_PRINT_INTERVAL_MS) {
#if CONFIG_TIREX_DECODER_CONSOLE_LOGGING
                telemetry_t snap;
                if (telemetry_db_get_snapshot(&snap)) {
                    ESP_LOGI(TAG, "TireX: LF=%.1f RF=%.1f LR=%.1f RR=%.1f (max=%.1f)",
                             snap.avg_temp_lf, snap.avg_temp_rf,
                             snap.avg_temp_lr, snap.avg_temp_rr,
                             snap.max_temp);
                }
#endif
                last_print_ms = now_ms;
            }

            /* Apply RaceChrono filter */
            if (filter_should_forward(can_id)) {
                /* Forward via BLE only when a RaceChrono client is connected.
                 * We intentionally stay quiet when BLE is idle so the console
                 * is not flooded during normal startup or reconnect windows. */
                if (ble_is_connected()) {
                    if (!ble_send_can_frame(can_id, msg.data, msg.data_length_code)) {
#if CONFIG_TIREX_SUPPRESS_BLE_IDLE_SEND_WARNINGS
                        ESP_LOGD(TAG, "BLE CAN forward dropped");
#else
                        ESP_LOGW(TAG, "BLE send failed (not connected?)");
#endif
                    }
                }
            }
        }
    }
}

#if CONFIG_EXLAP_ENABLED
/**
 * EXLAP receive task — reads packets from EXLAP client queue and parses
 */
static void exlap_recv_task(void *pvParameters)
{
    uint8_t rx_buf[EXLAP_MAX_PACKET_SIZE];
    size_t rx_len;
    uint32_t last_print_ms = 0;
    bool exlap_session_active = false;

    /* Initialize parser */
    exlap_parser_init();

    ESP_LOGI(TAG, "EXLAP receive task started on Core %d", xPortGetCoreID());

    while (true) {
        bool connected = exlap_client_is_connected();

        if (connected != exlap_session_active) {
            exlap_parser_reset();
            exlap_session_active = connected;
        }

        if (!connected) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        /* Receive packet from EXLAP client queue */
        rx_len = sizeof(rx_buf);
        if (exlap_client_recv_packet(rx_buf, &rx_len, pdMS_TO_TICKS(1000))) {
            /* Parse into a local struct */
            exlap_telemetry_t ex;
            if (exlap_parser_parse(rx_buf, rx_len, &ex)) {
                /*
                 * Only write fields that were actually present in the
                 * packet.  For missing fields, pass the current DB value
                 * so the DB doesn't overwrite fresh data with zeros.
                 */
                telemetry_t snap;
                telemetry_db_get_snapshot(&snap);

                float speed  = ex.has_vehicle_speed    ? ex.vehicle_speed    : snap.vehicle_speed_kmh;
                float rpm    = ex.has_engine_speed     ? ex.engine_speed     : snap.engine_rpm;
                float thr    = ex.has_accelerator_pos  ? ex.accelerator_pos  : snap.throttle_pct;
                float brake  = ex.has_brake_pressure   ? ex.brake_pressure   : snap.brake_pressure_bar;
                float steer  = ex.has_steering_angle   ? ex.steering_angle   : snap.steering_angle_deg;
                float yaw    = ex.has_yaw_rate         ? ex.yaw_rate         : snap.yaw_rate;
                int8_t gear  = ex.has_gear             ? ex.gear             : snap.gear;
                float ws_fl  = ex.has_wheel_speed_fl   ? ex.wheel_speed_fl   : snap.wheel_speed_fl;
                float ws_fr  = ex.has_wheel_speed_fr   ? ex.wheel_speed_fr   : snap.wheel_speed_fr;
                float ws_rl  = ex.has_wheel_speed_rl   ? ex.wheel_speed_rl   : snap.wheel_speed_rl;
                float ws_rr  = ex.has_wheel_speed_rr   ? ex.wheel_speed_rr   : snap.wheel_speed_rr;
                float tp_fl  = ex.has_tire_pressure_fl ? ex.tire_pressure_fl : snap.tire_pressure_lf;
                float tp_fr  = ex.has_tire_pressure_fr ? ex.tire_pressure_fr : snap.tire_pressure_rf;
                float tp_rl  = ex.has_tire_pressure_rl ? ex.tire_pressure_rl : snap.tire_pressure_lr;
                float tp_rr  = ex.has_tire_pressure_rr ? ex.tire_pressure_rr : snap.tire_pressure_rr;

                telemetry_db_update_exlap(speed, rpm, thr, brake, steer, yaw,
                                          gear, ws_fl, ws_fr, ws_rl, ws_rr,
                                          tp_fl, tp_fr, tp_rl, tp_rr);
            }

            /* Periodically print telemetry to console — read from DB snapshot */
            uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
            if (now - last_print_ms >= EXLAP_PRINT_INTERVAL_MS) {
#if CONFIG_EXLAP_PARSER_CONSOLE_LOGGING
                telemetry_t snap;
                if (telemetry_db_get_snapshot(&snap)) {
                    ESP_LOGI(TAG, "EXLAP: spd=%.1f rpm=%.0f thr=%.1f brk=%.2f gear=%d",
                             snap.vehicle_speed_kmh, snap.engine_rpm,
                             snap.throttle_pct, snap.brake_pressure_bar,
                             snap.gear);
                }
#endif
                last_print_ms = now;
            }
        }
    }
}
#endif /* CONFIG_EXLAP_ENABLED */

/**
 * Encode telemetry snapshot to compact binary payload (≤27 bytes for BLE MTU)
 *
 * Payload layout:
 *   [0-3]   timestamp_ms (LE uint32)
 *   [4-5]   vehicle_speed_kmh (LE uint16, ×10)
 *   [6-7]   engine_rpm (LE uint16, ×10)
 *   [8]     throttle_pct (uint8, 0-100)
 *   [9]     brake_pressure_bar (uint8, ×10)
 *   [10-11] steering_angle_deg (LE int16, ×10)
 *   [12]    gear (int8)
 *   [13-16] avg_temp_lf, rf, lr, rr (uint8, °C)
 *   [17]    max_temp (uint8, °C)
 *   [18-21] delta_temp_lf, rf, lr, rr (uint8, °C)
 *   [22-23] tirex/exlap update counts (LE uint16 each, capped)
 *
 * @param snap      Telemetry snapshot from DB
 * @param buf       Output buffer (≥24 bytes)
 * @param buf_len   Buffer size
 * @return          Encoded payload length
 */
static uint8_t encode_telemetry_binary(const telemetry_t *snap, uint8_t *buf, size_t buf_len)
{
    if (!snap || !buf || buf_len < 24) return 0;

    uint8_t i = 0;

    /* timestamp_ms */
    buf[i++] = snap->timestamp_ms & 0xFF;
    buf[i++] = (snap->timestamp_ms >> 8) & 0xFF;
    buf[i++] = (snap->timestamp_ms >> 16) & 0xFF;
    buf[i++] = (snap->timestamp_ms >> 24) & 0xFF;

    /* vehicle_speed_kmh × 10 */
    uint16_t speed = (uint16_t)(snap->vehicle_speed_kmh * 10.0f);
    buf[i++] = speed & 0xFF;
    buf[i++] = (speed >> 8) & 0xFF;

    /* engine_rpm × 10 */
    uint16_t rpm = (uint16_t)(snap->engine_rpm * 10.0f);
    buf[i++] = rpm & 0xFF;
    buf[i++] = (rpm >> 8) & 0xFF;

    /* throttle_pct */
    buf[i++] = (uint8_t)(snap->throttle_pct > 100.0f ? 100 : (snap->throttle_pct < 0.0f ? 0 : snap->throttle_pct));

    /* brake_pressure_bar × 10 */
    buf[i++] = (uint8_t)(snap->brake_pressure_bar * 10.0f > 255.0f ? 255 : snap->brake_pressure_bar * 10.0f);

    /* steering_angle_deg × 10 (signed int16) */
    int16_t steer = (int16_t)(snap->steering_angle_deg * 10.0f);
    buf[i++] = steer & 0xFF;
    buf[i++] = (steer >> 8) & 0xFF;

    /* gear */
    buf[i++] = (uint8_t)(snap->gear + 128); /* Offset to make unsigned */

    /* avg temps (°C as uint8) */
    buf[i++] = (uint8_t)snap->avg_temp_lf;
    buf[i++] = (uint8_t)snap->avg_temp_rf;
    buf[i++] = (uint8_t)snap->avg_temp_lr;
    buf[i++] = (uint8_t)snap->avg_temp_rr;

    /* max temp */
    buf[i++] = (uint8_t)snap->max_temp;

    /* delta temps */
    buf[i++] = (uint8_t)snap->delta_temp_lf;
    buf[i++] = (uint8_t)snap->delta_temp_rf;
    buf[i++] = (uint8_t)snap->delta_temp_lr;
    buf[i++] = (uint8_t)snap->delta_temp_rr;

    /* update counts (capped to uint16) */
    uint16_t tc = (uint16_t)(snap->tirex_update_count > 65535 ? 65535 : snap->tirex_update_count);
    buf[i++] = tc & 0xFF;
    buf[i++] = (tc >> 8) & 0xFF;

    uint16_t ec = (uint16_t)(snap->exlap_update_count > 65535 ? 65535 : snap->exlap_update_count);
    buf[i++] = ec & 0xFF;
    buf[i++] = (ec >> 8) & 0xFF;

    return i;
}

#if CONFIG_EXLAP_ENABLED
/**
 * Encode EXLAP steering angle into a simple CAN payload for RaceChrono.
 *
 * Payload format:
 *   [0-1] steering_angle_deg × 100 (LE int16)
 *   [2-7] reserved zero bytes
 */
static uint8_t encode_experimental_steering_can(const telemetry_t *snap,
                                                 uint8_t *buf,
                                                 size_t buf_len)
{
    if (!snap || !buf || buf_len < 8) return 0;

    memset(buf, 0, 8);

    int16_t steer = (int16_t)(snap->steering_angle_deg * EXPERIMENTAL_STEERING_SCALE);
    buf[0] = (uint8_t)(steer & 0xFF);
    buf[1] = (uint8_t)((steer >> 8) & 0xFF);

    return 8;
}
#endif

/**
 * Telemetry forwarding task — fetches snapshots from DB and sends via BLE
 */
static void telemetry_forward_task(void *pvParameters)
{
    telemetry_t snap;
    uint8_t buf[32];
    uint32_t last_json_ms = 0;

    ESP_LOGI(TAG, "Telemetry forward task started on Core %d", xPortGetCoreID());

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_FORWARD_MS));

        /* Fetch thread-safe snapshot */
        if (!telemetry_db_get_snapshot(&snap)) {
            continue;
        }

        /* Encode to compact binary */
        uint8_t len = encode_telemetry_binary(&snap, buf, sizeof(buf));
        if (len > 0 && ble_is_connected()) {
            uint16_t mtu = ble_get_mtu();
            if (mtu >= (uint16_t)(len + 3)) {
                if (!ble_send_telemetry_snapshot(buf, len)) {
#if CONFIG_TIREX_SUPPRESS_BLE_IDLE_SEND_WARNINGS
                    ESP_LOGD(TAG, "BLE telemetry forward dropped");
#else
                    ESP_LOGW(TAG, "BLE telemetry send failed");
#endif
                }
            } else {
                /* Wait for MTU negotiation before sending the full snapshot. */
                ESP_LOGD(TAG, "Skipping telemetry send until MTU is ready (need %u, have %u)",
                         (unsigned)(len + 3), (unsigned)mtu);
            }
        }

#if CONFIG_EXLAP_ENABLED
        /* Experimental path: publish EXLAP steering angle as a custom CAN frame
         * so RaceChrono can map it to a display channel for validation. */
        if (ble_is_connected() && snap.has_exlap_data) {
            uint8_t steering_buf[8];
            uint8_t steering_len = encode_experimental_steering_can(&snap, steering_buf, sizeof(steering_buf));
            if (steering_len == 8) {
                if (!ble_send_can_frame(EXPERIMENTAL_STEERING_CAN_ID, steering_buf, steering_len)) {
#if CONFIG_TIREX_SUPPRESS_BLE_IDLE_SEND_WARNINGS
                    ESP_LOGD(TAG, "Experimental steering CAN forward dropped");
#else
                    ESP_LOGW(TAG, "Experimental steering CAN send failed");
#endif
                } else {
                    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
                    if (now_ms - s_last_experimental_steering_log_ms >= 1000) {
                        int16_t steer_raw = (int16_t)(snap.steering_angle_deg * EXPERIMENTAL_STEERING_SCALE);
                        ESP_LOGI(TAG, "EXPERIMENT_CAN: id=0x%08X steering=%.1f raw=%d",
                                 EXPERIMENTAL_STEERING_CAN_ID,
                                 snap.steering_angle_deg,
                                 steer_raw);
                        s_last_experimental_steering_log_ms = now_ms;
                    }
                }
            }
        }
#endif

        /* Periodic JSON logging for web dashboard / console */
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if (now_ms - last_json_ms >= TELEMETRY_JSON_LOG_MS) {
            char json_buf[1024];
            if (telemetry_db_to_json(json_buf, sizeof(json_buf))) {
                ESP_LOGI(TAG, "TELEMETRY_JSON: %s", json_buf);
            }
            last_json_ms = now_ms;
        }
    }
}

/**
 * Application entry point
 */
void app_main(void)
{
#if !CONFIG_TIREX_DISABLE_ALL_CONSOLE_LOGGING
    ESP_LOGI(TAG, "=== TireX → RaceChrono BLE Bridge + EXLAP Gateway ===");
#endif

    /* Initialize NVS - required before BLE/WiFi */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize modules */
    filter_init();
    tirex_decoder_init();
    if (!tirex_config_init()) {
        ESP_LOGW(TAG, "Failed to initialize TireX config manager");
    }

    /* Initialize telemetry database */
    if (!telemetry_db_init()) {
        ESP_LOGE(TAG, "Failed to initialize telemetry database");
        return;
    }

    /* Initialize WiFi for the web UI and EXLAP gateway */
    if (!wifi_manager_init()) {
        ESP_LOGW(TAG, "WiFi manager initialization failed, web UI/EXLAP may be unavailable");
    }

#if CONFIG_EXLAP_TESTING_AP_UI
    /* Start web diagnostics UI */
    if (!web_ui_init()) {
        ESP_LOGW(TAG, "Failed to start web UI");
    }
#endif

    /* Initialize CAN driver */
    esp_err_t ret_can = can_driver_init();
    if (ret_can != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CAN driver: %s", esp_err_to_name(ret_can));
        return;
    }

    /* Initialize BLE service */
    if (!ble_service_init()) {
        ESP_LOGE(TAG, "Failed to initialize BLE service");
        can_driver_stop();
        return;
    }

#if CONFIG_EXLAP_ENABLED
    /* Initialize EXLAP client */
    exlap_client_init();

    /* Start EXLAP client task on Core 0 */
    exlap_client_start_task();

    /* Start EXLAP receive task on Core 0 */
    TaskHandle_t exlap_recv_handle;
    ret = xTaskCreatePinnedToCore(exlap_recv_task,
                                  "exlap_recv_task",
                                  4096,
                                  NULL,
                                  5,
                                  &exlap_recv_handle,
                                  0);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create EXLAP receive task");
    }

#endif /* CONFIG_EXLAP_ENABLED */

    /* Start CAN RX task on Core 1 */
    ret = xTaskCreatePinnedToCore(can_rx_task, "can_rx_task",
                                  CAN_RX_TASK_STACK_SIZE, NULL,
                                  CAN_RX_TASK_PRIORITY, NULL, 1);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create CAN RX task");
        ble_service_stop();
        can_driver_stop();
        return;
    }

    /* Start telemetry forwarding task on Core 0 */
    ret = xTaskCreatePinnedToCore(telemetry_forward_task, "telemetry_fwd_task",
                                  TELEMETRY_TASK_STACK_SIZE, NULL,
                                  TELEMETRY_TASK_PRIORITY, NULL, 0);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create telemetry forward task");
    }

#if !CONFIG_TIREX_DISABLE_ALL_CONSOLE_LOGGING
    ESP_LOGI(TAG, "System initialized successfully");
    ESP_LOGI(TAG, "Waiting for RaceChrono connection...");
#endif

    /* Main loop - monitor system status */
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        /* Periodic status */
#if !CONFIG_TIREX_DISABLE_ALL_CONSOLE_LOGGING
        ESP_LOGI(TAG, "Status: CAN=%s, BLE=%s, Frames=%lu",
                 can_driver_is_running() ? "OK" : "ERR",
                 ble_is_connected() ? "Connected" : "Advertising",
                 s_frame_count);

#if CONFIG_EXLAP_ENABLED
        exlap_status_t exlap_status;
        exlap_client_get_status(&exlap_status);
        ESP_LOGI(TAG, "EXLAP: state=%d auth=%d pkts_rx=%lu pkts_drop=%lu last_auth=%lu last_hb=%lu",
                 exlap_status.state,
                 exlap_status.authenticated ? 1 : 0,
                 exlap_status.packets_received,
                 exlap_status.packets_dropped,
                 (unsigned long)exlap_status.last_auth_ms,
                 (unsigned long)exlap_status.last_heartbeat_ms);
#endif /* CONFIG_EXLAP_ENABLED */
#endif /* !CONFIG_TIREX_DISABLE_ALL_CONSOLE_LOGGING */
    }
}
