/**
 * RaceChrono DIY BLE Service Implementation
 * 
 * Uses NimBLE stack to implement RaceChrono CAN-over-BLE protocol
 * - Service: 0x1FF8
 * - Characteristic 0x0001: CAN Notify (Notify)
 * - Characteristic 0x0002: CAN Filter (Write)
 * 
 * ESP-IDF v6.0.1 NimBLE GATT Server API
 */

#include "ble_service.h"
#include "filter.h"
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nimble/ble.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "host/ble_hs_mbuf.h"

#include <string.h>
#include <stdint.h>
#include <inttypes.h>

static const char *TAG = "ble_service";

/* RaceChrono Service UUID: 0x1FF8 */
#define RC_SVC_UUID16          0x1FF8
#define RC_CAN_NOTIFY_UUID16   0x0001
#define RC_CAN_FILTER_UUID16   0x0002

/* BLE Advertising interval (30ms) */
#define ADV_ITVL_MIN           (30 * 16 / 1.25)
#define ADV_ITVL_MAX           (30 * 16 / 1.25)

/* Connection state */
static uint16_t s_conn_handle = 0xFFFF;
static bool s_connected = false;

/* Characteristic value handles (filled during GATT registration) */
static uint16_t s_chr_can_notify_val_handle = 0;

/* Device name for advertising */
#define DEVICE_NAME            "RC_TireX"

/* BLE address type chosen once the host syncs with the controller */
static uint8_t s_own_addr_type = BLE_OWN_ADDR_PUBLIC;

static int gap_event_handler(struct ble_gap_event *event, void *arg);
static void ble_gatts_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);

/*
 * GATT Attribute Callbacks (ESP-IDF v6.0.1 NimBLE API)
 */

static int
rc_can_notify_read_cb(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    /* Read not used for notify characteristic */
    return BLE_ATT_ERR_READ_NOT_PERMITTED;
}

static int
rc_can_filter_write_cb(uint16_t conn_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->om == NULL) {
        return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
    }
    
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    ESP_LOGI(TAG, "Filter write received, len=%u", len);
    
    if (len < 1) {
        ESP_LOGW(TAG, "Filter write too short");
        return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
    }

    if (len > 32) {
        ESP_LOGW(TAG, "Filter write too long (%u bytes)", len);
        return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
    }

    uint8_t filter_buf[32];
    int rc = os_mbuf_copydata(ctxt->om, 0, len, filter_buf);
    if (rc != 0) {
        ESP_LOGW(TAG, "Failed to copy filter write payload: %d", rc);
        return BLE_ATT_ERR_UNLIKELY;
    }
    
    /* Pass to filter module */
    filter_process(filter_buf, len);
    
    return 0;
}

/*
 * GATT Attribute Definitions (struct ble_gatt_svc_def format)
 * Fixed: One service with two characteristics
 */

static const struct ble_gatt_svc_def s_rc_gatt_svcs[] = {
    /* Service: 0x1FF8 */
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(RC_SVC_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]) { {
            /* CAN Notify Characteristic */
            .uuid = BLE_UUID16_DECLARE(RC_CAN_NOTIFY_UUID16),
            .access_cb = rc_can_notify_read_cb,
            .flags = BLE_GATT_CHR_PROP_NOTIFY | BLE_GATT_CHR_F_READ,
        }, {
            /* CAN Filter Characteristic */
            .uuid = BLE_UUID16_DECLARE(RC_CAN_FILTER_UUID16),
            .access_cb = rc_can_filter_write_cb,
            .flags = BLE_GATT_CHR_PROP_WRITE | BLE_GATT_CHR_F_WRITE,
        }, {
            0, /* No more characteristics in this service */
        } },
    },
    {
        0, /* No more services */
    }
};

/*
 * NimBLE host task
 */

static void
nimble_host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE host task started");

    /* This returns only after nimble_port_stop() is called. */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

/*
 * Advertising helpers
 */

static bool
start_advertising(void)
{
    int rc;

    /* Flags only in the advertising packet. */
    uint8_t adv_data[] = {
        0x02, /* Length */
        0x01, /* AD Type: Flags */
        0x06, /* LE General Discoverable, BR/EDR Not Supported */
    };

    rc = ble_gap_adv_set_data(adv_data, sizeof(adv_data));
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertising data: %d", rc);
        return false;
    }

    /* Put name + service UUID in the scan response. */
    uint8_t scan_data[] = {
        0x09, /* Length = type byte + 8 name bytes */
        0x09, /* AD Type: Complete Local Name */
        'R', 'C', '_', 'T', 'i', 'r', 'e', 'X',
        0x03, /* Length = type byte + 2 UUID bytes */
        0x02, /* AD Type: Incomplete List of 16-bit UUIDs */
        0xF8, 0x01, /* Service UUID 0x1FF8 (little-endian) */
    };

    rc = ble_gap_adv_rsp_set_data(scan_data, sizeof(scan_data));
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set scan response data: %d", rc);
        return false;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = ADV_ITVL_MIN;
    adv_params.itvl_max = ADV_ITVL_MAX;

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising: %d", rc);
        return false;
    }

    ESP_LOGI(TAG, "Advertising started");
    return true;
}

static void
ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE stack reset: reason=%d", reason);
}

static void
ble_on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to infer address type: %d", rc);
        return;
    }

    if (!start_advertising()) {
        ESP_LOGE(TAG, "Failed to start advertising after sync");
    }
}

static void
ble_gatts_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_REGISTER_OP_CHR) {
        return;
    }

    if (ble_uuid_cmp(ctxt->chr.svc_def->uuid, BLE_UUID16_DECLARE(RC_SVC_UUID16)) != 0) {
        return;
    }

    if (ble_uuid_cmp(ctxt->chr.chr_def->uuid, BLE_UUID16_DECLARE(RC_CAN_NOTIFY_UUID16)) == 0) {
        s_chr_can_notify_val_handle = ctxt->chr.val_handle;
        ESP_LOGI(TAG, "Notify characteristic registered: val_handle=0x%04x",
                 s_chr_can_notify_val_handle);
    }
}

/*
 * GAP Event Handler
 */

static int
gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_connected = true;
            ESP_LOGD(TAG, "BLE connected: handle=%d", s_conn_handle);
            ESP_LOGI(TAG, "Connected to RaceChrono (handle=%d)", s_conn_handle);
        } else {
            ESP_LOGE(TAG, "Connection failed: %d", event->connect.status);
            ble_gap_terminate(event->connect.conn_handle,
                             BLE_ERR_REM_USER_CONN_TERM);
        }
        break;
        
    case BLE_GAP_EVENT_DISCONNECT:
        s_connected = false;
        s_conn_handle = 0xFFFF;
        ESP_LOGI(TAG, "Disconnected: reason=%d", event->disconnect.reason);
        if (!start_advertising()) {
            ESP_LOGE(TAG, "Failed to restart advertising");
        }
        break;
        
    default:
        break;
    }

    return 0;
}

/*
 * Notification Helper
 */

static int
notify_can_frame(uint16_t conn_handle, uint32_t can_id,
                 const uint8_t *data, uint8_t len)
{
    if (conn_handle == 0xFFFF ||
        s_chr_can_notify_val_handle == 0) {
        return BLE_ERR_UNSUPPORTED;
    }

    /* Build notification: 4-byte CAN ID (little-endian) + payload */
    uint8_t notify_buf[13];
    notify_buf[0] = can_id & 0xFF;
    notify_buf[1] = (can_id >> 8) & 0xFF;
    notify_buf[2] = (can_id >> 16) & 0xFF;
    notify_buf[3] = (can_id >> 24) & 0xFF;
    
    if (len > 8) len = 8;
    memcpy(&notify_buf[4], data, len);
    
    struct os_mbuf *om = ble_hs_mbuf_from_flat(notify_buf, 4 + len);
    if (om == NULL) {
        ESP_LOGE(TAG, "Failed to allocate mbuf for notification");
        return BLE_ERR_MEM_CAPACITY;
    }

    int rc = ble_gattc_notify_custom(conn_handle,
                                     s_chr_can_notify_val_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "Failed to send notification: %d", rc);
    } else {
        ESP_LOGD(TAG, "BLE notify delivered: conn=%u can_id=0x%08" PRIX32,
                 conn_handle, can_id);
        ESP_LOGI(TAG, "Sent BLE notify: conn=%u can_id=0x%08" PRIX32 " len=%u",
                 conn_handle, can_id, len);
    }

    return rc;
}

/*
 * Public API
 */

bool ble_service_init(void)
{
    ESP_LOGI(TAG, "Initializing RaceChrono BLE service");
    
    /* Initialize NimBLE */
    int rc = nimble_port_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to initialize NimBLE: %d", rc);
        return false;
    }

    /* Register GATT attributes */
    rc = ble_gatts_count_cfg(s_rc_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to count GATT config: %d", rc);
        return false;
    }

    rc = ble_gatts_add_svcs(s_rc_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to add GATT services: %d", rc);
        return false;
    }

    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.gatts_register_cb = ble_gatts_register_cb;

    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "RaceChrono BLE service initialized");
    return true;
}

void ble_service_stop(void)
{
    nimble_port_stop();
}

bool ble_send_can_frame(uint32_t can_id, const uint8_t *data, uint8_t len)
{
    if (!s_connected || s_conn_handle == 0xFFFF) {
        return false;
    }

    int rc = notify_can_frame(s_conn_handle, can_id, data, len);
    return (rc == 0);
}

bool ble_is_connected(void)
{
    return s_connected;
}

uint16_t ble_get_conn_handle(void)
{
    return s_conn_handle;
}
