#include "bluetooth_spp.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_bt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define DEVICE_NAME "PanelHMI"

static const char *TAG = "BLE";

static uint8_t own_addr_type;
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t nus_tx_val_handle;

static bool notify_enabled = false;

static ble_rx_callback_t rx_callback = NULL;
static void *rx_callback_ctx = NULL;

static ble_tx_ready_callback_t tx_ready_callback = NULL;
static void *tx_ready_callback_ctx = NULL;

/* UUIDs NUS */

static const ble_uuid128_t nus_service_uuid =
    BLE_UUID128_INIT(
        0x9e,0xca,0xdc,0x24,
        0x0e,0xe5,
        0xa9,0xe0,
        0x93,0xf3,
        0xa3,0xb5,
        0x01,0x00,
        0x40,0x6e);

static const ble_uuid128_t nus_rx_uuid =
    BLE_UUID128_INIT(
        0x9e,0xca,0xdc,0x24,
        0x0e,0xe5,
        0xa9,0xe0,
        0x93,0xf3,
        0xa3,0xb5,
        0x02,0x00,
        0x40,0x6e);

static const ble_uuid128_t nus_tx_uuid =
    BLE_UUID128_INIT(
        0x9e,0xca,0xdc,0x24,
        0x0e,0xe5,
        0xa9,0xe0,
        0x93,0xf3,
        0xa3,0xb5,
        0x03,0x00,
        0x40,0x6e);

bool ble_is_connected(void)
{
    return conn_handle != BLE_HS_CONN_HANDLE_NONE;
}

bool ble_can_send(void)
{
    return ble_is_connected() && notify_enabled;
}

void ble_register_rx_callback(
    ble_rx_callback_t callback,
    void *ctx)
{
    rx_callback = callback;
    rx_callback_ctx = ctx;
}

void ble_register_tx_ready_callback(
    ble_tx_ready_callback_t callback,
    void *ctx)
{
    tx_ready_callback = callback;
    tx_ready_callback_ctx = ctx;
}

/* ========================= SEND ========================= */

int ble_send(
    const uint8_t *data,
    uint16_t len)
{
    if (!ble_can_send())
        return -1;

    const uint16_t CHUNK = 20;

    uint16_t offset = 0;

    while (offset < len) {

        uint16_t chunk_len = len - offset;

        if (chunk_len > CHUNK)
            chunk_len = CHUNK;

        struct os_mbuf *om =
            ble_hs_mbuf_from_flat(
                data + offset,
                chunk_len);

        if (om == NULL)
            return -2;

        int rc = ble_gatts_notify_custom(
            conn_handle,
            nus_tx_val_handle,
            om);

        if (rc != 0)
            return rc;

        offset += chunk_len;

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return 0;
}

int ble_send_string(const char *str)
{
    return ble_send(
        (const uint8_t *)str,
        strlen(str));
}

/* ========================= RX ========================= */

static int nus_rx_access_cb(
    uint16_t conn_handle_param,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg)
{
    struct os_mbuf *om = ctxt->om;

    int len = OS_MBUF_PKTLEN(om);

    if (len <= 0)
        return 0;

    uint8_t buffer[128];

    if (len > sizeof(buffer))
        len = sizeof(buffer);

    os_mbuf_copydata(
        om,
        0,
        len,
        buffer);

    if (rx_callback != NULL) {

        rx_callback(
            buffer,
            len,
            rx_callback_ctx);
    }

    return 0;
}

static int nus_tx_access_cb(
    uint16_t conn_handle_param,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg)
{
    return 0;
}

/* ========================= GATT ========================= */

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,

        .uuid = &nus_service_uuid.u,

        .characteristics = (struct ble_gatt_chr_def[]) {

            {
                .uuid = &nus_rx_uuid.u,

                .access_cb = nus_rx_access_cb,

                .flags =
                    BLE_GATT_CHR_F_WRITE |
                    BLE_GATT_CHR_F_WRITE_NO_RSP,
            },

            {
                .uuid = &nus_tx_uuid.u,

                .access_cb = nus_tx_access_cb,

                .val_handle = &nus_tx_val_handle,

                .flags = BLE_GATT_CHR_F_NOTIFY,
            },

            {0}
        },
    },

    {0}
};

static void ble_app_advertise(void);

/* ========================= GAP EVENTS ========================= */

static int ble_gap_event_cb(
    struct ble_gap_event *event,
    void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:

        if (event->connect.status == 0) {

            conn_handle =
                event->connect.conn_handle;

            ESP_LOGI(TAG, "Connected");
        }

        else {

            ble_app_advertise();
        }

        return 0;

    case BLE_GAP_EVENT_DISCONNECT:

        conn_handle =
            BLE_HS_CONN_HANDLE_NONE;

        notify_enabled = false;

        ESP_LOGI(TAG, "Disconnected");

        ble_app_advertise();

        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:

        if (event->subscribe.attr_handle ==
            nus_tx_val_handle)
        {
            notify_enabled =
                event->subscribe.cur_notify;

            ESP_LOGI(
                TAG,
                "Notify %s",
                notify_enabled ? "ON" : "OFF");

            if (tx_ready_callback != NULL) {

                tx_ready_callback(
                    notify_enabled,
                    tx_ready_callback_ctx);
            }
        }

        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:

        ble_app_advertise();

        return 0;

    default:
        return 0;
    }
}

/* ========================= ADVERTISING ========================= */

static void ble_app_advertise(void)
{
    struct ble_hs_adv_fields fields;

    memset(&fields, 0, sizeof(fields));

    fields.flags =
        BLE_HS_ADV_F_DISC_GEN |
        BLE_HS_ADV_F_BREDR_UNSUP;

    fields.name =
        (uint8_t *)DEVICE_NAME;

    fields.name_len =
        strlen(DEVICE_NAME);

    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params adv_params;

    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.conn_mode =
        BLE_GAP_CONN_MODE_UND;

    adv_params.disc_mode =
        BLE_GAP_DISC_MODE_GEN;

    ble_gap_adv_start(
        own_addr_type,
        NULL,
        BLE_HS_FOREVER,
        &adv_params,
        ble_gap_event_cb,
        NULL);
}

/* ========================= HOST ========================= */

static void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(
        0,
        &own_addr_type);

    ble_app_advertise();
}

static void ble_app_on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE reset: %d", reason);
}

static void ble_host_task(void *param)
{
    nimble_port_run();

    nimble_port_freertos_deinit();
}

/* ========================= INIT ========================= */

void bluetooth_init(void)
{
    esp_bt_controller_mem_release(
        ESP_BT_MODE_CLASSIC_BT);

    nimble_port_init();

    ble_svc_gap_init();

    ble_svc_gatt_init();

    ble_svc_gap_device_name_set(
        DEVICE_NAME);

    ble_hs_cfg.sync_cb =
        ble_app_on_sync;

    ble_hs_cfg.reset_cb =
        ble_app_on_reset;

    ble_gatts_count_cfg(
        gatt_svr_svcs);

    ble_gatts_add_svcs(
        gatt_svr_svcs);

    nimble_port_freertos_init(
        ble_host_task);

    esp_err_t sleep_rc = esp_bt_sleep_disable();
    if (sleep_rc != ESP_OK) {
        ESP_LOGW(TAG, "esp_bt_sleep_disable rc=%d", sleep_rc);
    }

    ESP_LOGI(TAG, "BLE READY");
}