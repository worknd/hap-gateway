/* Support for Xiaomi2 temperature and humidity BLE sensor (LYWSD03MMC) */

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_mac.h>

#include <host/util/util.h>
#include <host/ble_hs.h>
#include <host/ble_uuid.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <services/gap/ble_svc_gap.h>

#include <hap_platform_keystore.h>

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#define SENSOR_GET_TIMEOUT (10 * 60 * 1000000) /* 10 minutes in microseconds */

#define SENSOR_FOUND_BIT BIT0
#define SENSOR_INFO_BIT  BIT1
#define SENSOR_GOT_BIT   BIT2

#define SENSOR_KEYSTORE  "sensor"
#define SENSOR_ADDR_KEY  "address"

#define MAX_VBAT_MV 3000 /* 100% */
#define MIN_VBAT_MV 2200 /* 0% */

extern char g_name[32];
extern char g_manufacturer[32];
extern char g_model[32];
extern char g_serial[16];
extern char g_fwrev[16];
extern char g_hwrev[16];

extern void led_blink(void);
extern void change_temperature(float temperature);
extern void change_humidity(float humidity);
extern void change_battery(uint32_t battery);

static const char *TAG = "MI2";

static ble_addr_t s_addr = {BLE_ADDR_PUBLIC};

enum {
    GET_NAME_NUM = 0,
    GET_MODEL_NUM,
    GET_FWREV_NUM,
    GET_HWREV_NUM,
    GET_MNFR_NUM,
    GET_DATA_NUM,
    GET_VALUES
};

static const ble_uuid_t *uuids[] = {
    BLE_UUID16_DECLARE(0x2a00), /* Name */
    BLE_UUID16_DECLARE(0x2a24), /* Model */
    BLE_UUID16_DECLARE(0x2a26), /* Firmware revision */
    BLE_UUID16_DECLARE(0x2a27), /* Hardware revision */
    BLE_UUID16_DECLARE(0x2a29), /* Manufacturer */
    BLE_UUID128_DECLARE(0xa6, 0xa3, 0x7d, 0x99, 0xf2, 0x6f, 0x1a, 0x8a, 0x0c, 0x4b, 0x0a, 0x7a, 0xc1, 0xcc, 0xe0, 0xeb)
};

static EventGroupHandle_t s_sensor_event_group;
static int event_ble_handler(struct ble_gap_event *event, void *arg);

void app_sensor_addr_save(void)
{
    bool need_save = true;
    const char *part_name = hap_platform_keystore_get_nvs_partition_name();
    size_t addr_size = sizeof(s_addr);
    ble_addr_t addr;

    if (!hap_platform_keystore_get(part_name, SENSOR_KEYSTORE, SENSOR_ADDR_KEY,
    (uint8_t *)&addr, &addr_size) && addr_size == sizeof(s_addr)) {
        need_save = !!memcmp(&s_addr, &addr, sizeof(s_addr));
    }

    if (need_save) {
        hap_platform_keystore_set(part_name, SENSOR_KEYSTORE,
            SENSOR_ADDR_KEY, (uint8_t *)&s_addr, sizeof(s_addr));
        ESP_LOGD(TAG, "Sensor address saved");
    }
}

static void app_sensor_addr_load(void)
{
    const char *part_name = hap_platform_keystore_get_nvs_partition_name();
    size_t addr_size = sizeof(s_addr);

    if (!hap_platform_keystore_get(part_name, SENSOR_KEYSTORE, SENSOR_ADDR_KEY,
    (uint8_t *)&s_addr, &addr_size) && addr_size == sizeof(s_addr)) {
        xEventGroupSetBits(s_sensor_event_group, SENSOR_FOUND_BIT);
        ESP_LOGD(TAG, "Sensor address loaded");
    }
}

/* Convert revision string to Apple-compatible format: x[.y[.z]] */
static void fix_revision_string(char *str)
{
    char *p = str;

    /* Remove all symbols except dot or digits */
    for (uint16_t i = 0; i <= strlen(str); i++) {
        if (!str[i] || str[i] == '.' || (str[i] >= '0' && str[i] <= '9')) {
            *p++ = str[i];
        }
    }
}

static bool get_sensor_values(const uint8_t *val, uint16_t len)
{
    uint16_t raw_temp, mv;
    uint32_t battery;
    float temperature, humidity;

    if (len < 5) {
        ESP_LOGD(TAG, "Incorrect data size");
        return false;
    }

    ESP_LOGD(TAG, "Raw data: %02x %02x %02x %02x %02x",
        val[0], val[1], val[2], val[3], val[4]);

    raw_temp = val[0] | (uint16_t)(val[1] & 0x7f) << 8;
    temperature = (float)(raw_temp / 100) + (float)((raw_temp % 100) / 10) * 0.1;
    humidity = (float)val[2];

    if (temperature > 100.0 || humidity > 100.0) {
        ESP_LOGD(TAG, "Incorrect sensor values");
        return false;
    }

    mv = val[3] | (uint16_t)val[4] << 8;
    if (mv <= MIN_VBAT_MV)
        battery = 0;
    else if (mv >= MAX_VBAT_MV)
        battery = 100;
    else
        battery = (mv - MIN_VBAT_MV) / ((MAX_VBAT_MV - MIN_VBAT_MV) / 100);

    ESP_LOGD(TAG, "Temperature/Humidity=%f/%f Battery=%lu", temperature, humidity, battery);

    change_temperature(temperature);
    change_humidity(humidity);
    change_battery(battery);

    led_blink();

    return true;
}

static int read_attr_callback(uint16_t conn_handle, const struct ble_gatt_error *error,
    struct ble_gatt_attr *attr, void *arg)
{
    uint32_t num = (uint32_t)arg;
    static uint16_t data_handle;
    uint16_t om_len;

    ESP_LOGD(TAG, "Read by %u uuid #%lu: handle %u status %d", conn_handle,
        num, attr ? attr->handle : 0, error->status);

    /* Reading by uuid has second callback call! */
    if (error->status == BLE_HS_EDONE) {
#if CONFIG_LOG_DEFAULT_LEVEL >= ESP_LOG_DEBUG
        char buffer[BLE_UUID_STR_LEN];
        ESP_LOGD(TAG, "Reading %s done", ble_uuid_to_str(uuids[num], buffer));
#endif /* CONFIG_LOG_DEFAULT_LEVEL >= ESP_LOG_DEBUG */

        if (num < GET_DATA_NUM) {
            num++;
            ESP_ERROR_CHECK(ble_gattc_read_by_uuid(conn_handle,
                1, 128, uuids[num], read_attr_callback, (void *)num));
        } else {
            ESP_ERROR_CHECK(ble_gattc_read(conn_handle, data_handle,
                read_attr_callback, (void *)GET_VALUES));
        }

        return ESP_OK;
    }

    if (error->status != 0) {
        ESP_LOGW(TAG, "Read error, status %d!", error->status);
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return ESP_OK;
    }

    om_len = OS_MBUF_PKTLEN(attr->om);

    switch (num) {
    case GET_NAME_NUM:
        ble_hs_mbuf_to_flat(attr->om + attr->offset, g_name, MIN(om_len, sizeof(g_name)), NULL);
        ESP_LOGD(TAG, "NAME %s", g_name);
        break;
    case GET_MODEL_NUM:
        ble_hs_mbuf_to_flat(attr->om + attr->offset, g_model, MIN(om_len, sizeof(g_model)), NULL);
        ESP_LOGD(TAG, "MODEL %s", g_model);
        break;
    case GET_FWREV_NUM:
        ble_hs_mbuf_to_flat(attr->om + attr->offset, g_fwrev, MIN(om_len, sizeof(g_fwrev)), NULL);
        fix_revision_string(g_fwrev);
        ESP_LOGD(TAG, "FWREV %s", g_fwrev);
        break;
    case GET_HWREV_NUM:
        ble_hs_mbuf_to_flat(attr->om + attr->offset, g_hwrev, MIN(om_len, sizeof(g_hwrev)), NULL);
        fix_revision_string(g_hwrev);
        ESP_LOGD(TAG, "HWREV %s", g_hwrev);
        break;
    case GET_MNFR_NUM:
        ble_hs_mbuf_to_flat(attr->om + attr->offset, g_manufacturer, MIN(om_len, sizeof(g_manufacturer)), NULL);
        ESP_LOGD(TAG, "MANUFACTURER %s", g_manufacturer);
        sprintf(g_serial, "%02X%02X%02X%02X%02X%02X",
            s_addr.val[5], s_addr.val[4], s_addr.val[3],
            s_addr.val[2], s_addr.val[1], s_addr.val[0]);
        ESP_LOGD(TAG, "SERIAL %s", g_serial);
        xEventGroupSetBits(s_sensor_event_group, SENSOR_INFO_BIT);
        break;
    case GET_DATA_NUM:
        /* Only save attr handle because data in buffer is incorrect by now */
        data_handle = attr->handle;
        ESP_LOGD(TAG, "HANDLE %u", data_handle);
        break;
    case GET_VALUES:
        if (get_sensor_values(attr->om->om_data + attr->offset, om_len)) {
            xEventGroupSetBits(s_sensor_event_group, SENSOR_GOT_BIT);
        }
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        break;
    default:
        ESP_LOGW(TAG, "Unknown attr %lu, igonre it", num);
    }

    return ESP_OK;
}

static bool is_sensor_found(struct ble_gap_disc_desc *disc)
{
    struct ble_hs_adv_fields fields;

    ESP_LOGD(TAG, "Received advertising report %02x:%02x:%02x:%02x:%02x:%02x",
        disc->addr.val[5], disc->addr.val[4], disc->addr.val[3],
        disc->addr.val[2], disc->addr.val[1], disc->addr.val[0]);

    if (xEventGroupGetBits(s_sensor_event_group) & SENSOR_FOUND_BIT) {
        return !memcmp(&s_addr, &disc->addr, sizeof(s_addr));
    }

    if (disc->addr.val[5] != 0xa4 ||
    disc->addr.val[4] != 0xc1 || disc->addr.val[3] != 0x38)
        return false;

    if (ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data) != ESP_OK)
        return false;

    if (fields.svc_data_uuid16 == NULL || fields.svc_data_uuid16_len < 14 ||
    fields.svc_data_uuid16[0] != 0x95 || fields.svc_data_uuid16[1] != 0xfe)
        return false;

    if (memcmp(disc->addr.val, &fields.svc_data_uuid16[7], BLE_DEV_ADDR_LEN))
        return false;

    memcpy(&s_addr, &disc->addr, sizeof(s_addr));
    xEventGroupSetBits(s_sensor_event_group, SENSOR_FOUND_BIT);

    return true;
}

static void start_ble_scan(void)
{
    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params;

    ESP_LOGD(TAG, "Start to discover sensor...");

    /* Figure out address */
    ESP_ERROR_CHECK(ble_hs_id_infer_auto(BLE_ADDR_PUBLIC, &own_addr_type));

    /* Tell the controller to filter duplicates; we don't want to process
     * repeated advertisements from the same device.
     */
    disc_params.filter_duplicates = 1;

    /**
     * Perform a passive scan. I.e., don't send follow-up scan requests to
     * each advertiser.
     */
    disc_params.passive = 1;

    /* Use the best scan parameters. */
    disc_params.itvl = 800;
    disc_params.window = 800;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    /* 60 sec for discovery */
    ESP_ERROR_CHECK(ble_gap_disc(own_addr_type, 60000,
        &disc_params, event_ble_handler, NULL));
}

static void start_ble_connect(void)
{
    uint8_t own_addr_type;

    /* Scanning must be stopped before a connection */
    if (ble_gap_disc_active()) {
        ESP_ERROR_CHECK(ble_gap_disc_cancel());
    }

    /* Figure out public address */
    ESP_ERROR_CHECK(ble_hs_id_infer_auto(BLE_ADDR_PUBLIC, &own_addr_type));

    ESP_LOGD(TAG, "Start to connect to sensor...");

    /* 30 sec for connection */
    ESP_ERROR_CHECK(ble_gap_connect(own_addr_type, &s_addr,
        30000, NULL, event_ble_handler, NULL));
}

static int event_ble_handler(struct ble_gap_event *event, void *arg)
{
    ESP_LOGD(TAG, "Handle BLE event %d", event->type);

    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        if (is_sensor_found(&event->disc)) {
            start_ble_connect();
        }
        break;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        if (!(xEventGroupGetBits(s_sensor_event_group) & SENSOR_FOUND_BIT)) {
            ESP_LOGW(TAG, "Discovery completed, reason: %d",
                event->disc_complete.reason);
            start_ble_scan();
        }
        break;

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            ESP_LOGW(TAG, "Connect failed, status: %d",
                event->connect.status);
            start_ble_scan();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        if (!(xEventGroupGetBits(s_sensor_event_group) & SENSOR_GOT_BIT)) {
            ESP_LOGW(TAG, "Disconnect, reason: %d", event->disconnect.reason);
            start_ble_scan();
        }
        break;

    case BLE_GAP_EVENT_LINK_ESTAB:
        if (event->link_estab.status == 0) {
            ESP_LOGD(TAG, "Connection established...");
            if (xEventGroupGetBits(s_sensor_event_group) & SENSOR_INFO_BIT) {
                ESP_ERROR_CHECK(ble_gattc_read_by_uuid(event->link_estab.conn_handle,
                    1, 128, uuids[GET_DATA_NUM], read_attr_callback, (void *)GET_DATA_NUM));
            } else {
                ESP_ERROR_CHECK(ble_gattc_read_by_uuid(event->link_estab.conn_handle,
                    1, 128, uuids[0], read_attr_callback, NULL));
            }
        } else {
            ESP_LOGW(TAG, "Connection not established, status: %d",
                event->connect.status);
            if (!ble_gap_disc_active()) {
                start_ble_scan();
            }
        }
        break;

    default:
        ESP_LOGD(TAG, "Event %d ignored", event->type);
    }

    return ESP_OK;
}

static void on_sync_ble(void)
{
    ESP_LOGD(TAG, "Bluetooth initialized");

    /* Make sure we have proper identity address set (public preferred) */
    ESP_ERROR_CHECK(ble_hs_util_ensure_addr(BLE_ADDR_PUBLIC));

    start_ble_scan();
}

static void on_timer(void* arg)
{
    ESP_LOGD(TAG, "Time to refresh sensor values");

    if (xEventGroupGetBits(s_sensor_event_group) & SENSOR_FOUND_BIT) {
        if (!ble_gap_conn_active()) {
            xEventGroupClearBits(s_sensor_event_group, SENSOR_GOT_BIT);
            start_ble_connect();
        }
    } else {
        if (!ble_gap_disc_active()) {
            start_ble_scan();
        }
    }
}

static void host_nimble_task(void *param)
{
    ESP_LOGD(TAG, "BLE host task started.");

    /* The function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    ESP_LOGW(TAG, "BLE host task finished.");
    nimble_port_freertos_deinit();
}

esp_err_t app_sensor_init(TickType_t ticks_to_wait)
{
    uint8_t mac[6];
    char unique_hostname[32];
    esp_timer_handle_t timer;
    esp_timer_create_args_t timer_args = {
        .dispatch_method = ESP_TIMER_TASK,
        .callback = &on_timer,
        .name = "GetSensorValuesPeriodic"
    };

    ESP_LOGD(TAG, "Init BLE connection to sensor");

    s_sensor_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(nimble_port_init());

    /* Configure the host */
    ble_hs_cfg.sync_cb = on_sync_ble;

    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_BT));
    sprintf(unique_hostname, "hap-gateway-%02x%02x%02x%02x%02x%02x",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_ERROR_CHECK(ble_svc_gap_device_name_set(unique_hostname));

    /* Try to load sensor address from NVS */
    app_sensor_addr_load();

    nimble_port_freertos_init(host_nimble_task);

    xEventGroupWaitBits(s_sensor_event_group, SENSOR_INFO_BIT,
        pdFALSE, pdFALSE, ticks_to_wait);

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, SENSOR_GET_TIMEOUT));

    return ESP_OK;
}

void app_sensor_reset(bool full)
{
    int mask = SENSOR_INFO_BIT | SENSOR_GOT_BIT;

    if (full) {
        hap_platform_keystore_delete_namespace(
            hap_platform_keystore_get_nvs_partition_name(),
            SENSOR_KEYSTORE);
        mask |= SENSOR_FOUND_BIT;
        ESP_LOGD(TAG, "Sensor address deleted");
    }

    xEventGroupClearBits(s_sensor_event_group, mask);

    if (ble_gap_conn_active()) {
        ble_gap_conn_cancel();
    }

    if (ble_gap_disc_active()) {
        ble_gap_disc_cancel();
    }
}
