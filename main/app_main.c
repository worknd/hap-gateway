/* HomeKit gateway for temperature and humidity sensors */

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_pm.h>

#include <hap.h>
#include <hap_apple_servs.h>
#include <hap_apple_chars.h>

#include <iot_button.h>
#include <qrcode.h>

extern void hap_configure_unique_param(hap_unique_param_t param);

extern void app_wifi_init(void);
extern esp_err_t app_wifi_start(TickType_t ticks_to_wait);
extern esp_err_t app_sensor_init(TickType_t ticks_to_wait);
extern void app_sensor_addr_save(void);
extern void app_sensor_reset(bool full);

char g_name[32] = "?";
char g_manufacturer[32] = "?";
char g_model[32] = "?";
char g_serial[16]= "1";
char g_fwrev[16] = "1";
char g_hwrev[16] = "1";

float g_temperature = 0.0;
float g_humidity = 50.0;
uint32_t g_battery = 100;

static const char *TAG = "HGW";

#define HGW_TASK_PRIORITY  1
#define HGW_TASK_STACKSIZE 6 * 1024

/* Reset network credentials if button is pressed for more than 3 seconds and then released */
#define RESET_NETWORK_BUTTON_TIMEOUT        3

/* Reset to factory if button is pressed and held for more than 10 seconds */
#define RESET_TO_FACTORY_BUTTON_TIMEOUT     10

/* The button "Boot" will be used as the Reset button for the example */
#define RESET_GPIO  CONFIG_BOOT_GPIO

/* Define status of "low battery" as less 10% battery level */
#define LOW_BATTERY(level) ((level) < 10 ? 1 : 0)

void led_blink(void)
{
    gpio_reset_pin(CONFIG_BLINK_GPIO);

    /* Set the GPIO as a push/pull output */
    gpio_set_direction(CONFIG_BLINK_GPIO, GPIO_MODE_OUTPUT);

    for (int i = 0; i < 3; i++) {
        gpio_set_level(CONFIG_BLINK_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(200));

        gpio_set_level(CONFIG_BLINK_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    gpio_set_direction(CONFIG_BLINK_GPIO, GPIO_MODE_DISABLE);
    gpio_set_pull_mode(CONFIG_BLINK_GPIO, GPIO_FLOATING);
}

/**
 * @brief The change sensor values callback handlers.
 */
void change_temperature(float temperature)
{
    if (temperature != g_temperature) {
        hap_acc_t *accessory;
        hap_serv_t *service;
        hap_char_t *chr;

        g_temperature = temperature;

        accessory = hap_get_first_acc();
        if (!accessory)
            return;

        service = hap_acc_get_serv_by_uuid(accessory, HAP_SERV_UUID_TEMPERATURE_SENSOR);
        if (!service)
            return;

        chr = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_CURRENT_TEMPERATURE);
        if (chr) {
            hap_val_t new_val = { .f = g_temperature };
            hap_char_update_val(chr, &new_val);
            ESP_LOGD(TAG, "Updated temperature value: %f", new_val.f);
        }
    }
}

void change_humidity(float humidity)
{
    if (humidity != g_humidity) {
        hap_acc_t *accessory;
        hap_serv_t *service;
        hap_char_t *chr;

        g_humidity = humidity;

        accessory = hap_get_first_acc();
        if (!accessory)
            return;

        service = hap_acc_get_serv_by_uuid(accessory, HAP_SERV_UUID_HUMIDITY_SENSOR);
        if (!service)
            return;

        chr = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_CURRENT_RELATIVE_HUMIDITY);
        if (chr) {
            hap_val_t new_val = { .f = g_humidity };
            hap_char_update_val(chr, &new_val);
            ESP_LOGD(TAG, "Updated humidity value: %f", new_val.f);
        }
    }
}

void change_battery(uint32_t battery)
{
    if (battery != g_battery) {
        hap_acc_t *accessory;
        hap_serv_t *service;
        hap_char_t *chr;
        uint32_t status_low = LOW_BATTERY(g_battery);

        g_battery = battery;

        accessory = hap_get_first_acc();
        if (!accessory)
            return;

        service = hap_acc_get_serv_by_uuid(accessory, HAP_SERV_UUID_BATTERY_SERVICE);
        if (!service)
            return;

        chr = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_BATTERY_LEVEL);
        if (chr) {
            hap_val_t new_val = { .u = g_battery };
            hap_char_update_val(chr, &new_val);
            ESP_LOGD(TAG, "Updated battery level: %lu", new_val.u);
        }

        if (LOW_BATTERY(g_battery) != status_low) {
            chr = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_STATUS_LOW_BATTERY);
            if (chr) {
                hap_val_t new_val = { .u = LOW_BATTERY(g_battery) };
                hap_char_update_val(chr, &new_val);
                ESP_LOGD(TAG, "Updated low battery status: %lu", new_val.u);
            }
        }
    }
}

/**
 * @brief The network reset button callback handler.
 * Useful for testing the Wi-Fi re-configuration.
 */
static void reset_network_handler(void* arg)
{
    led_blink();
    app_sensor_reset(false);
    hap_reset_network();
}

/**
 * @brief The factory reset button callback handler.
 */
static void reset_to_factory_handler(void* arg)
{
    led_blink();
    app_sensor_reset(true);
    hap_reset_to_factory();
}

/**
 * The Reset button GPIO initialisation function.
 * Same button will be used for resetting Wi-Fi network as well as for reset to factory based on
 * the time for which the button is pressed.
 */
static void reset_key_init(uint32_t key_gpio_pin)
{
    button_handle_t handle = iot_button_create(key_gpio_pin, BUTTON_ACTIVE_LOW);

    iot_button_add_on_release_cb(handle, RESET_NETWORK_BUTTON_TIMEOUT, reset_network_handler, NULL);
    iot_button_add_on_press_cb(handle, RESET_TO_FACTORY_BUTTON_TIMEOUT, reset_to_factory_handler, NULL);
}

/* Mandatory identify routine for the accessory.
 * In a real accessory, something like LED blink should be implemented
 * got visual identification
 */
static int hgw_identify(hap_acc_t *ha)
{
    ESP_LOGD(TAG, "Accessory identified");

    led_blink();

    return HAP_SUCCESS;
}

#if CONFIG_LOG_DEFAULT_LEVEL >= ESP_LOG_INFO
/*
 * An optional HomeKit Event handler which can be used to track HomeKit
 * specific events.
 */
static void hgw_hap_event_handler(void* arg, esp_event_base_t event_base, int32_t event, void *data)
{
    switch(event) {
        case HAP_EVENT_PAIRING_STARTED:
            ESP_LOGI(TAG, "Pairing Started");
            break;
        case HAP_EVENT_PAIRING_ABORTED:
            ESP_LOGI(TAG, "Pairing Aborted");
            break;
        case HAP_EVENT_CTRL_PAIRED:
            ESP_LOGI(TAG, "Controller %s Paired. Controller count: %d",
                (char *)data, hap_get_paired_controller_count());
            break;
        case HAP_EVENT_CTRL_UNPAIRED:
            ESP_LOGI(TAG, "Controller %s Removed. Controller count: %d",
                (char *)data, hap_get_paired_controller_count());
            break;
        case HAP_EVENT_CTRL_CONNECTED:
            ESP_LOGI(TAG, "Controller %s Connected", (char *)data);
            break;
        case HAP_EVENT_CTRL_DISCONNECTED:
            ESP_LOGI(TAG, "Controller %s Disconnected", (char *)data);
            break;
        case HAP_EVENT_ACC_REBOOTING:
            ESP_LOGI(TAG, "Accessory Rebooting (Reason: %s)",
                data ? (char *)data : "null");
            break;
        case HAP_EVENT_PAIRING_MODE_TIMED_OUT:
            ESP_LOGI(TAG, "Pairing Mode timed out. Please reboot the device.");
            break;
        default: ;
    }
}
#endif /* CONFIG_LOG_DEFAULT_LEVEL >= ESP_LOG_INFO */

/* A callback for handling a read on the characteristics.
 * Read routines are generally not required as the value is available with th HAP core
 * when it is updated from write routines. For external triggers (like fan switched on/off
 * using physical button), accessories should explicitly call hap_char_update_val()
 * instead of waiting for a read request.
 */
static int hgw_read(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv)
{
    const char *chr_uuid = hap_char_get_type_uuid(hc);
    const char *serv = serv_priv;
    const char *ctrl = hap_req_get_ctrl_id(read_priv);
    const hap_val_t *cur_val = hap_char_get_val(hc);

    ESP_LOGD(TAG, "Received read %s(%s) from %s", serv ? serv : "?",
        chr_uuid ? chr_uuid : "?", ctrl ? ctrl : "?");

    if (!strcmp(chr_uuid, HAP_CHAR_UUID_CURRENT_TEMPERATURE)) {
        if (cur_val->f != g_temperature) {
            hap_val_t new_val = { .f = g_temperature };
            hap_char_update_val(hc, &new_val);
            ESP_LOGD(TAG, "Updated temperature value: %f", new_val.f);
        }
    } else if (!strcmp(chr_uuid, HAP_CHAR_UUID_CURRENT_RELATIVE_HUMIDITY)) {
        if (cur_val->f != g_humidity) {
            hap_val_t new_val = { .f = g_humidity };
            hap_char_update_val(hc, &new_val);
            ESP_LOGD(TAG, "Updated humidity value: %f", new_val.f);
        }
    } else if (!strcmp(chr_uuid, HAP_CHAR_UUID_BATTERY_LEVEL)) {
        if (cur_val->u != g_battery) {
            hap_val_t new_val = { .u = g_battery };
            hap_char_update_val(hc, &new_val);
            ESP_LOGD(TAG, "Updated battery level: %lu", new_val.u);
        }
    } else if (!strcmp(chr_uuid, HAP_CHAR_UUID_STATUS_LOW_BATTERY)) {
        uint32_t status_low = LOW_BATTERY(g_battery);
        if (cur_val->u != status_low) {
            hap_val_t new_val = { .u = status_low };
            hap_char_update_val(hc, &new_val);
            ESP_LOGD(TAG, "Updated low battery status: %lu", new_val.u);
        }
    } else if (!strcmp(chr_uuid, HAP_CHAR_UUID_CHARGING_STATE)) {
        ESP_LOGD(TAG, "Charging State not updated");
    } else {
        *status_code = HAP_STATUS_RES_ABSENT;
        return HAP_FAIL;
    }

    *status_code = HAP_STATUS_SUCCESS;
    return HAP_SUCCESS;
}

/* The main thread for handling the Accessory */
static void hgw_thread_entry(void *arg)
{
    hap_acc_t *accessory;
    hap_serv_t *temperature_service;
    hap_serv_t *humidity_service;
    hap_serv_t *battery_service;
    char *setup_payload;

    /* Initialise the mandatory parameters for Accessory which will be added as
     * the mandatory services internally
     */
    hap_acc_cfg_t cfg = {
        .name = g_name,
        .manufacturer = g_manufacturer,
        .model = g_model,
        .serial_num = g_serial,
        .fw_rev = g_fwrev,
        .hw_rev = g_hwrev,
        .pv = "1.1.0",
        .identify_routine = hgw_identify,
        .cid = HAP_CID_SENSOR,
    };
    /* Create accessory object */
    accessory = hap_acc_create(&cfg);

    /* Add a dummy Product Data */
    uint8_t product_data[] = {'E','S','P','3','2','H','A','P'};
    hap_acc_add_product_data(accessory, product_data, sizeof(product_data));

    /* Add Wi-Fi Transport service required for HAP Spec R16 */
    hap_acc_add_wifi_transport_service(accessory, 0);

    /* Create the Services. Include the "name" since this is a user visible service  */
    temperature_service = hap_serv_temperature_sensor_create(g_temperature);
    hap_serv_add_char(temperature_service, hap_char_status_low_battery_create(LOW_BATTERY(g_battery)));
    humidity_service = hap_serv_humidity_sensor_create(g_humidity);
    hap_serv_add_char(temperature_service, hap_char_status_low_battery_create(LOW_BATTERY(g_battery)));
    battery_service = hap_serv_battery_service_create(g_battery, 2, LOW_BATTERY(g_battery));

    /* Add service name as priv data */
    hap_serv_set_priv(temperature_service, "temperature");
    hap_serv_set_priv(humidity_service, "humidity");
    hap_serv_set_priv(battery_service, "battery");

    /* Add links to the Services */
    hap_serv_mark_primary(temperature_service);
    hap_serv_link_serv(humidity_service, temperature_service);
    hap_serv_link_serv(battery_service, temperature_service);
    hap_serv_link_serv(battery_service, humidity_service);

    /* Set the read callback for the services */
    hap_serv_set_read_cb(temperature_service, hgw_read);
    hap_serv_set_read_cb(humidity_service, hgw_read);
    hap_serv_set_read_cb(battery_service, hgw_read);

    /* Add the Services to the Accessory Object */
    hap_acc_add_serv(accessory, temperature_service);
    hap_acc_add_serv(accessory, humidity_service);
    hap_acc_add_serv(accessory, battery_service);

    /* Add the Accessory to the HomeKit Database */
    hap_add_accessory(accessory);

    /* Query the controller count (just for information) */
    ESP_LOGI(TAG, "Accessory is paired with %d controllers",
        hap_get_paired_controller_count());

    /* Unique Setup code of the format xxx-xx-xxx. Default: 111-22-333 */
    hap_set_setup_code(CONFIG_EXAMPLE_SETUP_CODE);
    /* Unique four character Setup Id. Default: ES32 */
    hap_set_setup_id(CONFIG_EXAMPLE_SETUP_ID);
    /* Make payload QR code */
    setup_payload = esp_hap_get_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, false, cfg.cid);
    if (setup_payload) {
        ESP_LOGI(TAG, "-----QR Code for HomeKit (%s)-----", setup_payload);
        qrcode_display(setup_payload);
        free(setup_payload);
    }

    /* Initialize Wi-Fi */
    app_wifi_init();

#if CONFIG_LOG_DEFAULT_LEVEL >= ESP_LOG_INFO
    /* Register an event handler for HomeKit specific events.
     * All event handlers should be registered only after app_wifi_init()
     */
    esp_event_handler_register(HAP_EVENT, ESP_EVENT_ANY_ID, &hgw_hap_event_handler, NULL);
#endif /* CONFIG_LOG_DEFAULT_LEVEL >= ESP_LOG_INFO */

    /* After all the initializations are done, start the HAP core */
    hap_start();

    /* Save sensor address in HAP keystore */
    app_sensor_addr_save();

    /* Start Wi-Fi */
    app_wifi_start(portMAX_DELAY);

    led_blink();

    /* The task ends here. The read/write callbacks will be invoked by the HAP Framework */
    vTaskDelete(NULL);
}

void app_main()
{
    esp_pm_config_t pm_config = {
        .max_freq_mhz = 160,
        .min_freq_mhz = 80,
        .light_sleep_enable = false
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));

    /* Configure HomeKit core to make the Accessory name (and thus the WAC SSID) unique,
     * instead of the default configuration wherein only the WAC SSID is made unique.
     */
    hap_configure_unique_param(UNIQUE_NAME);

    /* Initialize the HAP core */
    hap_init(HAP_TRANSPORT_WIFI);

    /* Register a common button for reset Wi-Fi network and reset to factory */
    reset_key_init(RESET_GPIO);

    /* Search sensor */
    app_sensor_init(portMAX_DELAY);

    xTaskCreate(hgw_thread_entry, TAG, HGW_TASK_STACKSIZE, NULL, HGW_TASK_PRIORITY, NULL);
}
