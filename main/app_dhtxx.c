/* Support for DTH11/12/21/22 temperature and humidity GPIO sensor */

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <rom/ets_sys.h>
#include <esp_timer.h>
#include <esp_log.h>

#define SENSOR_GET_TIMEOUT (60 * 1000000) /* 1 min in microseconds */

extern char g_name[32];
extern char g_model[32];

extern void led_blink(void);
extern void change_temperature(float temperature);
extern void change_humidity(float humidity);

static const char *TAG = "DHT";

void app_sensor_addr_save(void)
{
}

static uint32_t wait_gpio_level(uint32_t usec, uint32_t level)
{
    for (uint32_t i = 1; i <= usec; i++) {
        if (gpio_get_level(CONFIG_SENSOR_GPIO) != level)
            return i;

        ets_delay_us(1);
    }

    return 0;
}

static bool get_sensor_values(void)
{
    TickType_t ticks;
    uint8_t val[5] = { 0, 0, 0, 0, 0 };
    float temperature, humidity;

    /* At first we send start signal to sensor */
    gpio_reset_pin(CONFIG_SENSOR_GPIO);
    gpio_set_direction(CONFIG_SENSOR_GPIO, GPIO_MODE_OUTPUT);

    /* Low level ~18ms */
    ticks = xTaskGetTickCount();
    gpio_set_level(CONFIG_SENSOR_GPIO, 0);
    vTaskDelayUntil(&ticks, pdMS_TO_TICKS(20));

    /* High level ~20-40us */
    gpio_set_level(CONFIG_SENSOR_GPIO, 1);
    ets_delay_us(40);

    /* Receive response from sensor */
    gpio_set_direction(CONFIG_SENSOR_GPIO, GPIO_MODE_INPUT);

    /* Wait for next step ~80us */
    if (!wait_gpio_level(85, 0)) {
        ESP_LOGD(TAG, "Incorrect start low level");
        return false;
    }

    /* Wait for next step ~80us */
    if (!wait_gpio_level(85, 1)) {
        ESP_LOGD(TAG, "Incorrect start high level");
        return false;
    }

    for(uint32_t i = 0; i < 40; i++) {
        /* Wait for every bit ~50us */
        if (!wait_gpio_level(55, 0)) {
            ESP_LOGD(TAG, "Incorrect bit %lu", i);
            return false;
        }

        /* bit0 = ~26-28us, bit1 = ~70us */
        if (wait_gpio_level(75, 1) > 30) {
            val[i / 8] |= 1 << (7 - i % 8);
        }
    }

    ESP_LOGD(TAG, "Raw data: %02x %02x %02x %02x %02x",
        val[0], val[1], val[2], val[3], val[4]);

    if (val[4] != (val[0] + val[1] + val[2] + val[3])) {
        ESP_LOGD(TAG, "Incorrect CRC");
        return false;
    }

#ifdef CONFIG_SENSOR_DHT11
    temperature = (float)val[2] + (float)(val[3] % 10) * 0.1;
    humidity = (float)val[0];
#else /* CONFIG_SENSOR_DHT11 */
    temperature = (float)(val[3] | (uint16_t)val[2] << 8) * 0.1;
    humidity = (float)((val[1] | (uint16_t)val[0] << 8) / 10);
#endif /* CONFIG_SENSOR_DHT11 */

    if (temperature > 100.0 || humidity > 100.0) {
        ESP_LOGD(TAG, "Incorrect sensor values");
        return false;
    }

    ESP_LOGW(TAG, "Temperature/Humidity=%f/%f", temperature, humidity);

    change_temperature(temperature);
    change_humidity(humidity);

    led_blink();

    return true;
}

static void on_timer(void* arg)
{
    ESP_LOGD(TAG, "Time to refresh sensor values");

    get_sensor_values();
}

esp_err_t app_sensor_init(TickType_t ticks_to_wait)
{
    esp_timer_handle_t timer;
    esp_timer_create_args_t timer_args = {
        .dispatch_method = ESP_TIMER_TASK,
        .callback = &on_timer,
        .name = "GetSensorValuesPeriodic"
    };

    ESP_LOGD(TAG, "Init connection to sensor");

    while (!get_sensor_values()) {
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

#ifdef CONFIG_SENSOR_DHT11
    strcpy(g_name, "DHT11");
    strcpy(g_model, "DHT11");
#else /* CONFIG_SENSOR_DHT11 */
    strcpy(g_name, "DHT22");
    strcpy(g_model, "DHT2");
#endif /* CONFIG_SENSOR_DHT11 */

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, SENSOR_GET_TIMEOUT));

    return ESP_OK;
}

void app_sensor_reset(bool full)
{
    (void)full;
}
