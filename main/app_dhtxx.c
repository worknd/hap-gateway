/* Support for DHT11/12/21/22 temperature and humidity GPIO sensor */

#include <math.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <rom/ets_sys.h>
#include <esp_timer.h>
#include <esp_log.h>

/* Convert minutes in microseconds */
#define SENSOR_GET_TIMEOUT (CONFIG_SENSOR_INQUERY_TIMEOUT * 60 * 1000000)

extern char g_name[32];
extern char g_model[32];

extern void led_blink(void);
extern void change_temperature(float temperature);
extern void change_humidity(float humidity);

static const char *TAG = "DHT";

void app_sensor_addr_save(void)
{
}

static uint32_t wait_gpio_level(gpio_num_t gpio_num, uint32_t usec, uint32_t level)
{
    for (uint32_t i = 1; i <= usec; i++) {
        if (gpio_get_level(gpio_num) != level)
            return i;

        ets_delay_us(1);
    }

    return 0;
}

static bool get_sensor_values(gpio_num_t gpio_num)
{
    TickType_t ticks;
    uint8_t val[5] = { 0, 0, 0, 0, 0 };
    int32_t raw_temperature;
    uint32_t raw_humidity;
    float temperature, humidity;

    /* At first we send start signal to sensor */
    gpio_reset_pin(gpio_num);
    gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT);

    /* Low level ~18ms */
    ticks = xTaskGetTickCount();
    gpio_set_level(gpio_num, 0);
    vTaskDelayUntil(&ticks, pdMS_TO_TICKS(20));

    /* High level ~20-40us */
    gpio_set_level(gpio_num, 1);
    ets_delay_us(40);

    /* Receive response from sensor */
    gpio_set_direction(gpio_num, GPIO_MODE_INPUT);

    /* Wait for next step ~80us */
    if (!wait_gpio_level(gpio_num, 85, 0)) {
        ESP_LOGD(TAG, "Incorrect start low level");
        return false;
    }

    /* Wait for next step ~80us */
    if (!wait_gpio_level(gpio_num, 85, 1)) {
        ESP_LOGD(TAG, "Incorrect start high level");
        return false;
    }

    for(uint32_t i = 0; i < 40; i++) {
        /* Wait for every bit ~50us */
        if (!wait_gpio_level(gpio_num, 55, 0)) {
            ESP_LOGD(TAG, "Incorrect bit %lu", i);
            return false;
        }

        /* bit0 = ~26-28us, bit1 = ~70us */
        if (wait_gpio_level(gpio_num, 75, 1) > 30) {
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
    raw_temperature = (int32_t)val[2] * 10 + val[3] & 0x7f;
    if (val[3] & 0x80)
        raw_temperature = -raw_temperature;
    raw_humidity = (uint32_t)val[0] * 10 + val[1];
#else /* CONFIG_SENSOR_DHT11 */
    raw_temperature = val[3] | ((int32_t)(val[2] & 0x7f) << 8);
    if (val[2] & 0x80)
        raw_temperature = -raw_temperature;
    raw_humidity = val[1] | ((uint32_t)val[0] << 8);
#endif /* CONFIG_SENSOR_DHT11 */

    temperature = raw_temperature * 0.1;
    humidity = floorf(raw_humidity * 0.1 + 0.5);
    ESP_LOGD(TAG, "Temperature/Humidity=%f/%f", temperature, humidity);

    if (temperature > 100.0 || temperature < -50.0 || humidity > 100.0) {
        ESP_LOGD(TAG, "Incorrect sensor values");
        return false;
    }

    change_temperature(temperature);
    change_humidity(humidity);

    led_blink();

    return true;
}

static void on_timer(void* arg)
{
    ESP_LOGD(TAG, "Time to refresh sensor values");

    get_sensor_values(CONFIG_SENSOR_GPIO);
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

    while (!get_sensor_values(CONFIG_SENSOR_GPIO)) {
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
