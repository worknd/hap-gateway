/* Support for AHT10/20 temperature and humidity I2C sensor */

#include <math.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c_master.h>
#include <esp_timer.h>
#include <esp_log.h>

#undef ESP_LOGD
#define ESP_LOGD ESP_LOGI

/* Convert minutes in microseconds */
#define SENSOR_GET_TIMEOUT (CONFIG_SENSOR_INQUERY_TIMEOUT * 60 * 1000000)

#define ROUND_TO_DECIMAL(f) (floorf((f) * 10.0 + 0.5) * 0.1)
#define ROUND_TO_INTEGER(f) floorf((f) + 0.5)

#define AHTX0_CMD_STATUS        0x71
#define AHTX0_CMD_MEASURE       0xac
#define AHTX0_CMD_SOFTRESET     0xba
#define AHT20_CMD_CALIBRATE     0xbe
#define AHT10_CMD_CALIBRATE     0xe1
#define AHTX0_STATUS_BUSY       0x80
#define AHTX0_STATUS_CALIBRATED 0x08

#define BMP280_REGISTER_DIG_T1    0x88
#define BMP280_REGISTER_DIG_T2    0x8A
#define BMP280_REGISTER_DIG_T3    0x8C
#define BMP280_REGISTER_DIG_P1    0x8E
#define BMP280_REGISTER_DIG_P2    0x90
#define BMP280_REGISTER_DIG_P3    0x92
#define BMP280_REGISTER_DIG_P4    0x94
#define BMP280_REGISTER_DIG_P5    0x96
#define BMP280_REGISTER_DIG_P6    0x98
#define BMP280_REGISTER_DIG_P7    0x9A
#define BMP280_REGISTER_DIG_P8    0x9C
#define BMP280_REGISTER_DIG_P9    0x9E
#define BMP280_REGISTER_CHIPID    0xD0
#define BMP280_REGISTER_VERSION   0xD1
#define BMP280_REGISTER_SOFTRESET 0xE0
#define BMP280_REGISTER_CAL26     0xE1 /* R calibration = 0xE1-0xF0 */
#define BMP280_REGISTER_STATUS    0xF3
#define BMP280_REGISTER_CONTROL   0xF4
#define BMP280_REGISTER_CONFIG    0xF5
#define BMP280_REGISTER_PRESSURE  0xF7
#define BMP280_REGISTER_TEMP      0xFA
#define BMP280_MODE_SLEEP         0x00
#define BMP280_MODE_FORCED        0x01
#define BMP280_MODE_NORMAL        0x03
#define BMP280_SOFTRESET          0xb6

#define BMP280_CONFIG(s5,s2,s0) (((s5) << 5) | ((s2) << 2) | (s0))

extern char g_name[32];
extern char g_model[32];

extern void led_blink(void);
extern void change_temperature(float temperature);
extern void change_humidity(float humidity);

static const char *TAG = "AHT";

static i2c_master_bus_handle_t s_bus_handle;
static i2c_master_dev_handle_t s_ahtx0_handle;
static i2c_master_dev_handle_t s_bmp280_handle;

static uint8_t bmp_id;
static uint16_t dig_t1, dig_p1;
static int16_t dig_t2, dig_t3, dig_p[8];
static int32_t temp_fine;

void app_sensor_addr_save(void)
{
}

static void fini_ahtx0(void)
{
    if (s_ahtx0_handle) {
        if (i2c_master_bus_rm_device(s_ahtx0_handle) != ESP_OK) {
            ESP_LOGW(TAG, "Deinit I2C device AHTx0 failed");
        }

        s_ahtx0_handle = 0;
    }
}

static void fini_bmp280(void)
{
    if (s_bmp280_handle) {
        if (i2c_master_bus_rm_device(s_bmp280_handle) != ESP_OK) {
            ESP_LOGW(TAG, "Deinit I2C device BMP280 failed");
        }

        s_bmp280_handle = 0;
    }
}

static void fini_i2c_bus(void)
{
    if (s_bus_handle) {
        fini_ahtx0();
        fini_bmp280();

        if (i2c_del_master_bus(s_bus_handle) != ESP_OK) {
            ESP_LOGW(TAG, "Deinit I2C bus failed");
        }

        s_bus_handle = 0;
    }
}

static bool init_i2c_bus(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = CONFIG_I2C_PORT,
        .scl_io_num = CONFIG_I2C_SCL_GPIO,
        .sda_io_num = CONFIG_I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    /* Deinit the bus and all devices */
    fini_i2c_bus();

    /* Wait for power up ~40ms */
    vTaskDelay(pdMS_TO_TICKS(40));

    if (i2c_new_master_bus(&bus_cfg, &s_bus_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Init I2C bus failed");
        return false;
    }

    ESP_LOGD(TAG, "Init I2C bus complete");

    return true;
}

static bool wait_ahtx0_status(uint8_t status, uint8_t mask, uint32_t timeout)
{
    TickType_t start = xTaskGetTickCount();
    TickType_t end = start + pdMS_TO_TICKS(timeout);
    uint8_t val, cmd = AHTX0_CMD_STATUS;

    do {
        if (i2c_master_transmit_receive(s_ahtx0_handle, &cmd, 1, &val, 1, 10) == ESP_OK) {
            if ((val & mask) == status)
                return true;
        }

        vTaskDelayUntil(&start, pdMS_TO_TICKS(10));
    } while (xTaskGetTickCount() < end);

    return false;
}

static bool init_ahtx0(bool reset)
{
    uint8_t val[3];

    if (!s_ahtx0_handle) {
        i2c_device_config_t ahtx0_cfg = {
            .scl_speed_hz = CONFIG_I2C_FREQUENCY,
            .device_address = CONFIG_I2C_AHTX0_ADDRESS,
        };

        if (!s_bus_handle) {
            if (!init_i2c_bus())
                return false;
        }

        if (i2c_master_bus_add_device(s_bus_handle, &ahtx0_cfg, &s_ahtx0_handle) != ESP_OK) {
            ESP_LOGE(TAG, "Add device failed");
            return false;
        }

        if (reset) {
            val[0] = AHTX0_CMD_SOFTRESET;
            if (i2c_master_transmit(s_ahtx0_handle, val, 1, 100) != ESP_OK) {
                ESP_LOGE(TAG, "Soft reset failed");
                return false;
            }

            /* Wait for soft reset complete ~20ms */
            vTaskDelay(pdMS_TO_TICKS(20));
            ESP_LOGD(TAG, "Sensor soft reset complete");
        }
    }

    if (!wait_ahtx0_status(AHTX0_STATUS_CALIBRATED,
    AHTX0_STATUS_BUSY | AHTX0_STATUS_CALIBRATED, 80)) {
        val[0] = AHT20_CMD_CALIBRATE;
        val[1] = 0x08;
        val[2] = 0;
        if (i2c_master_transmit(s_ahtx0_handle, val, 3, 100) != ESP_OK) {
            /* Try command for AHT10 sensor */
            val[0] = AHT10_CMD_CALIBRATE;
            if (i2c_master_transmit(s_ahtx0_handle, val, 3, 100) != ESP_OK) {
                ESP_LOGE(TAG, "Calibration failed");
                return false;
            }
        }

        if (!wait_ahtx0_status(AHTX0_STATUS_CALIBRATED,
        AHTX0_STATUS_BUSY | AHTX0_STATUS_CALIBRATED, 100)) {
            ESP_LOGE(TAG, "Calibration timeout expired");
            return false;
        }

        ESP_LOGD(TAG, "Sensor calibration complete");
    }

    return true;
}

static bool read_ahtx0(float *ptemp, float *phumi)
{
    uint8_t val[7];
    int32_t raw_temperature;
    uint32_t raw_humidity;
    float temperature, humidity;

    if (!s_ahtx0_handle) {
        if (!init_ahtx0(false) && !init_ahtx0(true))
            return false;
    }

    if (!wait_ahtx0_status(0, AHTX0_STATUS_BUSY, 80)) {
        ESP_LOGE(TAG, "Busy timeout expired");
        return false;
    }

    val[0] = AHTX0_CMD_MEASURE;
    val[1] = 0x33;
    val[2] = 0;
    if (i2c_master_transmit(s_ahtx0_handle, val, 3, 100) != ESP_OK) {
        ESP_LOGE(TAG, "Measurement failed");
        return false;
    }

    /* Wait for measurement complete ~80ms */
    vTaskDelay(pdMS_TO_TICKS(80));

    if (!wait_ahtx0_status(0, AHTX0_STATUS_BUSY, 100)) {
        ESP_LOGE(TAG, "Measurement timeout expired");
        return false;
    }

    if (i2c_master_receive(s_ahtx0_handle, val, 7, 100) != ESP_OK) {
        ESP_LOGE(TAG, "Reading sensor values failed");
        return false;
    }

    ESP_LOGD(TAG, "Raw AHTx0 data: %02x %02x %02x %02x %02x %02x",
        val[0], val[1], val[2], val[3], val[4], val[5]);

    if (!wait_ahtx0_status(0, AHTX0_STATUS_BUSY, 100)) {
        ESP_LOGW(TAG, "Finish timeout expired");
    }

    raw_temperature = ((int32_t)(val[3] & 0xf) << 16) |
        ((int32_t)val[4] << 8) | val[5];
    raw_humidity = ((uint32_t)val[1] << 12) |
        ((uint32_t)val[2] << 4) | (val[3] >> 4);

    temperature = ROUND_TO_DECIMAL((raw_temperature * 200) / 1048576.0 - 50.0);
    humidity = ROUND_TO_INTEGER((raw_humidity * 100) / 1048576.0);
    ESP_LOGW(TAG, "Temperature/Humidity=%f/%f", temperature, humidity);

    if (temperature > 100.0 || humidity > 100.0) {
        ESP_LOGD(TAG, "Incorrect sensor values");
        return false;
    }

    *ptemp = temperature;
    *phumi = humidity;

    return true;
}

static bool read_bmp280_reg(uint8_t reg, void *val, size_t size)
{
    uint8_t val2[3];

    if (i2c_master_transmit_receive(s_bmp280_handle, &reg, 1, val2, size, 100) != ESP_OK) {
        ESP_LOGE(TAG, "Reading from reg %02x failed", reg);
        return false;
    }

    /* 16-bit values in LE */
    switch (size) {
    case 1: *(uint8_t *)val = val2[0]; break;
    case 2: *(uint16_t *)val = ((uint16_t)val2[1] << 8) | val2[0]; break;
    default: *(uint32_t *)val = ((uint32_t)val2[0] << 12) |
        ((uint32_t)val2[1] << 4) | (val2[2] >> 4);
    }

    return true;
}

static bool write_bmp280_reg(uint8_t reg, uint8_t val)
{
    uint8_t val2[2] = {reg, val};

    if (i2c_master_transmit(s_bmp280_handle, val2, 2, 100) != ESP_OK) {
        ESP_LOGE(TAG, "Writing to reg %02x failed", reg);
        return false;
    }

    return true;
}

static bool init_bmp280(void)
{
    /* Check chip ID */
    if (bmp_id && (bmp_id > 0x58 || bmp_id < 0x56))
        return false;

    if (!s_bmp280_handle) {
        i2c_device_config_t bmp280_cfg = {
            .scl_speed_hz = CONFIG_I2C_FREQUENCY,
            .device_address = CONFIG_I2C_BMP280_ADDRESS,
        };

        if (!s_bus_handle) {
            if (!init_i2c_bus())
                return false;
        }

        if (i2c_master_bus_add_device(s_bus_handle, &bmp280_cfg, &s_bmp280_handle) != ESP_OK) {
            ESP_LOGE(TAG, "Add device failed");
            return false;
        }
    }

    /* Check chip ID */
    if (!bmp_id) {
        if (!read_bmp280_reg(BMP280_REGISTER_CHIPID, &bmp_id, 1))
            return false;

        /* 0x61=BME680, 0x60=BME280, 0x56,0x57,0x58=BMP280 */
        ESP_LOGD(TAG, "Sensor ID %02x", bmp_id);
        if (bmp_id > 0x58 || bmp_id < 0x56) {
            ESP_LOGE(TAG, "Unsupported sensor %x", bmp_id);
            fini_bmp280();
            return false;
        }
    }

    /* Read the temperature factory coefficients */
    if (!dig_t2 && !dig_t3) {
        if (!read_bmp280_reg(BMP280_REGISTER_DIG_T1, &dig_t1, 2) ||
        !read_bmp280_reg(BMP280_REGISTER_DIG_T2, &dig_t2, 2) ||
        !read_bmp280_reg(BMP280_REGISTER_DIG_T3, &dig_t3, 2))
            return false;

        ESP_LOGD(TAG, "Calibration data: %04x %04x %04x",
            dig_t1, dig_t2, dig_t3);
    }

    /* Read the pressure factory coefficients */
    if (!dig_p1) {
        for (uint8_t i = 0; i < 15; i += 2) {
            if (!read_bmp280_reg(BMP280_REGISTER_DIG_P2 + i, &dig_p[i], 2))
                return false;
        }

        if (!read_bmp280_reg(BMP280_REGISTER_DIG_P1, &dig_p1, 2))
            return false;
    }

    /* Standby duration = 1 ms */
    /* Filtering level for sensor data = Off */
    /* Enables 3-wire SPI = Off */
    /* Oversampling rate for the sensor = 16x/16x */
    /* Operating mode for the sensor = Normal */
    if (!write_bmp280_reg(BMP280_REGISTER_CONFIG, BMP280_CONFIG(0, 0, 0)) ||
    !write_bmp280_reg(BMP280_REGISTER_CONTROL, BMP280_CONFIG(5, 5, 3)))
        return false;

    /* Wait for configuration complete ~100ms */
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGD(TAG, "Configuration BMP280 complete");

    return true;
}

static bool read_bmp280(float *ptemp)
{
    int32_t raw_temperature;
    float temperature;

    if (!s_bmp280_handle) {
        if (!init_bmp280())
            return false;
    }

    if (!read_bmp280_reg(BMP280_REGISTER_TEMP, &raw_temperature, 3))
        return false;

    ESP_LOGD(TAG, "Raw BMP280 data: %08lx", raw_temperature);

    /* Don't try to understand it! */
    temp_fine = (((((raw_temperature >> 3) - ((int32_t)dig_t1 << 1))) *
        ((int32_t)dig_t2)) >> 11) +
        ((((((raw_temperature >> 4) - ((int32_t)dig_t1)) *
        ((raw_temperature >> 4) - ((int32_t)dig_t1))) >> 12) *
        ((int32_t)dig_t3)) >> 14);

    temperature = ROUND_TO_DECIMAL(((temp_fine * 5 + 128) >> 8) * 0.01);
    ESP_LOGW(TAG, "Temperature=%f", temperature);

    if (temperature > 100.0 || temperature < -50.0) {
        ESP_LOGD(TAG, "Incorrect sensor values");
        return false;
    }

    *ptemp = temperature;

    return true;
}

static bool read_bmp280_pressure(void)
{
    int32_t raw_pressure;
    int64_t v1, v2, p;
    float pressure;

    if (!s_bmp280_handle) {
        if (!init_bmp280())
            return false;
    }

    if (!read_bmp280_reg(BMP280_REGISTER_PRESSURE, &raw_pressure, 3))
        return false;

    ESP_LOGD(TAG, "Raw BMP280 pressure: %08lx", raw_pressure);

    /* Don't try to understand it! */
    v1 = ((int64_t)temp_fine) - 128000;
    v2 = v1 * v1 * (int64_t)dig_p[4];
    v2 = v2 + ((v1 * (int64_t)dig_p[3]) << 17);
    v2 = v2 + (((int64_t)dig_p[2]) << 35);
    v1 = ((v1 * v1 * (int64_t)dig_p[1]) >> 8) +
        ((v1 * (int64_t)dig_p[0]) << 12);
    v1 = (((((int64_t)1) << 47) + v1)) *
        ((int64_t)dig_p1) >> 33;

    /* Because division by zero later */
    if (!v1) {
        ESP_LOGD(TAG, "Incorrect sensor values");
        return false;
    }

    p = 1048576 - raw_pressure;
    p = (((p << 31) - v2) * 3125) / v1;
    v1 = (((int64_t)dig_p[7]) * (p >> 13) * (p >> 13)) >> 25;
    v2 = (((int64_t)dig_p[6]) * p) >> 19;
    p = ((p + v1 + v2) >> 8) + (((int64_t)dig_p[5]) << 4);

    pressure = ROUND_TO_DECIMAL((p >> 8) * 0.01);
    ESP_LOGW(TAG, "Pressure=%f(hPa) %.1f(cmHg)",
        pressure, pressure * 0.750062);

    return true;
}

static bool get_sensor_values(bool repeat)
{
    float temperature, temperature2, humidity;
    bool ok = read_ahtx0(&temperature, &humidity);
    bool ok2 = read_bmp280(&temperature2);

    if (ok2)
        read_bmp280_pressure();

    fini_i2c_bus();
    if (!ok && repeat) {
        ok = read_ahtx0(&temperature, &humidity);
        ok2 = read_bmp280(&temperature2);
        if (ok2)
            read_bmp280_pressure();

        fini_i2c_bus();
        if (!ok)
            return false;
    }

    if (ok2) {
        temperature = ROUND_TO_DECIMAL((temperature + temperature2) * 0.5);
        ESP_LOGW(TAG, "Average temperature=%f", temperature);
    }

    change_temperature(temperature);
    change_humidity(humidity);

    led_blink();

    return true;
}

static void on_timer(void* arg)
{
    ESP_LOGD(TAG, "Time to refresh sensor values");

    get_sensor_values(true);
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

    while (!get_sensor_values(false)) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    strcpy(g_name, "AHTx0");
    strcpy(g_model, "AHTx0");

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, SENSOR_GET_TIMEOUT));

    return ESP_OK;
}

void app_sensor_reset(bool full)
{
    (void)full;
}
