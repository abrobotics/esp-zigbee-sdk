#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "my_aht20.h"

static const char *TAG = "AHT20";

// Function to initialize sensor EN pin
void aht20_en_init() {
    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << I2C_MASTER_EN_IO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&conf);
    gpio_set_level(I2C_MASTER_EN_IO, 1); // Enable the sensor via GPIO
}

// Function to initialize I2C
void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Function to write data to the AHT20
esp_err_t aht20_write_command(uint8_t cmd) {
    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    i2c_master_start(handle);
    i2c_master_write_byte(handle, (AHT20_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(handle, cmd, true);
    if (cmd == AHT20_CMD_INIT) {
        i2c_master_write_byte(handle, 0x08, true); // Data byte for init
        i2c_master_write_byte(handle, 0x00, true); // Data byte for init
    }
    else if (cmd == AHT20_CMD_TRIGGER) {
        i2c_master_write_byte(handle, 0x33, true); // Data byte 1 for trigger
        i2c_master_write_byte(handle, 0x00, true); // Data byte 2 for trigger
    }
    i2c_master_stop(handle);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, handle, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(handle);
    return ret;
}

// Function to read data from the AHT20
esp_err_t aht20_read_data(uint8_t *data, size_t length) {
    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    i2c_master_start(handle);
    i2c_master_write_byte(handle, (AHT20_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(handle, data, length, I2C_MASTER_LAST_NACK);
    i2c_master_stop(handle);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, handle, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(handle);
    return ret;
}

// Function to reset and initialize the AHT20
void aht20_init() {
    esp_err_t ret = aht20_write_command(AHT20_CMD_SOFTRESET);
    ESP_LOGI(TAG, "Sensor reset command sent: %s", esp_err_to_name(ret));
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait after reset
    ret = aht20_write_command(AHT20_CMD_INIT);
    ESP_LOGI(TAG, "Sensor init command sent: %s", esp_err_to_name(ret));
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for initialization
}

// Function to calculate CRC8
uint8_t calculate_crc8(const uint8_t *data, size_t length) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// Function to get temperature and humidity
void aht20_get_temp_humidity(float *temperature, float *humidity) {
    // Initialize output values
    *humidity = -1.0;
    *temperature = -1.0;

    uint8_t data[6];
    esp_err_t ret = aht20_write_command(AHT20_CMD_TRIGGER);
    ESP_LOGD(TAG, "Measurement trigger command sent: %s", esp_err_to_name(ret));
    vTaskDelay(pdMS_TO_TICKS(80)); // Wait for the measurement

    if (aht20_read_data(data, 7) == ESP_OK) {
        // Log raw data for debugging
        ESP_LOGD(TAG, "Raw data: %02X %02X %02X %02X %02X %02X %02X", 
            data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
        // CRC8 check
        uint8_t crc = data[6];
        uint8_t calc_crc = calculate_crc8(data, 6);
        if (crc != calc_crc) {
            ESP_LOGE(TAG, "CRC check failed: received %02X, calculated %02X", crc, calc_crc);
            return;
        }
        // Parse humidity and temperature
        uint32_t raw_humidity = ((data[1] << 16) | (data[2] << 8) | data[3]) >> 4;
        uint32_t raw_temperature = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];
        ESP_LOGD(TAG, "Raw Humidity: %lu, Raw Temperature: %lu", raw_humidity, raw_temperature);

        *humidity = ((float)raw_humidity / 1048576.0) * 100.0;
        *temperature = ((float)raw_temperature / 1048576.0) * 200.0 - 50.0;
    } else {
        ESP_LOGE(TAG, "Failed to read data from sensor");
    }
}

void setup_aht20() {
    aht20_en_init();
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for sensor to power up
    i2c_master_init();
    aht20_init();
}


#include "esp_err.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* call back function pointer */
static esp_temp_sensor_callback_t func_ptr;
/* update interval in seconds */
static uint16_t interval = 1;

/**
 * @brief Tasks for updating the sensor value
 *
 * @param arg      Unused value.
 */
static void aht20_driver_value_update(void *arg)
{
    for (;;) {
        float tsens_value = 0.0, hsens_value = 0.0;
        aht20_get_temp_humidity(&tsens_value, &hsens_value);
        ESP_LOGI(TAG, "Measured temperature: %f", tsens_value);
        if (func_ptr) {
            func_ptr(tsens_value);
        }
        vTaskDelay(pdMS_TO_TICKS(interval * 1000));
    }
}


esp_err_t aht20_driver_init(uint16_t update_interval,
                             esp_temp_sensor_callback_t cb)
{
    setup_aht20(); // Setup Hardware AHT20 sensor
    func_ptr = cb;
    interval = update_interval;
    return (xTaskCreate(aht20_driver_value_update, "sensor_update", 2048, NULL, 10, NULL) == pdTRUE) ? ESP_OK : ESP_FAIL;
}

  

