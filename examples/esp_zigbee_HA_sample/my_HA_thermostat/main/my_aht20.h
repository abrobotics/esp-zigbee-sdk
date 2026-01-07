#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_MASTER_NUM I2C_NUM_0         // I2C port number
#define I2C_MASTER_EN_IO 5            // GPIO for SDA
#define I2C_MASTER_SDA_IO 6            // GPIO for SDA
#define I2C_MASTER_SCL_IO 7            // GPIO for SCL
#define I2C_MASTER_FREQ_HZ 100000       // I2C clock frequency
#define AHT20_ADDR 0x38                 // I2C address of the AHT20 sensor
#define AHT20_CMD_TRIGGER 0xAC          // Command to trigger measurement
#define AHT20_CMD_SOFTRESET 0xBA        // Command to reset the sensor
#define AHT20_CMD_INIT 0xBE             // Command to initialize the sensor

void setup_aht20();
void aht20_get_temp_humidity(float *temperature, float *humidity);


/** Temperature sensor callback
 *
 * @param[in] temperature temperature value in degrees Celsius from sensor
 *
 */
typedef void (*esp_temp_sensor_callback_t)(float temperature);

/**
 * @brief init function for temp sensor and callback setup
 *
 * @param update_interval       sensor value update interval in seconds.
 * @param cb                    callback pointer.
 *
 * @return ESP_OK if the driver initialization succeed, otherwise ESP_FAIL.
 */
esp_err_t aht20_driver_init(uint16_t update_interval, esp_temp_sensor_callback_t cb);


#ifdef __cplusplus
}
#endif
