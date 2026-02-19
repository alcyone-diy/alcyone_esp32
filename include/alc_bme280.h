#pragma once

#include "driver/i2c.h"
#include "esp_err.h"

namespace ALC {

/**
 * @brief BME280 sensor driver class for ESP-IDF.
 *
 * This class provides methods to read temperature, pressure, and humidity
 * from a Bosch BME280 sensor connected via I2C.
 */
class BME280 {
public:
    /**
     * @brief Construct a new BME280 object.
     *
     * @param i2c_port I2C port number (e.g., I2C_NUM_0).
     * @param address I2C address of the sensor (typically 0x76 or 0x77).
     */
    BME280(i2c_port_t i2c_port, uint8_t address);

    /**
     * @brief Initialize the sensor.
     *
     * This method verifies the sensor ID, reads calibration data,
     * and configures the sensor to 'Normal Mode' for continuous measurement.
     *
     * @return esp_err_t ESP_OK on success, or an error code.
     */
    esp_err_t Init();

    /**
     * @brief Read all sensor values (temperature, pressure, humidity).
     *
     * This method fetches the latest raw data from the sensor and applies
     * compensation formulas. The results are stored internally and can be
     * accessed via getters.
     *
     * @return esp_err_t ESP_OK on success, or an error code.
     */
    esp_err_t ReadAll();

    /**
     * @brief Get the last read temperature.
     * @return float Temperature in degrees Celsius.
     */
    float GetTemperature() const { return temperature_; }

    /**
     * @brief Get the last read pressure.
     * @return float Pressure in hPa (hectopascals).
     */
    float GetPressure() const { return pressure_; }

    /**
     * @brief Get the last read humidity.
     * @return float Relative humidity in percentage (%).
     */
    float GetHumidity() const { return humidity_; }

private:
    esp_err_t ReadCalibrationData();
    esp_err_t WriteRegister(uint8_t reg, uint8_t value);
    esp_err_t ReadRegisters(uint8_t reg, uint8_t* data, size_t len);

    i2c_port_t i2c_port_;
    uint8_t address_;

    // Calibration data structure
    struct {
        uint16_t dig_T1;
        int16_t  dig_T2;
        int16_t  dig_T3;
        uint16_t dig_P1;
        int16_t  dig_P2;
        int16_t  dig_P3;
        int16_t  dig_P4;
        int16_t  dig_P5;
        int16_t  dig_P6;
        int16_t  dig_P7;
        int16_t  dig_P8;
        int16_t  dig_P9;
        uint8_t  dig_H1;
        int16_t  dig_H2;
        uint8_t  dig_H3;
        int16_t  dig_H4;
        int16_t  dig_H5;
        int8_t   dig_H6;
    } calib_;

    int32_t t_fine_{0};
    float temperature_{0.0f};
    float pressure_{0.0f};
    float humidity_{0.0f};
};

} // namespace ALC
