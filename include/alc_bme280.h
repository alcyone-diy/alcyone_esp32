#pragma once

#include "driver/i2c.h"
#include "esp_err.h"

namespace ALC {

/**
 * @brief BME280 sensor driver class for ESP-IDF.
 */
class BME280 {
public:
  /**
   * @brief Oversampling settings for Temperature, Pressure, and Humidity.
   *
   * Oversampling reduces noise by averaging multiple raw measurements.
   * - Pros: Higher resolution, less jitter/noise.
   * - Cons: Increased power consumption and longer measurement time.
   *
   * Choosing a value:
   * - X1: Good for high-speed tracking or very low power.
   * - X16: Best for high-precision pressure (e.g., weather/storm detection).
   */
  enum class Oversampling : uint8_t {
    SKIPPED = 0,
    X1 = 1,
    X2 = 2,
    X4 = 3,
    X8 = 4,
    X16 = 5
  };

  /**
   * @brief IIR Filter coefficients.
   *
   * The filter smooths out short-term fluctuations (e.g., wind, slamming doors).
   * - Pros: Much more stable readings, filters out "glitches".
   * - Cons: Introduces measurement latency (lag). Values take longer to stabilize.
   *
   * Choosing a value:
   * - OFF: Fastest response, most noise.
   * - COEFF_16: Most stable, but can take several samples to reflect a real change.
   */
  enum class Filter : uint8_t {
    OFF = 0,
    COEFF_2 = 1,
    COEFF_4 = 2,
    COEFF_8 = 3,
    COEFF_16 = 4
  };

  /**
   * @brief Sensor operation modes.
   *
   * - SLEEP: No measurements. Lowest possible power.
   * - FORCED: (Recommended for Boat case) Sensor takes one measurement, then returns
   *           to Sleep. Ideal for low-frequency periodic sampling (e.g., every 10 min).
   * - NORMAL: Sensor cycles automatically between measurement and standby.
   *           Ideal for high-frequency data streams.
   */
  enum class SensorMode : uint8_t {
    SLEEP = 0,
    FORCED = 1,
    NORMAL = 3
  };

  /**
   * @brief Standby duration between measurements in NORMAL mode.
   *
   * Only applicable when using SensorMode::NORMAL.
   * - Pros: High values (e.g., 1000ms) save power.
   * - Cons: Low values provide faster update rates.
   */
  enum class StandbyTime : uint8_t {
    MS_0_5 = 0,
    MS_62_5 = 1,
    MS_125 = 2,
    MS_250 = 3,
    MS_500 = 4,
    MS_1000 = 5,
    MS_10 = 6,
    MS_20 = 7
  };

  struct Configuration {
    Oversampling temp_os = Oversampling::X1;
    Oversampling press_os = Oversampling::X1;
    Oversampling hum_os = Oversampling::X1;
    Filter filter = Filter::OFF;
    SensorMode mode = SensorMode::NORMAL;
    StandbyTime standby = StandbyTime::MS_1000;
  };

  /**
   * @brief Default constructor is deleted.
   */
  BME280() = delete;

  /**
   * @brief Construct a new BME280 object.
   *
   * @param i2c_port I2C port number.
   * @param address I2C address of the sensor (default: 0x76).
   */
  explicit BME280(i2c_port_t i2c_port, uint8_t address = 0x76);

  /**
   * @brief Copying or moving a sensor instance is not allowed.
   */
  BME280(const BME280&) = delete;
  BME280& operator=(const BME280&) = delete;
  BME280(BME280&&) = delete;
  BME280& operator=(BME280&&) = delete;

  /**
   * @brief Initialize the sensor.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t Init();

  /**
   * @brief Update the sensor configuration.
   * @param config The new configuration settings.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t Configure(const Configuration& config);

  /**
   * @brief Read all sensor values (temperature, pressure, humidity).
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t ReadAll();

  // Getters
  float GetTemperature() const { return temperature_; }
  float GetPressure() const { return pressure_; }
  float GetHumidity() const { return humidity_; }

private:
  esp_err_t ReadCalibrationData();
  esp_err_t WriteRegister(uint8_t reg, uint8_t value);
  esp_err_t ReadRegisters(uint8_t reg, uint8_t* data, size_t len);
  esp_err_t ApplyConfiguration();

  i2c_port_t i2c_port_;
  uint8_t address_;
  Configuration config_;

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
