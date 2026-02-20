#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include "alc_i2c_bus_manager.h"
#include <mutex>
#include <functional>

namespace ALC {

/**
 * @brief BME280 sensor driver class for ESP-IDF.
 */
class BME280 {
public:
  using Callback = std::function<void(esp_err_t)>;

  /**
   * @brief Oversampling settings for Temperature, Pressure, and Humidity.
   *
   * Oversampling reduces noise by averaging multiple raw measurements *per sample*.
   * It remains relevant in BOTH Normal and Forced modes.
   *
   * - X1: 1 raw sample. Minimal power, lowest conversion time (~10ms).
   * - X2, X4, X8: Intermediate noise reduction and power.
   * - X16: 16 raw samples averaged. Best resolution, highest power per snapshot,
   *        longest conversion time (~100ms).
   *
   * Choosing a value:
   * - X1: Best for battery-critical apps where jitter is acceptable.
   * - X16: Best for high-precision monitoring (e.g., weather/storm detection).
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
   * The filter smooths out fluctuations (e.g., wind, cabin pressure changes).
   *
   * ! IMPORTANT FOR FORCED MODE: If sampling infrequently (e.g., every 10 min),
   * keep this OFF. The filter anchors the NEW reading to the OLD one (from 10 min ago),
   * causing a massive "lag" in detecting actual atmospheric changes. Use Oversampling
   * instead for noise reduction in low-frequency Forced mode.
   *
   * Choosing a value:
   * - OFF: 100% weight to newest sample. Fastest response.
   * - COEFF_2: 50% new / 50% old. Moderate smoothing.
   * - COEFF_16: ~6% new / ~94% old. Extreme smoothing, ignores gusts/brief noise.
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
   * - FORCED: (Recommended for low-frequency sampling) Sensor takes one measurement,
   *           then returns to Sleep. Ideal for periodic sampling (e.g., every 10 min).
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
   * @param i2c_bus Reference to the I2C bus manager.
   * @param address I2C address of the sensor (default: 0x76).
   */
  explicit BME280(I2CBusManager& i2c_bus, uint8_t address = 0x76);

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
   * @brief Initialize the sensor asynchronously.
   * @param cb Callback called when initialization is complete.
   */
  void InitAsync(Callback cb);

  /**
   * @brief Update the sensor configuration.
   * @param config The new configuration settings.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t Configure(const Configuration& config);

  /**
   * @brief Update the sensor configuration asynchronously.
   * @param config The new configuration settings.
   * @param cb Callback called when configuration is complete.
   */
  void ConfigureAsync(const Configuration& config, Callback cb);

  /**
   * @brief Read all sensor values (temperature, pressure, humidity).
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t ReadAll();

  /**
   * @brief Read all sensor values asynchronously.
   * @param cb Callback called when reading is complete.
   */
  void ReadAllAsync(Callback cb);

  // Getters
  float GetTemperature() const;
  float GetPressure() const;
  float GetHumidity() const;

private:
  esp_err_t ReadCalibrationDataInternal(i2c_port_t port);
  esp_err_t WriteRegisterInternal(i2c_port_t port, uint8_t reg, uint8_t value);
  esp_err_t ReadRegistersInternal(i2c_port_t port, uint8_t reg, uint8_t* data, size_t len);
  esp_err_t ApplyConfigurationInternal(i2c_port_t port);

  void PollMeasurementAsync(int retries, Callback cb);
  void ReadDataAsync(Callback cb);
  void ProcessRawData(const uint8_t* data);

  I2CBusManager& i2c_bus_;
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

  mutable std::recursive_mutex mutex_;
};

} // namespace ALC
