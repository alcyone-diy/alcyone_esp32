#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include "alc_i2c_bus_manager.h"
#include <cstdint>
#include <mutex>
#include <functional>

namespace ALC {

/**
 * @brief BNO086 IMU driver for ESP-IDF.
 *
 * This class implements the SH-2 protocol over I2C to communicate with the BNO086 sensor.
 *
 * ### Usage
 * 1. Instantiate the @ref BNO086 class with the I2C bus manager.
 * 2. Call @ref Open() or @ref OpenAsync() to initialize the sensor and perform a soft reset.
 * 3. Call one or more `Enable...()` methods to start receiving data from specific sensors (e.g., @ref EnableRotationVector()).
 * 4. In a loop, call @ref Update() or @ref UpdateAsync() to poll for new packets from the sensor.
 * 5. Retrieve the latest data using the corresponding `Get...()` methods (e.g., @ref GetRotationVector()).
 *
 * Example:
 * @code
 * ALC::I2CBusManager i2c_bus(I2C_NUM_0);
 * i2c_bus.Init(GPIO_NUM_21, GPIO_NUM_22);
 * ALC::BNO086 imu(i2c_bus);
 * if (imu.Open() == ESP_OK) {
 *     imu.EnableRotationVector(10000); // 10ms period
 *     while (true) {
 *         imu.Update();
 *         auto rv = imu.GetRotationVector();
 *         // Use rv.i, rv.j, rv.k, rv.real
 *         vTaskDelay(pdMS_TO_TICKS(10));
 *     }
 * }
 * @endcode
 */
class BNO086 {
public:
  using Callback = std::function<void(esp_err_t)>;

  /**
   * @brief A 3D vector for sensor data.
   */
  struct Vector3 {
    float x;           ///< X-axis value (m/s^2 for accel, rad/s for gyro, uT for mag)
    float y;           ///< Y-axis value
    float z;           ///< Z-axis value
    uint8_t accuracy;  ///< Accuracy estimate (0-3: 0=Unreliable, 1=Low, 2=Medium, 3=High)
  };

  /**
   * @brief A quaternion representing rotation in 3D space.
   */
  struct Quaternion {
    float i;           ///< Imaginary part i (x)
    float j;           ///< Imaginary part j (y)
    float k;           ///< Imaginary part k (z)
    float real;        ///< Real part (w)
    float accuracy;    ///< Estimated accuracy in radians
  };

  /**
   * @brief Step counter data.
   */
  struct StepCounter {
    uint16_t count;    ///< Number of steps detected
    uint32_t latency;  ///< Time since last step was detected in microseconds
  };

  /**
   * @brief Stability classifier output.
   */
  enum class Stability {
    UNKNOWN = 0,       ///< Stability unknown
    ON_TABLE = 1,      ///< Device is on a flat, stable surface
    STATIONARY = 2,    ///< Device is not moving (e.g. held still)
    STABLE = 3,        ///< Device is relatively stable
    IN_MOTION = 4      ///< Device is moving
  };

  /**
   * @brief Construct a new BNO086 object.
   *
   * @param i2c_bus Reference to the I2C bus manager.
   * @param address I2C address (default is 0x4A, can be 0x4B).
   * @param i2c_timeout_ms I2C transaction timeout in milliseconds (default 100).
   */
  BNO086(I2CBusManager& i2c_bus, uint8_t address = 0x4A, uint32_t i2c_timeout_ms = 100);
  ~BNO086();

  // Disable default constructor
  BNO086() = delete;
  // Disable copy constructor and assignment
  BNO086(const BNO086&) = delete;
  BNO086& operator=(const BNO086&) = delete;

  /**
   * @brief Initialize the sensor. Performs a soft reset and waits for advertisement.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t Open();

  /**
   * @brief Initialize and open the sensor asynchronously.
   * @param cb Callback called when open is complete.
   */
  void OpenAsync(Callback cb);

  /**
   * @brief Close the sensor connection.
   * @return esp_err_t ESP_OK.
   */
  esp_err_t Close();

  /**
   * @brief Read pending packets from the sensor. Non-blocking.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t Update();

  /**
   * @brief Poll for new packets from the sensor asynchronously.
   * @param cb Callback called when update is complete.
   */
  void UpdateAsync(Callback cb);

  /**
   * @brief Enable Accelerometer reports.
   * @param period_us Reporting period in microseconds.
   * @return esp_err_t ESP_OK on success.
   * @note Unit: m/s^2.
   */
  esp_err_t EnableAccelerometer(uint32_t period_us);

  /**
   * @brief Enable Gyroscope reports.
   * @param period_us Reporting period in microseconds.
   * @return esp_err_t ESP_OK on success.
   * @note Unit: rad/s.
   */
  esp_err_t EnableGyroscope(uint32_t period_us);

  /**
   * @brief Enable Magnetometer reports.
   * @param period_us Reporting period in microseconds.
   * @return esp_err_t ESP_OK on success.
   * @note Unit: uT.
   */
  esp_err_t EnableMagnetometer(uint32_t period_us);

  /**
   * @brief Enable Linear Acceleration reports (Acceleration minus Gravity).
   * @param period_us Reporting period in microseconds.
   * @return esp_err_t ESP_OK on success.
   * @note Unit: m/s^2.
   */
  esp_err_t EnableLinearAcceleration(uint32_t period_us);

  /**
   * @brief Enable Gravity reports.
   * @param period_us Reporting period in microseconds.
   * @return esp_err_t ESP_OK on success.
   * @note Unit: m/s^2.
   */
  esp_err_t EnableGravity(uint32_t period_us);

  /**
   * @brief Enable Rotation Vector reports (Fused data including Magnetometer).
   * @param period_us Reporting period in microseconds.
   */
  esp_err_t EnableRotationVector(uint32_t period_us);

  /**
   * @brief Enable Game Rotation Vector reports (Fused data, no Magnetometer).
   * @param period_us Reporting period in microseconds.
   */
  esp_err_t EnableGameRotationVector(uint32_t period_us);

  /**
   * @brief Enable ARVR Stabilized Rotation Vector reports.
   * @param period_us Reporting period in microseconds.
   */
  esp_err_t EnableARVRStabilizedRotationVector(uint32_t period_us);

  /**
   * @brief Enable ARVR Stabilized Game Rotation Vector reports.
   * @param period_us Reporting period in microseconds.
   */
  esp_err_t EnableARVRStabilizedGameRotationVector(uint32_t period_us);

  /**
   * @brief Enable Gyro Integrated Rotation Vector reports (High-rate 400Hz+).
   * @param period_us Reporting period in microseconds.
   */
  esp_err_t EnableGyroIntegratedRotationVector(uint32_t period_us);

  /**
   * @brief Enable Step Counter reports.
   * @param period_us Reporting period in microseconds.
   */
  esp_err_t EnableStepCounter(uint32_t period_us);

  /**
   * @brief Enable Stability Classifier reports.
   * @param period_us Reporting period in microseconds.
   */
  esp_err_t EnableStabilityClassifier(uint32_t period_us);

  /**
   * @brief Configure dynamic calibration for the Motion Engine (ME).
   *
   * Dynamic calibration allows the sensor to continuously update its internal calibration offsets
   * based on motion patterns.
   *
   * @param accel Enable accelerometer calibration (based on periods of stillness and gravity).
   * @param gyro Enable gyroscope calibration (zero-rate offset when detected as stationary).
   * @param mag Enable magnetometer calibration (based on rotation through the magnetic field).
   * @return esp_err_t ESP_OK on success.
   *
   * @note It is recommended to enable these during normal operation to improve accuracy.
   * Use SaveCalibration() to persist the current calibration to the sensor's flash memory.
   */
  esp_err_t SetCalibrationConfig(bool accel, bool gyro, bool mag);

  /**
   * @brief Command the sensor to save its current calibration to flash.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t SaveCalibration();

  /**
   * @brief Perform a soft reset of the sensor.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t SoftReset();

  /**
   * @brief Set the sensor power mode.
   *
   * Putting the sensor to sleep reduces power consumption. The sensor will still respond
   * to commands in sleep mode.
   *
   * @param sleep True to put the sensor to sleep, false to wake it up (On).
   * @return esp_err_t ESP_OK on success.
   *
   * @note For ESP32 Deep Sleep, it is recommended to put the BNO086 to sleep first if
   * its power supply is maintained during the host's sleep.
   */
  esp_err_t SetPowerMode(bool sleep);

  // Getters for the latest data
  Vector3 GetAccelerometer() const { return accel_; }
  Vector3 GetGyroscope() const { return gyro_; }
  Vector3 GetMagnetometer() const { return mag_; }
  Vector3 GetLinearAcceleration() const { return linear_accel_; }
  Vector3 GetGravity() const { return gravity_; }
  Quaternion GetRotationVector() const { return rotation_vector_; }
  Quaternion GetGameRotationVector() const { return game_rotation_vector_; }
  Quaternion GetARVRStabilizedRotationVector() const { return arvr_rotation_vector_; }
  Quaternion GetARVRStabilizedGameRotationVector() const { return arvr_game_rotation_vector_; }
  Quaternion GetGyroIntegratedRotationVector() const { return gyro_integrated_rv_; }
  Vector3 GetGyroIntegratedAngularVelocity() const { return gyro_integrated_av_; }
  uint16_t GetStepCount() const { return step_counter_.count; }
  Stability GetStability() const { return stability_; }

private:
  I2CBusManager& i2c_bus_;
  uint8_t address_;
  uint32_t i2c_timeout_ms_;
  uint8_t sequence_number_[6] = {0}; // SHTP sequence numbers for each channel
  uint8_t sh2_sequence_number_ = 0;   // SH-2 command sequence number

  // Data storage
  Vector3 accel_ = {0, 0, 0, 0};
  Vector3 gyro_ = {0, 0, 0, 0};
  Vector3 mag_ = {0, 0, 0, 0};
  Vector3 linear_accel_ = {0, 0, 0, 0};
  Vector3 gravity_ = {0, 0, 0, 0};
  Quaternion rotation_vector_ = {0, 0, 0, 0, 0};
  Quaternion game_rotation_vector_ = {0, 0, 0, 0, 0};
  Quaternion arvr_rotation_vector_ = {0, 0, 0, 0, 0};
  Quaternion arvr_game_rotation_vector_ = {0, 0, 0, 0, 0};
  Quaternion gyro_integrated_rv_ = {0, 0, 0, 0, 0};
  Vector3 gyro_integrated_av_ = {0, 0, 0, 0};
  StepCounter step_counter_ = {0, 0};
  Stability stability_ = Stability::UNKNOWN;

  // SHTP buffer and recursive mutex for thread safety
  uint8_t buffer_[256];
  mutable std::recursive_mutex mutex_;

  // SH-2 Protocol helpers
  esp_err_t SendPacketInternal(i2c_port_t port, uint8_t channel, uint16_t len);
  esp_err_t ReceivePacketInternal(i2c_port_t port, uint16_t timeout_ms = 0);
  void ParsePacket();
  void ParseSH2Report(uint8_t* payload, uint16_t len);
  void ParseGyroIntegratedReport(uint8_t* payload, uint16_t len);
  esp_err_t SetFeature(uint8_t report_id, uint32_t period_us);
  void PollAdvertisementAsync(int retries, Callback cb);

  // SH-2 Report IDs
  static constexpr uint8_t SHTP_REPORT_COMMAND_RESPONSE = 0xF1;
  static constexpr uint8_t SHTP_REPORT_COMMAND_REQUEST = 0xF2;

  // SH-2 Commands
  static constexpr uint8_t SH2_COMMAND_SET_POWER_STATE = 0x01;
  static constexpr uint8_t SHTP_REPORT_SET_FEATURE_COMMAND = 0xFD;
  static constexpr uint8_t SHTP_REPORT_GET_FEATURE_RESPONSE = 0xFC;

  static constexpr uint8_t SENSOR_REPORTID_ACCELEROMETER = 0x01;
  static constexpr uint8_t SENSOR_REPORTID_GYROSCOPE = 0x02;
  static constexpr uint8_t SENSOR_REPORTID_MAGNETIC_FIELD = 0x03;
  static constexpr uint8_t SENSOR_REPORTID_LINEAR_ACCELERATION = 0x04;
  static constexpr uint8_t SENSOR_REPORTID_ROTATION_VECTOR = 0x05;
  static constexpr uint8_t SENSOR_REPORTID_GRAVITY = 0x06;
  static constexpr uint8_t SENSOR_REPORTID_UNCALIBRATED_GYROSCOPE = 0x07;
  static constexpr uint8_t SENSOR_REPORTID_GAME_ROTATION_VECTOR = 0x08;
  static constexpr uint8_t SENSOR_REPORTID_UNCALIBRATED_MAGNETIC_FIELD = 0x09;
  static constexpr uint8_t SENSOR_REPORTID_ARVR_STABILIZED_ROTATION_VECTOR = 0x0A;
  static constexpr uint8_t SENSOR_REPORTID_ARVR_STABILIZED_GAME_ROTATION_VECTOR = 0x0B;
  static constexpr uint8_t SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR = 0x1E;
  static constexpr uint8_t SENSOR_REPORTID_STEP_COUNTER = 0x14;
  static constexpr uint8_t SENSOR_REPORTID_STABILITY_CLASSIFIER = 0x15;

  // SH-2 Channels
  static constexpr uint8_t CHANNEL_COMMAND = 0;
  static constexpr uint8_t CHANNEL_EXECUTABLE = 1;
  static constexpr uint8_t CHANNEL_CONTROL = 2;
  static constexpr uint8_t CHANNEL_REPORTS = 3;
  static constexpr uint8_t CHANNEL_WAKE_REPORTS = 4;
  static constexpr uint8_t CHANNEL_GYRO_INTEGRATED = 5;
};

} // namespace ALC
