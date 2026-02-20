#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include <cstdint>
#include <mutex>

namespace ALC {

/**
 * @brief BNO086 IMU driver for ESP-IDF.
 *
 * This class implements the SH-2 protocol over I2C to communicate with the BNO086 sensor.
 */
class BNO086 {
public:
  struct Vector3 {
    float x, y, z;
    uint8_t accuracy;
  };

  struct Quaternion {
    float i, j, k, real;
    float accuracy; // radians
  };

  struct StepCounter {
    uint16_t count;
    uint32_t latency;
  };

  enum class Stability {
    UNKNOWN = 0,
    ON_TABLE = 1,
    STATIONARY = 2,
    STABLE = 3,
    IN_MOTION = 4
  };

  /**
   * @brief Construct a new BNO086 object.
   *
   * @param i2c_port I2C port number.
   * @param address I2C address (default is 0x4A, can be 0x4B).
   * @param i2c_timeout_ms I2C transaction timeout in milliseconds (default 100).
   */
  BNO086(i2c_port_t i2c_port, uint8_t address = 0x4A, uint32_t i2c_timeout_ms = 100);
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
   * @brief Close the sensor connection.
   * @return esp_err_t ESP_OK.
   */
  esp_err_t Close();

  /**
   * @brief Read pending packets from the sensor. Non-blocking.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t Update();

  // Configuration methods (period in microseconds)
  esp_err_t EnableAccelerometer(uint32_t period_us);
  esp_err_t EnableGyroscope(uint32_t period_us);
  esp_err_t EnableMagnetometer(uint32_t period_us);
  esp_err_t EnableLinearAcceleration(uint32_t period_us);
  esp_err_t EnableGravity(uint32_t period_us);
  esp_err_t EnableRotationVector(uint32_t period_us);
  esp_err_t EnableGameRotationVector(uint32_t period_us);
  esp_err_t EnableARVRStabilizedRotationVector(uint32_t period_us);
  esp_err_t EnableARVRStabilizedGameRotationVector(uint32_t period_us);
  esp_err_t EnableGyroIntegratedRotationVector(uint32_t period_us);
  esp_err_t EnableStepCounter(uint32_t period_us);
  esp_err_t EnableStabilityClassifier(uint32_t period_us);

  /**
   * @brief Configure dynamic calibration.
   * @param accel Enable accelerometer calibration.
   * @param gyro Enable gyroscope calibration.
   * @param mag Enable magnetometer calibration.
   * @return esp_err_t ESP_OK on success.
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
  i2c_port_t i2c_port_;
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
  esp_err_t SendPacket(uint8_t channel, uint16_t len);
  esp_err_t ReceivePacket(uint16_t timeout_ms = 0);
  void ParsePacket();
  void ParseSH2Report(uint8_t* payload, uint16_t len);
  void ParseGyroIntegratedReport(uint8_t* payload, uint16_t len);
  esp_err_t SetFeature(uint8_t report_id, uint32_t period_us);

  // SH-2 Report IDs
  static constexpr uint8_t SHTP_REPORT_COMMAND_RESPONSE = 0xF1;
  static constexpr uint8_t SHTP_REPORT_COMMAND_REQUEST = 0xF2;
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
