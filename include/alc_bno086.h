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
 */
class BNO086 {
public:
  using Callback = std::function<void(esp_err_t)>;

  struct Vector3 {
    float x;           ///< X-axis value (m/s^2 for accel, rad/s for gyro, uT for mag)
    float y;           ///< Y-axis value
    float z;           ///< Z-axis value
    uint8_t accuracy;  ///< Accuracy estimate (0-3: 0=Unreliable, 1=Low, 2=Medium, 3=High)
  };

  struct Quaternion {
    float i;           ///< Imaginary part i (x)
    float j;           ///< Imaginary part j (y)
    float k;           ///< Imaginary part k (z)
    float real;        ///< Real part (w)
    float accuracy;    ///< Estimated accuracy in radians
  };

  struct StepCounter {
    uint16_t count;    ///< Number of steps detected
    uint32_t latency;  ///< Time since last step was detected in microseconds
  };

  enum class Stability {
    UNKNOWN = 0,       ///< Stability unknown
    ON_TABLE = 1,      ///< Device is on a flat, stable surface
    STATIONARY = 2,    ///< Device is not moving (e.g. held still)
    STABLE = 3,        ///< Device is relatively stable
    IN_MOTION = 4      ///< Device is moving
  };

  /**
   * @brief Construct a new BNO086 object.
   * @param i2c_bus Reference to the I2C bus manager.
   * @param address I2C address (default is 0x4A).
   * @param i2c_timeout_ms I2C transaction timeout.
   */
  BNO086(I2CBusManager& i2c_bus, uint8_t address = 0x4A, uint32_t i2c_timeout_ms = 100);
  ~BNO086();

  BNO086() = delete;
  BNO086(const BNO086&) = delete;
  BNO086& operator=(const BNO086&) = delete;

  esp_err_t Open();
  void OpenAsync(Callback cb);

  esp_err_t Close();

  esp_err_t Update();
  void UpdateAsync(Callback cb);

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

  esp_err_t SetCalibrationConfig(bool accel, bool gyro, bool mag);
  esp_err_t SaveCalibration();
  esp_err_t SoftReset();
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
  uint8_t sequence_number_[6] = {0};
  uint8_t sh2_sequence_number_ = 0;

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

  uint8_t buffer_[256];
  mutable std::recursive_mutex mutex_;

  esp_err_t SendPacketInternal(i2c_port_t port, uint8_t channel, uint16_t len);
  esp_err_t ReceivePacketInternal(i2c_port_t port, uint16_t timeout_ms = 0);
  void ParsePacket();
  void ParseSH2Report(uint8_t* payload, uint16_t len);
  void ParseGyroIntegratedReport(uint8_t* payload, uint16_t len);
  esp_err_t SetFeature(uint8_t report_id, uint32_t period_us);
  void PollAdvertisementAsync(int retries, Callback cb);

  static constexpr uint8_t SHTP_REPORT_COMMAND_RESPONSE = 0xF1;
  static constexpr uint8_t SHTP_REPORT_COMMAND_REQUEST = 0xF2;
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

  static constexpr uint8_t CHANNEL_COMMAND = 0;
  static constexpr uint8_t CHANNEL_EXECUTABLE = 1;
  static constexpr uint8_t CHANNEL_CONTROL = 2;
  static constexpr uint8_t CHANNEL_REPORTS = 3;
  static constexpr uint8_t CHANNEL_WAKE_REPORTS = 4;
  static constexpr uint8_t CHANNEL_GYRO_INTEGRATED = 5;
};

} // namespace ALC
