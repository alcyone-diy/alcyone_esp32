#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "alc_i2c_bus_manager.h"
#include <cstdint>
#include <mutex>

namespace ALC {

/**
 * @brief BNO086Sensor IMU driver for ESP-IDF.
 *
 * This class implements the SH-2 protocol over I2C to communicate with the BNO086 sensor.
 * It utilizes the ALC::I2CBusManager for non-blocking, serialized I2C access.
 *
 * ### Usage
 * 1. Instantiate the @ref BNO086Sensor class with a reference to the I2CBusManager.
 * 2. Call @ref Init() to initialize the sensor and perform a soft reset.
 * 3. Call one or more `Enable...()` methods to start receiving data from specific sensors.
 * 4. Call @ref Update() periodically to poll for new packets.
 * 5. Retrieve the latest data using the corresponding `Get...()` methods.
 */
class BNO086Sensor {
public:
  using Callback = I2CBusManager::Callback;
  using BusToken = I2CBusManager::BusToken;

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
   * @brief Construct a new BNO086Sensor object.
   *
   * @param bus_manager Reference to the I2C bus manager.
   * @param address I2C address (default is 0x4A, can be 0x4B).
   */
  explicit BNO086Sensor(I2CBusManager& bus_manager, uint16_t address = 0x4A);

  /**
   * @brief Default constructor is deleted.
   */
  BNO086Sensor() = delete;

  /**
   * @brief Copying or moving a sensor instance is not allowed.
   */
  BNO086Sensor(const BNO086Sensor&) = delete;
  BNO086Sensor& operator=(const BNO086Sensor&) = delete;
  BNO086Sensor(BNO086Sensor&&) = delete;
  BNO086Sensor& operator=(BNO086Sensor&&) = delete;

  /**
   * @brief Initialize the sensor. Performs a soft reset and waits for advertisement.
   * @param cb Optional callback called when initialization is complete.
   */
  void Init(Callback cb = nullptr);

  /**
   * @brief Read pending packets from the sensor. Non-blocking.
   * @param cb Optional callback called when update is complete.
   */
  void Update(Callback cb = nullptr);

  /**
   * @brief Enable Accelerometer reports.
   * @param period_us Reporting period in microseconds.
   * @param cb Optional callback.
   */
  void EnableAccelerometer(uint32_t period_us, Callback cb = nullptr);

  /**
   * @brief Enable Gyroscope reports.
   * @param period_us Reporting period in microseconds.
   * @param cb Optional callback.
   */
  void EnableGyroscope(uint32_t period_us, Callback cb = nullptr);

  /**
   * @brief Enable Magnetometer reports.
   * @param period_us Reporting period in microseconds.
   * @param cb Optional callback.
   */
  void EnableMagnetometer(uint32_t period_us, Callback cb = nullptr);

  /**
   * @brief Enable Linear Acceleration reports (Acceleration minus Gravity).
   * @param period_us Reporting period in microseconds.
   * @param cb Optional callback.
   */
  void EnableLinearAcceleration(uint32_t period_us, Callback cb = nullptr);

  /**
   * @brief Enable Gravity reports.
   * @param period_us Reporting period in microseconds.
   * @param cb Optional callback.
   */
  void EnableGravity(uint32_t period_us, Callback cb = nullptr);

  /**
   * @brief Enable Rotation Vector reports.
   * @param period_us Reporting period in microseconds.
   * @param cb Optional callback.
   */
  void EnableRotationVector(uint32_t period_us, Callback cb = nullptr);

  /**
   * @brief Enable Game Rotation Vector reports.
   * @param period_us Reporting period in microseconds.
   * @param cb Optional callback.
   */
  void EnableGameRotationVector(uint32_t period_us, Callback cb = nullptr);

  /**
   * @brief Enable ARVR Stabilized Rotation Vector reports.
   * @param period_us Reporting period in microseconds.
   * @param cb Optional callback.
   */
  void EnableARVRStabilizedRotationVector(uint32_t period_us, Callback cb = nullptr);

  /**
   * @brief Enable ARVR Stabilized Game Rotation Vector reports.
   * @param period_us Reporting period in microseconds.
   * @param cb Optional callback.
   */
  void EnableARVRStabilizedGameRotationVector(uint32_t period_us, Callback cb = nullptr);

  /**
   * @brief Enable Gyro Integrated Rotation Vector reports (High-rate 400Hz+).
   * @param period_us Reporting period in microseconds.
   * @param cb Optional callback.
   */
  void EnableGyroIntegratedRotationVector(uint32_t period_us, Callback cb = nullptr);

  /**
   * @brief Enable Step Counter reports.
   * @param period_us Reporting period in microseconds.
   * @param cb Optional callback.
   */
  void EnableStepCounter(uint32_t period_us, Callback cb = nullptr);

  /**
   * @brief Enable Stability Classifier reports.
   * @param period_us Reporting period in microseconds.
   * @param cb Optional callback.
   */
  void EnableStabilityClassifier(uint32_t period_us, Callback cb = nullptr);

  /**
   * @brief Configure dynamic calibration for the Motion Engine (ME).
   */
  void SetCalibrationConfig(bool accel, bool gyro, bool mag, Callback cb = nullptr);

  /**
   * @brief Command the sensor to save its current calibration to flash.
   */
  void SaveCalibration(Callback cb = nullptr);

  /**
   * @brief Perform a soft reset of the sensor.
   */
  void SoftReset(Callback cb = nullptr);

  /**
   * @brief Set the sensor power mode.
   * @param sleep True to put the sensor to sleep, false to wake it up (On).
   */
  void SetPowerMode(bool sleep, Callback cb = nullptr);

  // Getters for the latest data (thread-safe)
  Vector3 GetAccelerometer() const;
  Vector3 GetGyroscope() const;
  Vector3 GetMagnetometer() const;
  Vector3 GetLinearAcceleration() const;
  Vector3 GetGravity() const;
  Quaternion GetRotationVector() const;
  Quaternion GetGameRotationVector() const;
  Quaternion GetARVRStabilizedRotationVector() const;
  Quaternion GetARVRStabilizedGameRotationVector() const;
  Quaternion GetGyroIntegratedRotationVector() const;
  Vector3 GetGyroIntegratedAngularVelocity() const;
  uint16_t GetStepCount() const;
  Stability GetStability() const;

private:
  I2CBusManager& bus_manager_;
  uint16_t address_;
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
  esp_err_t SendPacket(BusToken& token, uint8_t channel, uint16_t len);
  esp_err_t ReceivePacket(BusToken& token, uint16_t timeout_ms = 0);
  void ParsePacket();
  void ParseSH2Report(uint8_t* payload, uint16_t len);
  void ParseGyroIntegratedReport(uint8_t* payload, uint16_t len);
  void SetFeature(uint8_t report_id, uint32_t period_us, Callback cb);
  void PollForAdvertisement(int attempts_left, Callback cb);
};

} // namespace ALC
