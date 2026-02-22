#include "i2c/alc_bno086_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <cmath>

static const char* TAG = "ALC_BNO086Driver";

// SH-2 Report IDs
#define SHTP_REPORT_COMMAND_RESPONSE                  0xF1
#define SHTP_REPORT_COMMAND_REQUEST                   0xF2
#define SHTP_REPORT_SET_FEATURE_COMMAND               0xFD
#define SHTP_REPORT_GET_FEATURE_RESPONSE              0xFC

// SH-2 Commands
#define SH2_COMMAND_SET_POWER_STATE                   0x01
#define SH2_COMMAND_ME_CALIBRATION                    0x07
#define SH2_COMMAND_SAVE_DCD                          0x06

// Driver Report IDs
#define DRIVER_REPORTID_ACCELEROMETER                 0x01
#define DRIVER_REPORTID_GYROSCOPE                     0x02
#define DRIVER_REPORTID_MAGNETIC_FIELD                0x03
#define DRIVER_REPORTID_LINEAR_ACCELERATION           0x04
#define DRIVER_REPORTID_ROTATION_VECTOR               0x05
#define DRIVER_REPORTID_GRAVITY                       0x06
#define DRIVER_REPORTID_UNCALIBRATED_GYROSCOPE        0x07
#define DRIVER_REPORTID_GAME_ROTATION_VECTOR          0x08
#define DRIVER_REPORTID_UNCALIBRATED_MAGNETIC_FIELD   0x09
#define DRIVER_REPORTID_ARVR_STABILIZED_ROTATION_VECTOR      0x0A
#define DRIVER_REPORTID_ARVR_STABILIZED_GAME_ROTATION_VECTOR 0x0B
#define DRIVER_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR      0x1E
#define DRIVER_REPORTID_STEP_COUNTER                  0x14
#define DRIVER_REPORTID_STABILITY_CLASSIFIER          0x15

// SH-2 Channels
#define CHANNEL_COMMAND                               0
#define CHANNEL_EXECUTABLE                            1
#define CHANNEL_CONTROL                               2
#define CHANNEL_REPORTS                               3
#define CHANNEL_WAKE_REPORTS                          4
#define CHANNEL_GYRO_INTEGRATED                       5

namespace ALC {

BNO086Driver::BNO086Driver(I2CBusManager& bus_manager, uint16_t address)
  : bus_manager_(bus_manager), address_(address) {
  memset(sequence_number_, 0, sizeof(sequence_number_));
}

void BNO086Driver::Init(Callback cb) {
  SoftReset([this, cb](esp_err_t err) {
    if (err != ESP_OK) {
      if (cb) cb(err);
      return;
    }
    // The driver sends an advertisement packet after reset.
    // We poll for it to ensure the driver is ready.
    PollForAdvertisement(100, cb);
  });
}

void BNO086Driver::PollForAdvertisement(int attempts_left, Callback cb) {
  bus_manager_.Enqueue([this](BusToken& token) {
    return ReceivePacket(token, 10);
  }, [this, attempts_left, cb](esp_err_t err) {
    if (err == ESP_OK) {
      ParsePacket();
      if (cb) cb(ESP_OK);
    } else if (attempts_left > 0) {
      PollForAdvertisement(attempts_left - 1, cb);
    } else {
      ESP_LOGW(TAG, "Did not receive advertisement packet, but continuing...");
      if (cb) cb(ESP_OK);
    }
  }, pdMS_TO_TICKS(10));
}

void BNO086Driver::SoftReset(Callback cb) {
  bus_manager_.Enqueue([this](BusToken& token) {
    memset(buffer_, 0, sizeof(buffer_));
    buffer_[4] = 1; // Reset command in executable channel
    return SendPacket(token, CHANNEL_EXECUTABLE, 1);
  }, [this, cb](esp_err_t err) {
    if (err != ESP_OK) {
      if (cb) cb(err);
      return;
    }
    // Wait for reset to complete
    bus_manager_.Enqueue([this](BusToken& token) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      memset(sequence_number_, 0, sizeof(sequence_number_));
      sh2_sequence_number_ = 0;
      return ESP_OK;
    }, cb, pdMS_TO_TICKS(200));
  });
}

void BNO086Driver::Update(Callback cb) {
  bus_manager_.Enqueue([this](BusToken& token) {
    int max_packets = 10;
    while (max_packets-- > 0) {
      // Use a small but non-zero timeout for polling
      esp_err_t err = ReceivePacket(token, 2);
      if (err == ESP_OK) {
        ParsePacket();
      } else {
        break; // No more packets or error
      }
    }
    return ESP_OK;
  }, cb);
}

esp_err_t BNO086Driver::SendPacket(BusToken& token, uint8_t channel, uint16_t len) {
  uint16_t total_len = len + 4;
  buffer_[0] = total_len & 0xFF;
  buffer_[1] = (total_len >> 8) & 0xFF;
  buffer_[2] = channel;
  buffer_[3] = sequence_number_[channel]++;

  return bus_manager_.Write(token, address_, buffer_, total_len);
}

esp_err_t BNO086Driver::ReceivePacket(BusToken& token, uint16_t timeout_ms) {
  // To avoid the BNO08x discarding packets upon an I2C STOP condition between
  // header and payload reads, we perform a single transaction reading the
  // maximum possible packet size we can handle.
  esp_err_t err = bus_manager_.Read(token, address_, buffer_, sizeof(buffer_), timeout_ms);

  // Even if err != ESP_OK, we check if we have a valid-looking header.
  uint16_t length = (buffer_[1] << 8) | buffer_[0];
  length &= 0x7FFF;

  if (length < 4 || length > sizeof(buffer_)) {
    return (err == ESP_OK) ? ESP_ERR_NOT_FOUND : err;
  }

  // If we have a valid header and length, we consider it a success even if
  // the full 256-byte read didn't complete (due to NACK at end of packet).
  return ESP_OK;
}

void BNO086Driver::ParsePacket() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  uint8_t channel = buffer_[2];
  uint16_t length = (buffer_[1] << 8) | buffer_[0];
  length &= 0x7FFF;

  if (length < 4) return;
  uint16_t payload_len = length - 4;
  uint8_t* payload = buffer_ + 4;

  if (channel == CHANNEL_REPORTS || channel == CHANNEL_WAKE_REPORTS) {
    ParseSH2Report(payload, payload_len);
  } else if (channel == CHANNEL_GYRO_INTEGRATED) {
    ParseGyroIntegratedReport(payload, payload_len);
  }
}

static float qToFloat(int16_t fixed_point, int8_t q_point) {
  return ldexpf((float)fixed_point, -q_point);
}

void BNO086Driver::ParseGyroIntegratedReport(uint8_t* payload, uint16_t len) {
  if (len < 15) return;
  int16_t raw_av_x = (payload[2] << 8) | payload[1];
  int16_t raw_av_y = (payload[4] << 8) | payload[3];
  int16_t raw_av_z = (payload[6] << 8) | payload[5];
  int16_t raw_i = (payload[8] << 8) | payload[7];
  int16_t raw_j = (payload[10] << 8) | payload[9];
  int16_t raw_k = (payload[12] << 8) | payload[11];
  int16_t raw_real = (payload[14] << 8) | payload[13];

  gyro_integrated_av_.x = qToFloat(raw_av_x, 10);
  gyro_integrated_av_.y = qToFloat(raw_av_y, 10);
  gyro_integrated_av_.z = qToFloat(raw_av_z, 10);
  gyro_integrated_rv_.i = qToFloat(raw_i, 14);
  gyro_integrated_rv_.j = qToFloat(raw_j, 14);
  gyro_integrated_rv_.k = qToFloat(raw_k, 14);
  gyro_integrated_rv_.real = qToFloat(raw_real, 14);
  gyro_integrated_rv_.accuracy = 0;
}

void BNO086Driver::ParseSH2Report(uint8_t* payload, uint16_t len) {
  if (len < 1) return;

  uint16_t curr = 0;
  if (payload[curr] == 0xFB) {
    curr += 5; // ID + 4 bytes timestamp
  }

  while (curr < len) {
    uint8_t report_id = payload[curr];
    if (curr + 1 >= len) break;

    switch (report_id) {
      case DRIVER_REPORTID_ACCELEROMETER: {
        if (curr + 9 >= len) return;
        int16_t raw_x = (payload[curr + 5] << 8) | payload[curr + 4];
        int16_t raw_y = (payload[curr + 7] << 8) | payload[curr + 6];
        int16_t raw_z = (payload[curr + 9] << 8) | payload[curr + 8];
        accel_.x = qToFloat(raw_x, 8);
        accel_.y = qToFloat(raw_y, 8);
        accel_.z = qToFloat(raw_z, 8);
        accel_.accuracy = payload[curr + 2];
        curr += 10;
        break;
      }
      case DRIVER_REPORTID_GYROSCOPE: {
        if (curr + 9 >= len) return;
        int16_t raw_x = (payload[curr + 5] << 8) | payload[curr + 4];
        int16_t raw_y = (payload[curr + 7] << 8) | payload[curr + 6];
        int16_t raw_z = (payload[curr + 9] << 8) | payload[curr + 8];
        gyro_.x = qToFloat(raw_x, 9);
        gyro_.y = qToFloat(raw_y, 9);
        gyro_.z = qToFloat(raw_z, 9);
        gyro_.accuracy = payload[curr + 2];
        curr += 10;
        break;
      }
      case DRIVER_REPORTID_MAGNETIC_FIELD: {
        if (curr + 9 >= len) return;
        int16_t raw_x = (payload[curr + 5] << 8) | payload[curr + 4];
        int16_t raw_y = (payload[curr + 7] << 8) | payload[curr + 6];
        int16_t raw_z = (payload[curr + 9] << 8) | payload[curr + 8];
        mag_.x = qToFloat(raw_x, 4);
        mag_.y = qToFloat(raw_y, 4);
        mag_.z = qToFloat(raw_z, 4);
        mag_.accuracy = payload[curr + 2];
        curr += 10;
        break;
      }
      case DRIVER_REPORTID_LINEAR_ACCELERATION: {
        if (curr + 9 >= len) return;
        int16_t raw_x = (payload[curr + 5] << 8) | payload[curr + 4];
        int16_t raw_y = (payload[curr + 7] << 8) | payload[curr + 6];
        int16_t raw_z = (payload[curr + 9] << 8) | payload[curr + 8];
        linear_accel_.x = qToFloat(raw_x, 8);
        linear_accel_.y = qToFloat(raw_y, 8);
        linear_accel_.z = qToFloat(raw_z, 8);
        linear_accel_.accuracy = payload[curr + 2];
        curr += 10;
        break;
      }
      case DRIVER_REPORTID_GRAVITY: {
        if (curr + 9 >= len) return;
        int16_t raw_x = (payload[curr + 5] << 8) | payload[curr + 4];
        int16_t raw_y = (payload[curr + 7] << 8) | payload[curr + 6];
        int16_t raw_z = (payload[curr + 9] << 8) | payload[curr + 8];
        gravity_.x = qToFloat(raw_x, 8);
        gravity_.y = qToFloat(raw_y, 8);
        gravity_.z = qToFloat(raw_z, 8);
        gravity_.accuracy = payload[curr + 2];
        curr += 10;
        break;
      }
      case DRIVER_REPORTID_ROTATION_VECTOR:
      case DRIVER_REPORTID_GAME_ROTATION_VECTOR:
      case DRIVER_REPORTID_ARVR_STABILIZED_ROTATION_VECTOR:
      case DRIVER_REPORTID_ARVR_STABILIZED_GAME_ROTATION_VECTOR: {
        if (curr + 11 >= len) return;
        int16_t raw_i = (payload[curr + 5] << 8) | payload[curr + 4];
        int16_t raw_j = (payload[curr + 7] << 8) | payload[curr + 6];
        int16_t raw_k = (payload[curr + 9] << 8) | payload[curr + 8];
        int16_t raw_real = (payload[curr + 11] << 8) | payload[curr + 10];

        Quaternion q;
        q.i = qToFloat(raw_i, 14);
        q.j = qToFloat(raw_j, 14);
        q.k = qToFloat(raw_k, 14);
        q.real = qToFloat(raw_real, 14);

        uint8_t size = 12;
        if (report_id == DRIVER_REPORTID_ROTATION_VECTOR ||
            report_id == DRIVER_REPORTID_ARVR_STABILIZED_ROTATION_VECTOR) {
          if (curr + 13 >= len) return;
          int16_t raw_acc = (payload[curr + 13] << 8) | payload[curr + 12];
          q.accuracy = qToFloat(raw_acc, 12);
          size = 14;
        } else {
          q.accuracy = 0;
        }

        if (report_id == DRIVER_REPORTID_ROTATION_VECTOR) {
          rotation_vector_ = q;
        } else if (report_id == DRIVER_REPORTID_GAME_ROTATION_VECTOR) {
          game_rotation_vector_ = q;
        } else if (report_id == DRIVER_REPORTID_ARVR_STABILIZED_ROTATION_VECTOR) {
          arvr_rotation_vector_ = q;
        } else if (report_id == DRIVER_REPORTID_ARVR_STABILIZED_GAME_ROTATION_VECTOR) {
          arvr_game_rotation_vector_ = q;
        }

        curr += size;
        break;
      }
      case DRIVER_REPORTID_STEP_COUNTER: {
        if (curr + 11 >= len) return;
        step_counter_.count = (payload[curr + 9] << 8) | payload[curr + 8];
        step_counter_.latency = (payload[curr + 7] << 24) | (payload[curr + 6] << 16) |
                                (payload[curr + 5] << 8) | payload[curr + 4];
        curr += 12;
        break;
      }
      case DRIVER_REPORTID_STABILITY_CLASSIFIER: {
        if (curr + 4 >= len) return;
        stability_ = static_cast<Stability>(payload[curr + 4]);
        curr += 6;
        break;
      }
      default:
        curr = len;
        break;
    }
  }
}

void BNO086Driver::SetFeature(uint8_t report_id, uint32_t period_us, Callback cb) {
  bus_manager_.Enqueue([this, report_id, period_us](BusToken& token) {
    uint8_t cmd_payload[21];
    memset(cmd_payload, 0, 21);
    cmd_payload[0] = SHTP_REPORT_SET_FEATURE_COMMAND;
    cmd_payload[1] = report_id;
    cmd_payload[5] = period_us & 0xFF;
    cmd_payload[6] = (period_us >> 8) & 0xFF;
    cmd_payload[7] = (period_us >> 16) & 0xFF;
    cmd_payload[8] = (period_us >> 24) & 0xFF;

    memcpy(buffer_ + 4, cmd_payload, 21);
    return SendPacket(token, CHANNEL_CONTROL, 21);
  }, cb);
}

void BNO086Driver::EnableAccelerometer(uint32_t period_us, Callback cb) {
  SetFeature(DRIVER_REPORTID_ACCELEROMETER, period_us, cb);
}
void BNO086Driver::EnableGyroscope(uint32_t period_us, Callback cb) {
  SetFeature(DRIVER_REPORTID_GYROSCOPE, period_us, cb);
}
void BNO086Driver::EnableMagnetometer(uint32_t period_us, Callback cb) {
  SetFeature(DRIVER_REPORTID_MAGNETIC_FIELD, period_us, cb);
}
void BNO086Driver::EnableLinearAcceleration(uint32_t period_us, Callback cb) {
  SetFeature(DRIVER_REPORTID_LINEAR_ACCELERATION, period_us, cb);
}
void BNO086Driver::EnableGravity(uint32_t period_us, Callback cb) {
  SetFeature(DRIVER_REPORTID_GRAVITY, period_us, cb);
}
void BNO086Driver::EnableRotationVector(uint32_t period_us, Callback cb) {
  SetFeature(DRIVER_REPORTID_ROTATION_VECTOR, period_us, cb);
}
void BNO086Driver::EnableGameRotationVector(uint32_t period_us, Callback cb) {
  SetFeature(DRIVER_REPORTID_GAME_ROTATION_VECTOR, period_us, cb);
}
void BNO086Driver::EnableARVRStabilizedRotationVector(uint32_t period_us, Callback cb) {
  SetFeature(DRIVER_REPORTID_ARVR_STABILIZED_ROTATION_VECTOR, period_us, cb);
}
void BNO086Driver::EnableARVRStabilizedGameRotationVector(uint32_t period_us, Callback cb) {
  SetFeature(DRIVER_REPORTID_ARVR_STABILIZED_GAME_ROTATION_VECTOR, period_us, cb);
}
void BNO086Driver::EnableStepCounter(uint32_t period_us, Callback cb) {
  SetFeature(DRIVER_REPORTID_STEP_COUNTER, period_us, cb);
}
void BNO086Driver::EnableStabilityClassifier(uint32_t period_us, Callback cb) {
  SetFeature(DRIVER_REPORTID_STABILITY_CLASSIFIER, period_us, cb);
}
void BNO086Driver::EnableGyroIntegratedRotationVector(uint32_t period_us, Callback cb) {
  SetFeature(DRIVER_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, period_us, cb);
}

void BNO086Driver::SetCalibrationConfig(bool accel, bool gyro, bool mag, Callback cb) {
  bus_manager_.Enqueue([this, accel, gyro, mag](BusToken& token) {
    uint8_t cmd_payload[12];
    memset(cmd_payload, 0, 12);
    cmd_payload[0] = SHTP_REPORT_COMMAND_REQUEST;
    cmd_payload[1] = sh2_sequence_number_++;
    cmd_payload[2] = SH2_COMMAND_ME_CALIBRATION;
    cmd_payload[3] = accel ? 1 : 0;
    cmd_payload[4] = gyro ? 1 : 0;
    cmd_payload[5] = mag ? 1 : 0;
    cmd_payload[6] = 0x00; // Planar

    memcpy(buffer_ + 4, cmd_payload, 12);
    return SendPacket(token, CHANNEL_CONTROL, 12);
  }, cb);
}

void BNO086Driver::SaveCalibration(Callback cb) {
  bus_manager_.Enqueue([this](BusToken& token) {
    uint8_t cmd_payload[3];
    cmd_payload[0] = SHTP_REPORT_COMMAND_REQUEST;
    cmd_payload[1] = sh2_sequence_number_++;
    cmd_payload[2] = SH2_COMMAND_SAVE_DCD;

    memcpy(buffer_ + 4, cmd_payload, 3);
    return SendPacket(token, CHANNEL_CONTROL, 3);
  }, cb);
}

void BNO086Driver::SetPowerMode(bool sleep, Callback cb) {
  bus_manager_.Enqueue([this, sleep](BusToken& token) {
    uint8_t cmd_payload[4];
    cmd_payload[0] = SHTP_REPORT_COMMAND_REQUEST;
    cmd_payload[1] = sh2_sequence_number_++;
    cmd_payload[2] = SH2_COMMAND_SET_POWER_STATE;
    cmd_payload[3] = sleep ? 1 : 0; // 0 = ON, 1 = SLEEP

    memcpy(buffer_ + 4, cmd_payload, 4);
    return SendPacket(token, CHANNEL_CONTROL, 4);
  }, cb);
}

// Getters
BNO086Driver::Vector3 BNO086Driver::GetAccelerometer() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return accel_;
}
BNO086Driver::Vector3 BNO086Driver::GetGyroscope() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return gyro_;
}
BNO086Driver::Vector3 BNO086Driver::GetMagnetometer() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return mag_;
}
BNO086Driver::Vector3 BNO086Driver::GetLinearAcceleration() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return linear_accel_;
}
BNO086Driver::Vector3 BNO086Driver::GetGravity() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return gravity_;
}
BNO086Driver::Quaternion BNO086Driver::GetRotationVector() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return rotation_vector_;
}
BNO086Driver::Quaternion BNO086Driver::GetGameRotationVector() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return game_rotation_vector_;
}
BNO086Driver::Quaternion BNO086Driver::GetARVRStabilizedRotationVector() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return arvr_rotation_vector_;
}
BNO086Driver::Quaternion BNO086Driver::GetARVRStabilizedGameRotationVector() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return arvr_game_rotation_vector_;
}
BNO086Driver::Quaternion BNO086Driver::GetGyroIntegratedRotationVector() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return gyro_integrated_rv_;
}
BNO086Driver::Vector3 BNO086Driver::GetGyroIntegratedAngularVelocity() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return gyro_integrated_av_;
}
uint16_t BNO086Driver::GetStepCount() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return step_counter_.count;
}
BNO086Driver::Stability BNO086Driver::GetStability() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return stability_;
}

} // namespace ALC
