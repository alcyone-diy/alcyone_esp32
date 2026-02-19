#include "alc_bno086.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <cmath>

static const char* TAG = "BNO086";

namespace ALC {

BNO086::BNO086(i2c_port_t i2c_port, uint8_t address, uint32_t i2c_timeout_ms)
  : i2c_port_(i2c_port), address_(address), i2c_timeout_ms_(i2c_timeout_ms) {
  memset(sequence_number_, 0, sizeof(sequence_number_));
}

BNO086::~BNO086() {
  Close();
}

esp_err_t BNO086::Open() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  ESP_LOGI(TAG, "Opening BNO086 at address 0x%02X", address_);

  // Perform soft reset to ensure clean state
  esp_err_t err = SoftReset();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Soft reset failed: %s", esp_err_to_name(err));
    return err;
  }

  // The sensor sends an advertisement packet after reset.
  // We poll for it to ensure the sensor is ready.
  bool ready = false;
  for (int i = 0; i < 100; ++i) {
    if (ReceivePacket(10) == ESP_OK) {
      ParsePacket();
      ready = true;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  if (!ready) {
    ESP_LOGW(TAG, "Did not receive advertisement packet, but continuing...");
  }

  return ESP_OK;
}

esp_err_t BNO086::Close() {
  return ESP_OK;
}

esp_err_t BNO086::SoftReset() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  memset(buffer_, 0, sizeof(buffer_));
  buffer_[4] = 1; // Reset command in executable channel
  esp_err_t err = SendPacket(CHANNEL_EXECUTABLE, 1);
  if (err != ESP_OK) return err;

  vTaskDelay(pdMS_TO_TICKS(200)); // Wait for reset to complete
  memset(sequence_number_, 0, sizeof(sequence_number_));
  sh2_sequence_number_ = 0;
  return ESP_OK;
}

esp_err_t BNO086::Update() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  int max_packets = 10;
  while (max_packets-- > 0) {
    // Use a small but non-zero timeout for polling
    esp_err_t err = ReceivePacket(2);
    if (err == ESP_OK) {
      ParsePacket();
    } else {
      break; // No more packets or error
    }
  }
  return ESP_OK;
}

esp_err_t BNO086::SendPacket(uint8_t channel, uint16_t len) {
  uint16_t total_len = len + 4;
  buffer_[0] = total_len & 0xFF;
  buffer_[1] = (total_len >> 8) & 0xFF;
  buffer_[2] = channel;
  buffer_[3] = sequence_number_[channel]++;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address_ << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write(cmd, buffer_, total_len, true);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(i2c_port_, cmd, pdMS_TO_TICKS(i2c_timeout_ms_));
  i2c_cmd_link_delete(cmd);

  return err;
}

esp_err_t BNO086::ReceivePacket(uint16_t timeout_ms) {
  // To avoid the BNO08x discarding packets upon an I2C STOP condition between
  // header and payload reads, we perform a single transaction reading the
  // maximum possible packet size we can handle.
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address_ << 1) | I2C_MASTER_READ, true);
  // We read 256 bytes. The sensor will NACK when it has no more data to send.
  i2c_master_read(cmd, buffer_, sizeof(buffer_), I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);

  // Note: This transaction might return ESP_FAIL if the sensor NACKs before
  // 256 bytes are read. However, the data already read into the buffer
  // should be valid.
  esp_err_t err = i2c_master_cmd_begin(i2c_port_, cmd, pdMS_TO_TICKS(timeout_ms > 0 ? timeout_ms : i2c_timeout_ms_));
  i2c_cmd_link_delete(cmd);

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

void BNO086::ParsePacket() {
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
  } else if (channel == CHANNEL_CONTROL) {
    // Command responses could be handled here
  }
}

static float qToFloat(int16_t fixed_point, int8_t q_point) {
  return ldexpf((float)fixed_point, -q_point);
}

void BNO086::ParseGyroIntegratedReport(uint8_t* payload, uint16_t len) {
  if (len < 15) return;
  // payload[0] is sequence
  // payload[1..6] is angular velocity (Q10)
  // payload[7..14] is quaternion (Q14)
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

void BNO086::ParseSH2Report(uint8_t* payload, uint16_t len) {
  if (len < 1) return;

  uint16_t curr = 0;
  // Check if it's a timestamp base report
  if (payload[curr] == 0xFB) {
    curr += 5; // ID + 4 bytes timestamp
  }

  while (curr < len) {
    uint8_t report_id = payload[curr];
    if (curr + 1 >= len) break;

    switch (report_id) {
      case SENSOR_REPORTID_ACCELEROMETER: {
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
      case SENSOR_REPORTID_GYROSCOPE: {
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
      case SENSOR_REPORTID_MAGNETIC_FIELD: {
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
      case SENSOR_REPORTID_LINEAR_ACCELERATION: {
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
      case SENSOR_REPORTID_GRAVITY: {
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
      case SENSOR_REPORTID_ROTATION_VECTOR:
      case SENSOR_REPORTID_GAME_ROTATION_VECTOR:
      case SENSOR_REPORTID_ARVR_STABILIZED_ROTATION_VECTOR:
      case SENSOR_REPORTID_ARVR_STABILIZED_GAME_ROTATION_VECTOR: {
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
        if (report_id == SENSOR_REPORTID_ROTATION_VECTOR || report_id == SENSOR_REPORTID_ARVR_STABILIZED_ROTATION_VECTOR) {
          if (curr + 13 >= len) return;
          int16_t raw_acc = (payload[curr + 13] << 8) | payload[curr + 12];
          q.accuracy = qToFloat(raw_acc, 12);
          size = 14;
        } else {
          q.accuracy = 0;
        }

        if (report_id == SENSOR_REPORTID_ROTATION_VECTOR) rotation_vector_ = q;
        else if (report_id == SENSOR_REPORTID_GAME_ROTATION_VECTOR) game_rotation_vector_ = q;
        else if (report_id == SENSOR_REPORTID_ARVR_STABILIZED_ROTATION_VECTOR) arvr_rotation_vector_ = q;
        else if (report_id == SENSOR_REPORTID_ARVR_STABILIZED_GAME_ROTATION_VECTOR) arvr_game_rotation_vector_ = q;

        curr += size;
        break;
      }
      case SENSOR_REPORTID_STEP_COUNTER: {
        if (curr + 11 >= len) return;
        step_counter_.count = (payload[curr + 9] << 8) | payload[curr + 8];
        // Latency is 4 bytes at [4..7]
        step_counter_.latency = (payload[curr + 7] << 24) | (payload[curr + 6] << 16) | (payload[curr + 5] << 8) | payload[curr + 4];
        curr += 12;
        break;
      }
      case SENSOR_REPORTID_STABILITY_CLASSIFIER: {
        if (curr + 4 >= len) return;
        stability_ = static_cast<Stability>(payload[curr + 4]);
        curr += 6;
        break;
      }
      default:
        // Unknown report, skip to end
        curr = len;
        break;
    }
  }
}

esp_err_t BNO086::SetFeature(uint8_t report_id, uint32_t period_us) {
  uint8_t cmd_payload[21];
  memset(cmd_payload, 0, 21);
  cmd_payload[0] = SHTP_REPORT_SET_FEATURE_COMMAND;
  cmd_payload[1] = report_id;
  cmd_payload[5] = period_us & 0xFF;
  cmd_payload[6] = (period_us >> 8) & 0xFF;
  cmd_payload[7] = (period_us >> 16) & 0xFF;
  cmd_payload[8] = (period_us >> 24) & 0xFF;

  memcpy(buffer_ + 4, cmd_payload, 21);
  return SendPacket(CHANNEL_CONTROL, 21);
}

esp_err_t BNO086::EnableAccelerometer(uint32_t period_us) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return SetFeature(SENSOR_REPORTID_ACCELEROMETER, period_us);
}
esp_err_t BNO086::EnableGyroscope(uint32_t period_us) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return SetFeature(SENSOR_REPORTID_GYROSCOPE, period_us);
}
esp_err_t BNO086::EnableMagnetometer(uint32_t period_us) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return SetFeature(SENSOR_REPORTID_MAGNETIC_FIELD, period_us);
}
esp_err_t BNO086::EnableLinearAcceleration(uint32_t period_us) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return SetFeature(SENSOR_REPORTID_LINEAR_ACCELERATION, period_us);
}
esp_err_t BNO086::EnableGravity(uint32_t period_us) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return SetFeature(SENSOR_REPORTID_GRAVITY, period_us);
}
esp_err_t BNO086::EnableRotationVector(uint32_t period_us) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return SetFeature(SENSOR_REPORTID_ROTATION_VECTOR, period_us);
}
esp_err_t BNO086::EnableGameRotationVector(uint32_t period_us) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return SetFeature(SENSOR_REPORTID_GAME_ROTATION_VECTOR, period_us);
}
esp_err_t BNO086::EnableARVRStabilizedRotationVector(uint32_t period_us) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return SetFeature(SENSOR_REPORTID_ARVR_STABILIZED_ROTATION_VECTOR, period_us);
}
esp_err_t BNO086::EnableARVRStabilizedGameRotationVector(uint32_t period_us) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return SetFeature(SENSOR_REPORTID_ARVR_STABILIZED_GAME_ROTATION_VECTOR, period_us);
}
esp_err_t BNO086::EnableStepCounter(uint32_t period_us) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return SetFeature(SENSOR_REPORTID_STEP_COUNTER, period_us);
}
esp_err_t BNO086::EnableStabilityClassifier(uint32_t period_us) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return SetFeature(SENSOR_REPORTID_STABILITY_CLASSIFIER, period_us);
}
esp_err_t BNO086::EnableGyroIntegratedRotationVector(uint32_t period_us) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return SetFeature(SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, period_us);
}

esp_err_t BNO086::SetCalibrationConfig(bool accel, bool gyro, bool mag) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  uint8_t cmd_payload[12];
  memset(cmd_payload, 0, 12);
  cmd_payload[0] = SHTP_REPORT_COMMAND_REQUEST;
  cmd_payload[1] = sh2_sequence_number_++;
  cmd_payload[2] = 0x07; // ME Calibration
  cmd_payload[3] = accel ? 1 : 0;
  cmd_payload[4] = gyro ? 1 : 0;
  cmd_payload[5] = mag ? 1 : 0;
  cmd_payload[6] = 0x00; // Planar

  memcpy(buffer_ + 4, cmd_payload, 12);
  return SendPacket(CHANNEL_CONTROL, 12);
}

esp_err_t BNO086::SaveCalibration() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  uint8_t cmd_payload[3];
  cmd_payload[0] = SHTP_REPORT_COMMAND_REQUEST;
  cmd_payload[1] = sh2_sequence_number_++;
  cmd_payload[2] = 0x06; // Save DCD

  memcpy(buffer_ + 4, cmd_payload, 3);
  return SendPacket(CHANNEL_CONTROL, 3);
}

} // namespace ALC
