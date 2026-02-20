#include "alc_bno086.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <cstring>
#include <cmath>

static const char* TAG = "BNO086";

namespace ALC {

BNO086::BNO086(I2CBusManager& i2c_bus, uint8_t address, uint32_t i2c_timeout_ms)
  : i2c_bus_(i2c_bus), address_(address), i2c_timeout_ms_(i2c_timeout_ms) {
  memset(sequence_number_, 0, sizeof(sequence_number_));
}

BNO086::~BNO086() {
  Close();
}

esp_err_t BNO086::Open() {
  SemaphoreHandle_t done = xSemaphoreCreateBinary();
  esp_err_t result = ESP_FAIL;
  OpenAsync([&result, done](esp_err_t err) {
    result = err;
    xSemaphoreGive(done);
  });
  xSemaphoreTake(done, portMAX_DELAY);
  vSemaphoreDelete(done);
  return result;
}

void BNO086::OpenAsync(Callback cb) {
  ESP_LOGI(TAG, "Opening BNO086 at address 0x%02X", address_);

  i2c_bus_.Enqueue([this](i2c_port_t port) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    memset(buffer_, 0, sizeof(buffer_));
    buffer_[4] = 1; // Reset command in executable channel
    esp_err_t err = SendPacketInternal(port, CHANNEL_EXECUTABLE, 1);
    if (err == ESP_OK) {
      memset(sequence_number_, 0, sizeof(sequence_number_));
      sh2_sequence_number_ = 0;
    }
    return err;
  }, [this, cb](esp_err_t err) {
    if (err != ESP_OK) {
      if (cb) cb(err);
      return;
    }
    PollAdvertisementAsync(100, cb);
  }, pdMS_TO_TICKS(200));
}

void BNO086::PollAdvertisementAsync(int retries, Callback cb) {
  if (retries <= 0) {
    ESP_LOGW(TAG, "Did not receive advertisement packet, but continuing...");
    if (cb) cb(ESP_OK);
    return;
  }

  i2c_bus_.Enqueue([this](i2c_port_t port) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    esp_err_t err = ReceivePacketInternal(port, 10);
    if (err == ESP_OK) {
      ParsePacket();
      return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
  }, [this, retries, cb](esp_err_t err) {
    if (err == ESP_OK) {
      if (cb) cb(ESP_OK);
    } else {
      PollAdvertisementAsync(retries - 1, cb);
    }
  }, pdMS_TO_TICKS(10));
}

esp_err_t BNO086::Close() {
  return ESP_OK;
}

esp_err_t BNO086::SoftReset() {
  SemaphoreHandle_t done = xSemaphoreCreateBinary();
  esp_err_t result = ESP_FAIL;
  i2c_bus_.Enqueue([this](i2c_port_t port) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    memset(buffer_, 0, sizeof(buffer_));
    buffer_[4] = 1; // Reset command in executable channel
    return SendPacketInternal(port, CHANNEL_EXECUTABLE, 1);
  }, [&result, done](esp_err_t err) {
    result = err;
    xSemaphoreGive(done);
  });
  xSemaphoreTake(done, portMAX_DELAY);
  vSemaphoreDelete(done);

  if (result == ESP_OK) {
    vTaskDelay(pdMS_TO_TICKS(200));
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    memset(sequence_number_, 0, sizeof(sequence_number_));
    sh2_sequence_number_ = 0;
  }
  return result;
}

esp_err_t BNO086::Update() {
  SemaphoreHandle_t done = xSemaphoreCreateBinary();
  esp_err_t result = ESP_FAIL;
  UpdateAsync([&result, done](esp_err_t err) {
    result = err;
    xSemaphoreGive(done);
  });
  xSemaphoreTake(done, portMAX_DELAY);
  vSemaphoreDelete(done);
  return result;
}

void BNO086::UpdateAsync(Callback cb) {
  i2c_bus_.Enqueue([this](i2c_port_t port) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    int max_packets = 10;
    while (max_packets-- > 0) {
      esp_err_t err = ReceivePacketInternal(port, 2);
      if (err == ESP_OK) {
        ParsePacket();
      } else {
        break;
      }
    }
    return ESP_OK;
  }, cb);
}

esp_err_t BNO086::SendPacketInternal(i2c_port_t port, uint8_t channel, uint16_t len) {
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
  esp_err_t err = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(i2c_timeout_ms_));
  i2c_cmd_link_delete(cmd);

  return err;
}

esp_err_t BNO086::ReceivePacketInternal(i2c_port_t port, uint16_t timeout_ms) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address_ << 1) | I2C_MASTER_READ, true);
  i2c_master_read(cmd, buffer_, sizeof(buffer_), I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);

  esp_err_t err = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(timeout_ms > 0 ? timeout_ms : i2c_timeout_ms_));
  i2c_cmd_link_delete(cmd);

  uint16_t length = (buffer_[1] << 8) | buffer_[0];
  length &= 0x7FFF;

  if (length < 4 || length > sizeof(buffer_)) {
    return (err == ESP_OK) ? ESP_ERR_NOT_FOUND : err;
  }

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
  int16_t raw_av_x = (payload[2] << 8) | payload[1];
  int16_t raw_av_y = (payload[4] << 8) | payload[3];
  int16_t raw_av_z = (payload[6] << 8) | payload[5];
  int16_t raw_i = (payload[8] << 8) | payload[7];
  int16_t raw_j = (payload[10] << 8) | payload[9];
  int16_t raw_k = (payload[12] << 8) | payload[11];
  int16_t raw_real = (payload[14] << 8) | payload[13];

  std::lock_guard<std::recursive_mutex> lock(mutex_);
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
  if (payload[curr] == 0xFB) {
    curr += 5;
  }

  std::lock_guard<std::recursive_mutex> lock(mutex_);
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
        curr = len;
        break;
    }
  }
}

esp_err_t BNO086::SetFeature(uint8_t report_id, uint32_t period_us) {
  SemaphoreHandle_t done = xSemaphoreCreateBinary();
  esp_err_t result = ESP_FAIL;

  i2c_bus_.Enqueue([this, report_id, period_us](i2c_port_t port) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    uint8_t cmd_payload[21];
    memset(cmd_payload, 0, 21);
    cmd_payload[0] = SHTP_REPORT_SET_FEATURE_COMMAND;
    cmd_payload[1] = report_id;
    cmd_payload[5] = period_us & 0xFF;
    cmd_payload[6] = (period_us >> 8) & 0xFF;
    cmd_payload[7] = (period_us >> 16) & 0xFF;
    cmd_payload[8] = (period_us >> 24) & 0xFF;

    memcpy(buffer_ + 4, cmd_payload, 21);
    return SendPacketInternal(port, CHANNEL_CONTROL, 21);
  }, [&result, done](esp_err_t err) {
    result = err;
    xSemaphoreGive(done);
  });

  xSemaphoreTake(done, portMAX_DELAY);
  vSemaphoreDelete(done);
  return result;
}

esp_err_t BNO086::EnableAccelerometer(uint32_t period_us) {
  return SetFeature(SENSOR_REPORTID_ACCELEROMETER, period_us);
}
esp_err_t BNO086::EnableGyroscope(uint32_t period_us) {
  return SetFeature(SENSOR_REPORTID_GYROSCOPE, period_us);
}
esp_err_t BNO086::EnableMagnetometer(uint32_t period_us) {
  return SetFeature(SENSOR_REPORTID_MAGNETIC_FIELD, period_us);
}
esp_err_t BNO086::EnableLinearAcceleration(uint32_t period_us) {
  return SetFeature(SENSOR_REPORTID_LINEAR_ACCELERATION, period_us);
}
esp_err_t BNO086::EnableGravity(uint32_t period_us) {
  return SetFeature(SENSOR_REPORTID_GRAVITY, period_us);
}
esp_err_t BNO086::EnableRotationVector(uint32_t period_us) {
  return SetFeature(SENSOR_REPORTID_ROTATION_VECTOR, period_us);
}
esp_err_t BNO086::EnableGameRotationVector(uint32_t period_us) {
  return SetFeature(SENSOR_REPORTID_GAME_ROTATION_VECTOR, period_us);
}
esp_err_t BNO086::EnableARVRStabilizedRotationVector(uint32_t period_us) {
  return SetFeature(SENSOR_REPORTID_ARVR_STABILIZED_ROTATION_VECTOR, period_us);
}
esp_err_t BNO086::EnableARVRStabilizedGameRotationVector(uint32_t period_us) {
  return SetFeature(SENSOR_REPORTID_ARVR_STABILIZED_GAME_ROTATION_VECTOR, period_us);
}
esp_err_t BNO086::EnableStepCounter(uint32_t period_us) {
  return SetFeature(SENSOR_REPORTID_STEP_COUNTER, period_us);
}
esp_err_t BNO086::EnableStabilityClassifier(uint32_t period_us) {
  return SetFeature(SENSOR_REPORTID_STABILITY_CLASSIFIER, period_us);
}
esp_err_t BNO086::EnableGyroIntegratedRotationVector(uint32_t period_us) {
  return SetFeature(SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, period_us);
}

esp_err_t BNO086::SetCalibrationConfig(bool accel, bool gyro, bool mag) {
  SemaphoreHandle_t done = xSemaphoreCreateBinary();
  esp_err_t result = ESP_FAIL;

  i2c_bus_.Enqueue([this, accel, gyro, mag](i2c_port_t port) {
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
    return SendPacketInternal(port, CHANNEL_CONTROL, 12);
  }, [&result, done](esp_err_t err) {
    result = err;
    xSemaphoreGive(done);
  });

  xSemaphoreTake(done, portMAX_DELAY);
  vSemaphoreDelete(done);
  return result;
}

esp_err_t BNO086::SaveCalibration() {
  SemaphoreHandle_t done = xSemaphoreCreateBinary();
  esp_err_t result = ESP_FAIL;

  i2c_bus_.Enqueue([this](i2c_port_t port) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    uint8_t cmd_payload[3];
    cmd_payload[0] = SHTP_REPORT_COMMAND_REQUEST;
    cmd_payload[1] = sh2_sequence_number_++;
    cmd_payload[2] = 0x06; // Save DCD

    memcpy(buffer_ + 4, cmd_payload, 3);
    return SendPacketInternal(port, CHANNEL_CONTROL, 3);
  }, [&result, done](esp_err_t err) {
    result = err;
    xSemaphoreGive(done);
  });

  xSemaphoreTake(done, portMAX_DELAY);
  vSemaphoreDelete(done);
  return result;
}

esp_err_t BNO086::SetPowerMode(bool sleep) {
  SemaphoreHandle_t done = xSemaphoreCreateBinary();
  esp_err_t result = ESP_FAIL;

  i2c_bus_.Enqueue([this, sleep](i2c_port_t port) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    uint8_t cmd_payload[4];
    cmd_payload[0] = SHTP_REPORT_COMMAND_REQUEST;
    cmd_payload[1] = sh2_sequence_number_++;
    cmd_payload[2] = SH2_COMMAND_SET_POWER_STATE;
    cmd_payload[3] = sleep ? 1 : 0; // 0 = ON, 1 = SLEEP

    memcpy(buffer_ + 4, cmd_payload, 4);
    return SendPacketInternal(port, CHANNEL_CONTROL, 4);
  }, [&result, done](esp_err_t err) {
    result = err;
    xSemaphoreGive(done);
  });

  xSemaphoreTake(done, portMAX_DELAY);
  vSemaphoreDelete(done);
  return result;
}

} // namespace ALC
