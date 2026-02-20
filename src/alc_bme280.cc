#include "alc_bme280.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

static const char* TAG = "BME280";

// BME280 Registers
#define BME280_REG_ID          0xD0
#define BME280_REG_RESET       0xE0
#define BME280_REG_CTRL_HUM    0xF2
#define BME280_REG_STATUS      0xF3
#define BME280_REG_CTRL_MEAS   0xF4
#define BME280_REG_CONFIG      0xF5
#define BME280_REG_PRESS_MSB   0xF7
#define BME280_REG_CALIB_00    0x88
#define BME280_REG_CALIB_H1    0xA1
#define BME280_REG_CALIB_26    0xE1

#define BME280_ID              0x60
#define BME280_RESET_VALUE     0xB6

namespace ALC {

BME280::BME280(i2c_port_t i2c_port, uint8_t address)
    : i2c_port_(i2c_port), address_(address) {}

esp_err_t BME280::Init() {
  uint8_t id;
  esp_err_t err = ReadRegisters(BME280_REG_ID, &id, 1);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read ID register (0x%x)", err);
    return err;
  }

  if (id != BME280_ID) {
    ESP_LOGE(TAG, "Device ID mismatch: expected 0x%02x, got 0x%02x", BME280_ID, id);
    return ESP_ERR_NOT_FOUND;
  }

  // Reset the device
  err = WriteRegister(BME280_REG_RESET, BME280_RESET_VALUE);
  if (err != ESP_OK) return err;
  vTaskDelay(pdMS_TO_TICKS(10)); // Wait for reset

  // Read calibration data
  err = ReadCalibrationData();
  if (err != ESP_OK) return err;

  // Apply default configuration
  return ApplyConfiguration();
}

esp_err_t BME280::Configure(const Configuration& config) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  config_ = config;
  return ApplyConfiguration();
}

esp_err_t BME280::ApplyConfiguration() {
  // Humidity oversampling
  esp_err_t err = WriteRegister(BME280_REG_CTRL_HUM, static_cast<uint8_t>(config_.hum_os));
  if (err != ESP_OK) return err;

  // Config: standby time and filter
  uint8_t config_val = (static_cast<uint8_t>(config_.standby) << 5) | (static_cast<uint8_t>(config_.filter) << 2);
  err = WriteRegister(BME280_REG_CONFIG, config_val);
  if (err != ESP_OK) return err;

  // CTRL_MEAS: temp oversampling, press oversampling, and mode
  // Note: Writing to CTRL_MEAS triggers the changes for CTRL_HUM too.
  uint8_t ctrl_meas = (static_cast<uint8_t>(config_.temp_os) << 5) |
                      (static_cast<uint8_t>(config_.press_os) << 2) |
                      static_cast<uint8_t>(config_.mode);
  err = WriteRegister(BME280_REG_CTRL_MEAS, ctrl_meas);
  if (err != ESP_OK) return err;

  ESP_LOGI(TAG, "Configuration applied");
  return ESP_OK;
}

esp_err_t BME280::ReadAll() {
  SensorMode current_mode;
  Configuration current_config;

  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    current_mode = config_.mode;
    current_config = config_;
  }

  if (current_mode == SensorMode::FORCED) {
    // Trigger Forced Mode measurement
    uint8_t ctrl_meas = (static_cast<uint8_t>(current_config.temp_os) << 5) |
                        (static_cast<uint8_t>(current_config.press_os) << 2) |
                        static_cast<uint8_t>(SensorMode::FORCED);
    esp_err_t err = WriteRegister(BME280_REG_CTRL_MEAS, ctrl_meas);
    if (err != ESP_OK) return err;

    // Wait for measurement to complete
    uint8_t status;
    int retry = 10;
    while (retry--) {
      err = ReadRegisters(BME280_REG_STATUS, &status, 1);
      if (err != ESP_OK) return err;
      if (!(status & 0x08)) break; // Bit 3 is 'measuring'
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (retry < 0) {
      ESP_LOGE(TAG, "Timeout waiting for measurement");
      return ESP_ERR_TIMEOUT;
    }
  }

  uint8_t data[8];
  esp_err_t err = ReadRegisters(BME280_REG_PRESS_MSB, data, 8);
  if (err != ESP_OK) return err;

  int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
  int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
  int32_t adc_H = (data[6] << 8) | data[7];

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  // Temperature compensation
  int32_t var1, var2;
  var1 = ((((adc_T >> 3) - ((int32_t)calib_.dig_T1 << 1))) * ((int32_t)calib_.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)calib_.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib_.dig_T1))) >> 12) * ((int32_t)calib_.dig_T3)) >> 14;
  t_fine_ = var1 + var2;
  temperature_ = ((t_fine_ * 5 + 128) >> 8) / 100.0f;

  // Pressure compensation
  int64_t v1, v2, p;
  v1 = ((int64_t)t_fine_) - 128000;
  v2 = v1 * v1 * (int64_t)calib_.dig_P6;
  v2 = v2 + ((v1 * (int64_t)calib_.dig_P5) << 17);
  v2 = v2 + (((int64_t)calib_.dig_P4) << 35);
  v1 = ((v1 * v1 * (int64_t)calib_.dig_P3) >> 8) + ((v1 * (int64_t)calib_.dig_P2) << 12);
  v1 = (((((int64_t)1) << 47) + v1)) * ((int64_t)calib_.dig_P1) >> 33;
  if (v1 != 0) {
    p = 1048576 - adc_P;
    p = (((p << 31) - v2) * 3125) / v1;
    v1 = (((int64_t)calib_.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    v2 = (((int64_t)calib_.dig_P8) * p) >> 19;
    p = ((p + v1 + v2) >> 8) + (((int64_t)calib_.dig_P7) << 4);
    pressure_ = (float)p / 25600.0f;
  }

  // Humidity compensation
  int32_t v_x1_u32r;
  v_x1_u32r = (t_fine_ - ((int32_t)76800));
  v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib_.dig_H4) << 20) - (((int32_t)calib_.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
               (((((((v_x1_u32r * ((int32_t)calib_.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)calib_.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                 ((int32_t)calib_.dig_H2) + 8192) >> 14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib_.dig_H1)) >> 4));
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
  humidity_ = (float)(v_x1_u32r >> 12) / 1024.0f;

  return ESP_OK;
}

float BME280::GetTemperature() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return temperature_;
}

float BME280::GetPressure() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return pressure_;
}

float BME280::GetHumidity() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return humidity_;
}

esp_err_t BME280::ReadCalibrationData() {
  uint8_t data[24];
  esp_err_t err = ReadRegisters(BME280_REG_CALIB_00, data, 24);
  if (err != ESP_OK) return err;

  calib_.dig_T1 = (data[1] << 8) | data[0];
  calib_.dig_T2 = (data[3] << 8) | data[2];
  calib_.dig_T3 = (data[5] << 8) | data[4];
  calib_.dig_P1 = (data[7] << 8) | data[6];
  calib_.dig_P2 = (data[9] << 8) | data[8];
  calib_.dig_P3 = (data[11] << 8) | data[10];
  calib_.dig_P4 = (data[13] << 8) | data[12];
  calib_.dig_P5 = (data[15] << 8) | data[14];
  calib_.dig_P6 = (data[17] << 8) | data[16];
  calib_.dig_P7 = (data[19] << 8) | data[18];
  calib_.dig_P8 = (data[21] << 8) | data[20];
  calib_.dig_P9 = (data[23] << 8) | data[22];

  err = ReadRegisters(BME280_REG_CALIB_H1, &calib_.dig_H1, 1);
  if (err != ESP_OK) return err;

  uint8_t h_data[7];
  err = ReadRegisters(BME280_REG_CALIB_26, h_data, 7);
  if (err != ESP_OK) return err;

  calib_.dig_H2 = (h_data[1] << 8) | h_data[0];
  calib_.dig_H3 = h_data[2];
  calib_.dig_H4 = (h_data[3] << 4) | (h_data[4] & 0x0F);
  if (calib_.dig_H4 & (1 << 11)) calib_.dig_H4 |= 0xF000;
  calib_.dig_H5 = (h_data[5] << 4) | (h_data[4] >> 4);
  if (calib_.dig_H5 & (1 << 11)) calib_.dig_H5 |= 0xF000;
  calib_.dig_H6 = (int8_t)h_data[6];

  return ESP_OK;
}

esp_err_t BME280::WriteRegister(uint8_t reg, uint8_t value) {
  uint8_t data[2] = {reg, value};
  return i2c_master_write_to_device(i2c_port_, address_, data, 2, pdMS_TO_TICKS(100));
}

esp_err_t BME280::ReadRegisters(uint8_t reg, uint8_t* data, size_t len) {
  return i2c_master_write_read_device(i2c_port_, address_, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

} // namespace ALC
