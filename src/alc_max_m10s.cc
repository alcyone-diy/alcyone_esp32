#include "alc_max_m10s.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <cstring>

static const char* TAG = "MaxM10S";

namespace ALC {

MaxM10S::MaxM10S(I2CBusManager& i2c_bus, uint8_t address, uint32_t i2c_timeout_ms)
  : i2c_bus_(i2c_bus), address_(address), i2c_timeout_ms_(i2c_timeout_ms) {
}

MaxM10S::~MaxM10S() {
}

esp_err_t MaxM10S::Open() {
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

void MaxM10S::OpenAsync(Callback cb) {
  ESP_LOGI(TAG, "Opening MaxM10S at address 0x%02X", address_);

  i2c_bus_.Enqueue([this](i2c_port_t port) {
    // Enable UBX-NAV-PVT message on I2C
    uint8_t payload[9];
    payload[0] = 0; payload[1] = 0x01; payload[2] = 0; payload[3] = 0;
    uint32_t key = CFG_MSGOUT_UBX_NAV_PVT_I2C;
    memcpy(payload + 4, &key, 4);
    payload[8] = 1;
    esp_err_t err = SendUBXInternal(port, 0x06, 0x8A, payload, 9);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to connect to MaxM10S (check I2C): %s", esp_err_to_name(err));
      return err;
    }

    // Enable UBX-NAV-DOP and UBX-NAV-SAT
    key = CFG_MSGOUT_UBX_NAV_DOP_I2C;
    memcpy(payload + 4, &key, 4);
    SendUBXInternal(port, 0x06, 0x8A, payload, 9);

    key = CFG_MSGOUT_UBX_NAV_SAT_I2C;
    memcpy(payload + 4, &key, 4);
    SendUBXInternal(port, 0x06, 0x8A, payload, 9);

    // Set 5Hz measurement rate (200ms)
    uint16_t rate = 200;
    uint8_t payload_rate[10];
    payload_rate[0] = 0; payload_rate[1] = 0x01; payload_rate[2] = 0; payload_rate[3] = 0;
    key = CFG_RATE_MEAS;
    memcpy(payload_rate + 4, &key, 4);
    memcpy(payload_rate + 8, &rate, 2);
    SendUBXInternal(port, 0x06, 0x8A, payload_rate, 10);

    return ESP_OK;
  }, cb);
}

esp_err_t MaxM10S::Close() {
  return ESP_OK;
}

esp_err_t MaxM10S::Update() {
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

void MaxM10S::UpdateAsync(Callback cb) {
  i2c_bus_.Enqueue([this](i2c_port_t port) {
    // Read available length from registers 0xFD and 0xFE
    uint8_t len_reg = 0xFD;
    uint8_t len_bytes[2] = {0, 0};
    esp_err_t err = i2c_master_write_read_device(port, address_, &len_reg, 1, len_bytes, 2, pdMS_TO_TICKS(i2c_timeout_ms_));
    if (err != ESP_OK) return err;

    uint16_t available = (len_bytes[0] << 8) | len_bytes[1];
    if (available == 0 || available == 0xFFFF) return ESP_OK;

    // Limit to 1024 bytes per update
    if (available > 1024) available = 1024;

    uint8_t data_reg = 0xFF;
    uint8_t chunk[128];
    while (available > 0) {
      uint16_t to_read = (available > 128) ? 128 : available;
      err = i2c_master_write_read_device(port, address_, &data_reg, 1, chunk, to_read, pdMS_TO_TICKS(i2c_timeout_ms_));
      if (err != ESP_OK) break;
      for (uint16_t i = 0; i < to_read; ++i) {
        ProcessByte(chunk[i]);
      }
      available -= to_read;
    }
    return err;
  }, cb);
}

void MaxM10S::ProcessByte(uint8_t byte) {
  switch (parse_state_) {
    case ParseState::SYNC1:
      if (byte == 0xB5) parse_state_ = ParseState::SYNC2;
      break;
    case ParseState::SYNC2:
      if (byte == 0x62) {
        parse_state_ = ParseState::CLASS;
        rx_ck_a_ = 0;
        rx_ck_b_ = 0;
      } else {
        parse_state_ = ParseState::SYNC1;
      }
      break;
    case ParseState::CLASS:
      rx_class_ = byte;
      rx_ck_a_ += byte;
      rx_ck_b_ += rx_ck_a_;
      parse_state_ = ParseState::ID;
      break;
    case ParseState::ID:
      rx_id_ = byte;
      rx_ck_a_ += byte;
      rx_ck_b_ += rx_ck_a_;
      parse_state_ = ParseState::LEN1;
      break;
    case ParseState::LEN1:
      rx_len_ = byte;
      rx_ck_a_ += byte;
      rx_ck_b_ += rx_ck_a_;
      parse_state_ = ParseState::LEN2;
      break;
    case ParseState::LEN2:
      rx_len_ |= (byte << 8);
      rx_ck_a_ += byte;
      rx_ck_b_ += rx_ck_a_;
      if (rx_len_ == 0) {
        parse_state_ = ParseState::CK_A;
      } else if (rx_len_ <= sizeof(rx_buffer_)) {
        rx_idx_ = 0;
        parse_state_ = ParseState::PAYLOAD;
      } else {
        parse_state_ = ParseState::SYNC1;
      }
      break;
    case ParseState::PAYLOAD:
      rx_buffer_[rx_idx_++] = byte;
      rx_ck_a_ += byte;
      rx_ck_b_ += rx_ck_a_;
      if (rx_idx_ == rx_len_) {
        parse_state_ = ParseState::CK_A;
      }
      break;
    case ParseState::CK_A:
      if (byte == rx_ck_a_) {
        parse_state_ = ParseState::CK_B;
      } else {
        parse_state_ = ParseState::SYNC1;
      }
      break;
    case ParseState::CK_B:
      if (byte == rx_ck_b_) {
        HandleMessage(rx_class_, rx_id_, rx_buffer_, rx_len_);
      }
      parse_state_ = ParseState::SYNC1;
      break;
  }
}

template<typename T>
static T readLE(const uint8_t* buf) {
  T val;
  memcpy(&val, buf, sizeof(T));
  return val;
}

void MaxM10S::HandleMessage(uint8_t msgClass, uint8_t msgID, const uint8_t* payload, uint16_t len) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (msgClass == 0x01) { // NAV
    if (msgID == 0x07 && len >= 92) { // PVT
      pvt_data_.iTOW = readLE<uint32_t>(payload + 0);
      pvt_data_.year = readLE<uint16_t>(payload + 4);
      pvt_data_.month = payload[6];
      pvt_data_.day = payload[7];
      pvt_data_.hour = payload[8];
      pvt_data_.minute = payload[9];
      pvt_data_.second = payload[10];
      pvt_data_.valid = payload[11];
      pvt_data_.tAcc = readLE<uint32_t>(payload + 12);
      pvt_data_.nano = readLE<int32_t>(payload + 16);
      pvt_data_.fixType = payload[20];
      pvt_data_.flags = payload[21];
      pvt_data_.flags2 = payload[22];
      pvt_data_.numSV = payload[23];
      pvt_data_.lon = readLE<int32_t>(payload + 24);
      pvt_data_.lat = readLE<int32_t>(payload + 28);
      pvt_data_.height = readLE<int32_t>(payload + 32);
      pvt_data_.hMSL = readLE<int32_t>(payload + 36);
      pvt_data_.hAcc = readLE<uint32_t>(payload + 40);
      pvt_data_.vAcc = readLE<uint32_t>(payload + 44);
      pvt_data_.velN = readLE<int32_t>(payload + 48);
      pvt_data_.velE = readLE<int32_t>(payload + 52);
      pvt_data_.velD = readLE<int32_t>(payload + 56);
      pvt_data_.gSpeed = readLE<int32_t>(payload + 60);
      pvt_data_.headMot = readLE<int32_t>(payload + 64);
      pvt_data_.sAcc = readLE<uint32_t>(payload + 68);
      pvt_data_.headAcc = readLE<uint32_t>(payload + 72);
      pvt_data_.pDOP = readLE<uint16_t>(payload + 76);
      pvt_data_.flags3 = readLE<uint16_t>(payload + 78);
      pvt_data_.headVeh = readLE<int32_t>(payload + 84);
      pvt_data_.magDec = readLE<int16_t>(payload + 88);
      pvt_data_.magAcc = readLE<uint16_t>(payload + 90);
    } else if (msgID == 0x04 && len >= 18) { // DOP
      dop_data_.iTOW = readLE<uint32_t>(payload + 0);
      dop_data_.gDOP = readLE<uint16_t>(payload + 4);
      dop_data_.pDOP = readLE<uint16_t>(payload + 6);
      dop_data_.tDOP = readLE<uint16_t>(payload + 8);
      dop_data_.vDOP = readLE<uint16_t>(payload + 10);
      dop_data_.hDOP = readLE<uint16_t>(payload + 12);
      dop_data_.nDOP = readLE<uint16_t>(payload + 14);
      dop_data_.eDOP = readLE<uint16_t>(payload + 16);
    } else if (msgID == 0x35 && len >= 8) { // SAT
      uint8_t numSvs = payload[5];
      sat_data_.clear();
      for (int i = 0; i < numSvs && (8 + i * 12 + 12 <= len); ++i) {
        const uint8_t* p = payload + 8 + i * 12;
        SatInfo sat;
        sat.gnssId = p[0];
        sat.svId = p[1];
        sat.cno = p[2];
        sat.elev = (int8_t)p[3];
        sat.azim = readLE<int16_t>(p + 4);
        sat.prRes = readLE<int16_t>(p + 6);
        sat.flags = readLE<uint32_t>(p + 8);
        sat_data_.push_back(sat);
      }
    }
  }
}

esp_err_t MaxM10S::SendUBXInternal(i2c_port_t port, uint8_t msgClass, uint8_t msgID, const uint8_t* payload, uint16_t len) {
  uint16_t total_len = len + 8;
  std::vector<uint8_t> frame(total_len);
  frame[0] = 0xB5;
  frame[1] = 0x62;
  frame[2] = msgClass;
  frame[3] = msgID;
  frame[4] = len & 0xFF;
  frame[5] = (len >> 8) & 0xFF;
  if (len > 0 && payload != nullptr) {
    memcpy(frame.data() + 6, payload, len);
  }

  uint8_t ck_a = 0, ck_b = 0;
  for (uint16_t i = 2; i < total_len - 2; ++i) {
    ck_a += frame[i];
    ck_b += ck_a;
  }
  frame[total_len - 2] = ck_a;
  frame[total_len - 1] = ck_b;

  return i2c_master_write_to_device(port, address_, frame.data(), total_len, pdMS_TO_TICKS(i2c_timeout_ms_));
}

esp_err_t MaxM10S::SetMeasurementRate(uint16_t rate_ms) {
  return SetConfig(CFG_RATE_MEAS, rate_ms);
}

esp_err_t MaxM10S::SetNavigationRate(uint16_t cycles) {
  return SetConfig(CFG_RATE_NAV, cycles);
}

esp_err_t MaxM10S::SetDynamicModel(DynamicModel model) {
  return SetConfig(CFG_NAVSPG_DYNMODEL, (uint8_t)model);
}

esp_err_t MaxM10S::SetGNSSSystems(bool gps, bool galileo, bool beidou, bool glonass) {
  esp_err_t err;
  err = SetConfig(CFG_SIGNAL_GPS_ENA, gps);
  if (err != ESP_OK) return err;
  err = SetConfig(CFG_SIGNAL_GAL_ENA, galileo);
  if (err != ESP_OK) return err;
  err = SetConfig(CFG_SIGNAL_BDS_ENA, beidou);
  if (err != ESP_OK) return err;
  err = SetConfig(CFG_SIGNAL_GLO_ENA, glonass);
  return err;
}

esp_err_t MaxM10S::SetOperatingMode(OperatingMode mode) {
  return SetConfig(CFG_PM_OPERATEMODE, (uint8_t)mode);
}

esp_err_t MaxM10S::Standby(uint32_t duration_ms) {
  SemaphoreHandle_t done = xSemaphoreCreateBinary();
  esp_err_t result = ESP_FAIL;
  i2c_bus_.Enqueue([this, duration_ms](i2c_port_t port) {
    uint8_t payload[8];
    memset(payload, 0, 8);
    memcpy(payload, &duration_ms, 4);
    uint32_t flags = 0x02; // Force
    memcpy(payload + 4, &flags, 4);
    return SendUBXInternal(port, 0x02, 0x41, payload, 8);
  }, [&result, done](esp_err_t err) {
    result = err;
    xSemaphoreGive(done);
  });
  xSemaphoreTake(done, portMAX_DELAY);
  vSemaphoreDelete(done);
  return result;
}

esp_err_t MaxM10S::Hibernate(uint32_t duration_ms) {
  SemaphoreHandle_t done = xSemaphoreCreateBinary();
  esp_err_t result = ESP_FAIL;
  i2c_bus_.Enqueue([this, duration_ms](i2c_port_t port) {
    uint8_t payload[8];
    memset(payload, 0, 8);
    memcpy(payload, &duration_ms, 4);
    uint32_t flags = 0x06; // Backup + Force
    memcpy(payload + 4, &flags, 4);
    return SendUBXInternal(port, 0x02, 0x41, payload, 8);
  }, [&result, done](esp_err_t err) {
    result = err;
    xSemaphoreGive(done);
  });
  xSemaphoreTake(done, portMAX_DELAY);
  vSemaphoreDelete(done);
  return result;
}

esp_err_t MaxM10S::Wake() {
  SemaphoreHandle_t done = xSemaphoreCreateBinary();
  esp_err_t result = ESP_FAIL;
  i2c_bus_.Enqueue([this](i2c_port_t port) {
    uint8_t dummy = 0xFF;
    return i2c_master_write_to_device(port, address_, &dummy, 1, pdMS_TO_TICKS(i2c_timeout_ms_));
  }, [&result, done](esp_err_t err) {
    result = err;
    xSemaphoreGive(done);
  });
  xSemaphoreTake(done, portMAX_DELAY);
  vSemaphoreDelete(done);

  if (result == ESP_OK) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  return result;
}

esp_err_t MaxM10S::SetConfig(uint32_t key, uint8_t value) {
  uint8_t payload[9];
  payload[0] = 0; payload[1] = 0x01; payload[2] = 0; payload[3] = 0;
  memcpy(payload + 4, &key, 4);
  payload[8] = value;
  SemaphoreHandle_t done = xSemaphoreCreateBinary();
  esp_err_t result = ESP_FAIL;
  i2c_bus_.Enqueue([this, payload](i2c_port_t port) {
    return SendUBXInternal(port, 0x06, 0x8A, payload, 9);
  }, [&result, done](esp_err_t err) {
    result = err;
    xSemaphoreGive(done);
  });
  xSemaphoreTake(done, portMAX_DELAY);
  vSemaphoreDelete(done);
  return result;
}

esp_err_t MaxM10S::SetConfig(uint32_t key, uint16_t value) {
  uint8_t payload[10];
  payload[0] = 0; payload[1] = 0x01; payload[2] = 0; payload[3] = 0;
  memcpy(payload + 4, &key, 4);
  memcpy(payload + 8, &value, 2);
  SemaphoreHandle_t done = xSemaphoreCreateBinary();
  esp_err_t result = ESP_FAIL;
  i2c_bus_.Enqueue([this, payload](i2c_port_t port) {
    return SendUBXInternal(port, 0x06, 0x8A, payload, 10);
  }, [&result, done](esp_err_t err) {
    result = err;
    xSemaphoreGive(done);
  });
  xSemaphoreTake(done, portMAX_DELAY);
  vSemaphoreDelete(done);
  return result;
}

esp_err_t MaxM10S::SetConfig(uint32_t key, uint32_t value) {
  uint8_t payload[12];
  payload[0] = 0; payload[1] = 0x01; payload[2] = 0; payload[3] = 0;
  memcpy(payload + 4, &key, 4);
  memcpy(payload + 8, &value, 4);
  SemaphoreHandle_t done = xSemaphoreCreateBinary();
  esp_err_t result = ESP_FAIL;
  i2c_bus_.Enqueue([this, payload](i2c_port_t port) {
    return SendUBXInternal(port, 0x06, 0x8A, payload, 12);
  }, [&result, done](esp_err_t err) {
    result = err;
    xSemaphoreGive(done);
  });
  xSemaphoreTake(done, portMAX_DELAY);
  vSemaphoreDelete(done);
  return result;
}

esp_err_t MaxM10S::SetConfig(uint32_t key, bool value) {
  return SetConfig(key, (uint8_t)(value ? 1 : 0));
}

void MaxM10S::ResetParser() {
  parse_state_ = ParseState::SYNC1;
  rx_idx_ = 0;
  rx_len_ = 0;
}

MaxM10S::PVTData MaxM10S::GetPVT() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return pvt_data_;
}

MaxM10S::DOPData MaxM10S::GetDOP() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return dop_data_;
}

std::vector<MaxM10S::SatInfo> MaxM10S::GetSatellites() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return sat_data_;
}

} // namespace ALC
