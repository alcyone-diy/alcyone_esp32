#include "alc_max_m10s_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

static const char* TAG = "ALC_MaxM10sDriver";

namespace ALC {

MaxM10sDriver::MaxM10sDriver(I2CBusManager& bus_manager, uint16_t address)
  : bus_manager_(bus_manager), address_(address) {
}

MaxM10sDriver::~MaxM10sDriver() {
}

void MaxM10sDriver::Init(Callback cb) {
  bus_manager_.Enqueue([this](BusToken& token) -> esp_err_t {
    ESP_LOGI(TAG, "Initializing MaxM10sDriver at address 0x%02X", address_);

    // Initial dummy read to clear anything in the buffer
    UpdateInternal(token);

    // Enable UBX-NAV-PVT message on I2C
    esp_err_t err = SetConfigInternal(token, CFG_MSGOUT_UBX_NAV_PVT_I2C, (uint8_t)1);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to connect to MaxM10sDriver: %s", esp_err_to_name(err));
      return err;
    }

    // Enable UBX-NAV-DOP and UBX-NAV-SAT
    SetConfigInternal(token, CFG_MSGOUT_UBX_NAV_DOP_I2C, (uint8_t)1);
    SetConfigInternal(token, CFG_MSGOUT_UBX_NAV_SAT_I2C, (uint8_t)1);

    // Set 5Hz measurement rate (200ms)
    return SetConfigInternal(token, CFG_RATE_MEAS, (uint16_t)200);
  }, cb);
}

void MaxM10sDriver::Close() {
}

void MaxM10sDriver::Update(Callback cb) {
  bus_manager_.Enqueue([this](BusToken& token) {
    return UpdateInternal(token);
  }, cb);
}

esp_err_t MaxM10sDriver::UpdateInternal(BusToken& token) {
  // Read available length from registers 0xFD and 0xFE
  uint8_t len_reg = 0xFD;
  uint8_t len_bytes[2] = {0, 0};

  esp_err_t err = bus_manager_.WriteRead(token, address_, &len_reg, 1, len_bytes, 2);
  if (err != ESP_OK) return err;

  uint16_t available = (len_bytes[0] << 8) | len_bytes[1];
  if (available == 0 || available == 0xFFFF) return ESP_OK;

  // Limit to 1024 bytes per update to avoid blocking or excessive memory usage
  if (available > 1024) available = 1024;

  uint8_t data_reg = 0xFF;
  uint8_t chunk[128];

  while (available > 0) {
    uint16_t to_read = (available > 128) ? 128 : available;

    err = bus_manager_.WriteRead(token, address_, &data_reg, 1, chunk, to_read);
    if (err != ESP_OK) break;

    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      for (uint16_t i = 0; i < to_read; ++i) {
        ProcessByte(chunk[i]);
      }
    }
    available -= to_read;
  }

  return err;
}

void MaxM10sDriver::ProcessByte(uint8_t byte) {
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

void MaxM10sDriver::HandleMessage(uint8_t msgClass, uint8_t msgID, const uint8_t* payload,
                                  uint16_t len) {
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

esp_err_t MaxM10sDriver::SendUBX(BusToken& token, uint8_t msgClass, uint8_t msgID,
                                  const uint8_t* payload, uint16_t len) {
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

  return bus_manager_.Write(token, address_, frame.data(), total_len);
}

void MaxM10sDriver::SetMeasurementRate(uint16_t rate_ms, Callback cb) {
  SetConfig(CFG_RATE_MEAS, rate_ms, cb);
}

void MaxM10sDriver::SetNavigationRate(uint16_t cycles, Callback cb) {
  SetConfig(CFG_RATE_NAV, cycles, cb);
}

void MaxM10sDriver::SetDynamicModel(DynamicModel model, Callback cb) {
  SetConfig(CFG_NAVSPG_DYNMODEL, (uint8_t)model, cb);
}

void MaxM10sDriver::SetGNSSSystems(bool gps, bool galileo, bool beidou, bool glonass, Callback cb) {
  bus_manager_.Enqueue([this, gps, galileo, beidou, glonass](BusToken& token) -> esp_err_t {
    esp_err_t err = SetConfigInternal(token, CFG_SIGNAL_GPS_ENA, gps);
    if (err != ESP_OK) return err;
    err = SetConfigInternal(token, CFG_SIGNAL_GAL_ENA, galileo);
    if (err != ESP_OK) return err;
    err = SetConfigInternal(token, CFG_SIGNAL_BDS_ENA, beidou);
    if (err != ESP_OK) return err;
    return SetConfigInternal(token, CFG_SIGNAL_GLO_ENA, glonass);
  }, cb);
}

void MaxM10sDriver::SetOperatingMode(OperatingMode mode, Callback cb) {
  SetConfig(CFG_PM_OPERATEMODE, (uint8_t)mode, cb);
}

void MaxM10sDriver::Standby(uint32_t duration_ms, Callback cb) {
  uint8_t payload[8];
  memset(payload, 0, 8);
  memcpy(payload, &duration_ms, 4);
  uint32_t flags = 0x02; // Force
  memcpy(payload + 4, &flags, 4);

  std::vector<uint8_t> p(payload, payload + 8);
  bus_manager_.Enqueue([this, p](BusToken& token) {
    return SendUBX(token, 0x02, 0x41, p.data(), 8);
  }, cb);
}

void MaxM10sDriver::Hibernate(uint32_t duration_ms, Callback cb) {
  uint8_t payload[8];
  memset(payload, 0, 8);
  memcpy(payload, &duration_ms, 4);
  uint32_t flags = 0x06; // Backup + Force
  memcpy(payload + 4, &flags, 4);

  std::vector<uint8_t> p(payload, payload + 8);
  bus_manager_.Enqueue([this, p](BusToken& token) {
    return SendUBX(token, 0x02, 0x41, p.data(), 8);
  }, cb);
}

void MaxM10sDriver::Wake(Callback cb) {
  bus_manager_.Enqueue([this](BusToken& token) {
    // Sending a dummy byte to the I2C address is usually enough to wake the device
    uint8_t dummy = 0xFF;
    return bus_manager_.Write(token, address_, &dummy, 1);
  }, [this, cb](esp_err_t err) {
    if (err != ESP_OK) {
      if (cb) cb(err);
      return;
    }
    // Wait a bit for the device to wake up
    bus_manager_.Enqueue([](BusToken&) { return ESP_OK; }, cb, pdMS_TO_TICKS(50));
  });
}

void MaxM10sDriver::SetConfig(uint32_t key, uint8_t value, Callback cb) {
  bus_manager_.Enqueue([this, key, value](BusToken& token) {
    return SetConfigInternal(token, key, value);
  }, cb);
}

void MaxM10sDriver::SetConfig(uint32_t key, uint16_t value, Callback cb) {
  bus_manager_.Enqueue([this, key, value](BusToken& token) {
    return SetConfigInternal(token, key, value);
  }, cb);
}

void MaxM10sDriver::SetConfig(uint32_t key, uint32_t value, Callback cb) {
  bus_manager_.Enqueue([this, key, value](BusToken& token) {
    return SetConfigInternal(token, key, value);
  }, cb);
}

void MaxM10sDriver::SetConfig(uint32_t key, bool value, Callback cb) {
  SetConfig(key, (uint8_t)(value ? 1 : 0), cb);
}

esp_err_t MaxM10sDriver::SetConfigInternal(BusToken& token, uint32_t key, uint8_t value) {
  uint8_t payload[9];
  payload[0] = 0;    // Version 0
  payload[1] = 0x01; // Layers: 1 = RAM
  payload[2] = 0;
  payload[3] = 0;
  memcpy(payload + 4, &key, 4);
  payload[8] = value;
  return SendUBX(token, 0x06, 0x8A, payload, 9);
}

esp_err_t MaxM10sDriver::SetConfigInternal(BusToken& token, uint32_t key, uint16_t value) {
  uint8_t payload[10];
  payload[0] = 0;
  payload[1] = 0x01;
  payload[2] = 0;
  payload[3] = 0;
  memcpy(payload + 4, &key, 4);
  memcpy(payload + 8, &value, 2);
  return SendUBX(token, 0x06, 0x8A, payload, 10);
}

esp_err_t MaxM10sDriver::SetConfigInternal(BusToken& token, uint32_t key, uint32_t value) {
  uint8_t payload[12];
  payload[0] = 0;
  payload[1] = 0x01;
  payload[2] = 0;
  payload[3] = 0;
  memcpy(payload + 4, &key, 4);
  memcpy(payload + 8, &value, 4);
  return SendUBX(token, 0x06, 0x8A, payload, 12);
}

esp_err_t MaxM10sDriver::SetConfigInternal(BusToken& token, uint32_t key, bool value) {
  return SetConfigInternal(token, key, (uint8_t)(value ? 1 : 0));
}

void MaxM10sDriver::ResetParser() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  parse_state_ = ParseState::SYNC1;
  rx_idx_ = 0;
  rx_len_ = 0;
}

MaxM10sDriver::PVTData MaxM10sDriver::GetPVT() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return pvt_data_;
}

MaxM10sDriver::DOPData MaxM10sDriver::GetDOP() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return dop_data_;
}

std::vector<MaxM10sDriver::SatInfo> MaxM10sDriver::GetSatellites() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return sat_data_;
}

} // namespace ALC
