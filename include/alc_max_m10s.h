#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include "alc_i2c_bus_manager.h"
#include <cstdint>
#include <mutex>
#include <vector>
#include <functional>

namespace ALC {

/**
 * @brief Driver for u-blox MAX-M10S GNSS module using UBX protocol over I2C.
 */
class MaxM10S {
public:
  using Callback = std::function<void(esp_err_t)>;

  /**
   * @brief Navigation Position Velocity Time Solution data.
   */
  struct PVTData {
    uint32_t iTOW = 0;
    uint16_t year = 0;
    uint8_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    uint8_t valid = 0;
    uint32_t tAcc = 0;
    int32_t nano = 0;
    uint8_t fixType = 0;
    uint8_t flags = 0;
    uint8_t flags2 = 0;
    uint8_t numSV = 0;
    int32_t lon = 0;
    int32_t lat = 0;
    int32_t height = 0;
    int32_t hMSL = 0;
    uint32_t hAcc = 0;
    uint32_t vAcc = 0;
    int32_t velN = 0;
    int32_t velE = 0;
    int32_t velD = 0;
    int32_t gSpeed = 0;
    int32_t headMot = 0;
    uint32_t sAcc = 0;
    uint32_t headAcc = 0;
    uint16_t pDOP = 0;
    uint16_t flags3 = 0;
    int32_t headVeh = 0;
    int16_t magDec = 0;
    uint16_t magAcc = 0;
  };

  /**
   * @brief Dilution of precision data.
   */
  struct DOPData {
    uint32_t iTOW = 0;
    uint16_t gDOP = 0;
    uint16_t pDOP = 0;
    uint16_t tDOP = 0;
    uint16_t vDOP = 0;
    uint16_t hDOP = 0;
    uint16_t nDOP = 0;
    uint16_t eDOP = 0;
  };

  /**
   * @brief Single satellite information.
   */
  struct SatInfo {
    uint8_t gnssId = 0;
    uint8_t svId = 0;
    uint8_t cno = 0;
    int8_t elev = 0;
    int16_t azim = 0;
    int16_t prRes = 0;
    uint32_t flags = 0;
  };

  /**
   * @brief Dynamic platform model.
   */
  enum class DynamicModel : uint8_t {
    PORTABLE = 0,
    STATIONARY = 2,
    PEDESTRIAN = 3,
    AUTOMOTIVE = 4,
    SEA = 5,
    AIRBORNE_1G = 6,
    AIRBORNE_2G = 7,
    AIRBORNE_4G = 8,
    WRIST = 9,
    BIKE = 10
  };

  /**
   * @brief Power optimization modes for the M10 engine.
   */
  enum class OperatingMode : uint8_t {
    CONTINUOUS = 0,
    BALANCED = 1
  };

  // Common Configuration Keys (UBX-CFG-VALSET)
  static constexpr uint32_t CFG_RATE_MEAS = 0x30210001;
  static constexpr uint32_t CFG_RATE_NAV = 0x30210002;
  static constexpr uint32_t CFG_NAVSPG_DYNMODEL = 0x20110021;
  static constexpr uint32_t CFG_MSGOUT_UBX_NAV_PVT_I2C = 0x20910006;
  static constexpr uint32_t CFG_MSGOUT_UBX_NAV_DOP_I2C = 0x20910038;
  static constexpr uint32_t CFG_MSGOUT_UBX_NAV_SAT_I2C = 0x20910015;
  static constexpr uint32_t CFG_SIGNAL_GPS_ENA = 0x1031001f;
  static constexpr uint32_t CFG_SIGNAL_GAL_ENA = 0x10310021;
  static constexpr uint32_t CFG_SIGNAL_BDS_ENA = 0x10310022;
  static constexpr uint32_t CFG_SIGNAL_GLO_ENA = 0x10310025;
  static constexpr uint32_t CFG_PM_OPERATEMODE = 0x20110052;

  /**
   * @brief Construct a new MaxM10S object.
   */
  explicit MaxM10S(I2CBusManager& i2c_bus, uint8_t address = 0x42, uint32_t i2c_timeout_ms = 100);
  ~MaxM10S();

  MaxM10S() = delete;
  MaxM10S(const MaxM10S&) = delete;
  MaxM10S& operator=(const MaxM10S&) = delete;

  /**
   * @brief Initialize communication and basic configuration.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t Open();
  void OpenAsync(Callback cb);

  /**
   * @brief Close the driver.
   */
  esp_err_t Close();

  /**
   * @brief Poll for new data and update internal state. Non-blocking.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t Update();
  void UpdateAsync(Callback cb);

  // Getters for latest data
  PVTData GetPVT() const;
  DOPData GetDOP() const;
  std::vector<SatInfo> GetSatellites() const;

  // Configuration Methods
  esp_err_t SetMeasurementRate(uint16_t rate_ms);
  esp_err_t SetNavigationRate(uint16_t cycles);
  esp_err_t SetDynamicModel(DynamicModel model);
  esp_err_t SetGNSSSystems(bool gps, bool galileo, bool beidou, bool glonass);
  esp_err_t SetOperatingMode(OperatingMode mode);

  esp_err_t Standby(uint32_t duration_ms = 0);
  esp_err_t Hibernate(uint32_t duration_ms = 0);
  esp_err_t Wake();

  /**
   * @brief Generic configuration using UBX-CFG-VALSET.
   */
  esp_err_t SetConfig(uint32_t key, uint8_t value);
  esp_err_t SetConfig(uint32_t key, uint16_t value);
  esp_err_t SetConfig(uint32_t key, uint32_t value);
  esp_err_t SetConfig(uint32_t key, bool value);

private:
  I2CBusManager& i2c_bus_;
  uint8_t address_;
  uint32_t i2c_timeout_ms_;
  mutable std::recursive_mutex mutex_;

  PVTData pvt_data_;
  DOPData dop_data_;
  std::vector<SatInfo> sat_data_;

  // Protocol helpers
  esp_err_t SendUBXInternal(i2c_port_t port, uint8_t msgClass, uint8_t msgID, const uint8_t* payload, uint16_t len);
  void ProcessByte(uint8_t byte);
  void HandleMessage(uint8_t msgClass, uint8_t msgID, const uint8_t* payload, uint16_t len);

  // Parsing state
  enum class ParseState {
    SYNC1, SYNC2, CLASS, ID, LEN1, LEN2, PAYLOAD, CK_A, CK_B
  } parse_state_ = ParseState::SYNC1;

  uint8_t rx_class_ = 0;
  uint8_t rx_id_ = 0;
  uint16_t rx_len_ = 0;
  uint16_t rx_idx_ = 0;
  uint8_t rx_buffer_[1024];
  uint8_t rx_ck_a_ = 0;
  uint8_t rx_ck_b_ = 0;

  void ResetParser();
};

} // namespace ALC
