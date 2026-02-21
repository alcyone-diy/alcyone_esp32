#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include "alc_i2c_bus_manager.h"
#include <cstdint>
#include <mutex>
#include <vector>

namespace ALC {

/**
 * @brief Driver for u-blox MAX-M10S GNSS module using UBX protocol over I2C.
 */
class MaxM10sSensor {
public:
  using Callback = I2CBusManager::Callback;
  using BusToken = I2CBusManager::BusToken;

  /**
   * @brief Navigation Position Velocity Time Solution data.
   */
  struct PVTData {
    uint32_t iTOW = 0;      ///< GPS time of week [ms]. The standard time reference for GNSS.
    uint16_t year = 0;      ///< Year (UTC)
    uint8_t month = 0;      ///< Month, range 1..12 (UTC)
    uint8_t day = 0;        ///< Day of month, range 1..31 (UTC)
    uint8_t hour = 0;       ///< Hour, range 0..23 (UTC)
    uint8_t minute = 0;     ///< Minute, range 0..59 (UTC)
    uint8_t second = 0;     ///< Seconds, range 0..60 (UTC)
    uint8_t valid = 0;      ///< Validity flags (validDate, validTime, fullyResolved, validMag)
    uint32_t tAcc = 0;      ///< Time accuracy estimate [ns] (UTC)
    int32_t nano = 0;       ///< Fraction of second, range -1e9 .. 1e9 [ns] (UTC)
    uint8_t fixType = 0;    ///< GNSSfix Type: 0: no fix, 1: 2D-fix, 3: 3D-fix, etc.
    uint8_t flags = 0;      ///< Fix status flags
    uint8_t flags2 = 0;     ///< Additional flags
    uint8_t numSV = 0;      ///< Number of satellites used in Nav Solution
    int32_t lon = 0;        ///< Longitude [1e-7 deg]
    int32_t lat = 0;        ///< Latitude [1e-7 deg]
    int32_t height = 0;     ///< Height above ellipsoid [mm]
    int32_t hMSL = 0;       ///< Height above mean sea level [mm]
    uint32_t hAcc = 0;      ///< Horizontal accuracy estimate [mm]
    uint32_t vAcc = 0;      ///< Vertical accuracy estimate [mm]
    int32_t velN = 0;       ///< NED north velocity [mm/s]
    int32_t velE = 0;       ///< NED east velocity [mm/s]
    int32_t velD = 0;       ///< NED down velocity [mm/s]
    int32_t gSpeed = 0;     ///< Ground Speed (2D) [mm/s]
    int32_t headMot = 0;    ///< Heading of motion (2D) [1e-5 deg]
    uint32_t sAcc = 0;      ///< Speed accuracy estimate [mm/s]
    uint32_t headAcc = 0;   ///< Heading accuracy estimate [1e-5 deg]
    uint16_t pDOP = 0;      ///< Position DOP [0.01]. See DOPData for explanation.
    uint16_t flags3 = 0;    ///< Additional flags
    int32_t headVeh = 0;    ///< Heading of vehicle (2D) [1e-5 deg]
    int16_t magDec = 0;     ///< Magnetic declination [1e-2 deg]
    uint16_t magAcc = 0;    ///< Magnetic declination accuracy [1e-2 deg]
  };

  /**
   * @brief Dilution of precision data.
   *
   * DOP values describe the geometric strength of the satellite constellation at the
   * time of measurement.
   *
   * Relevance:
   * - Lower values (closer to 1.0) indicate high accuracy due to satellites being
   *   widely spread across the sky.
   * - Higher values (> 5.0) indicate poor satellite geometry (e.g., in urban canyons
   *   where only a narrow strip of sky is visible), leading to lower position accuracy
   *   even if the signal strength is good.
   */
  struct DOPData {
    uint32_t iTOW = 0;      ///< GPS time of week [ms]
    uint16_t gDOP = 0;      ///< Geometric DOP [0.01]. Combines 3D position and time.
    uint16_t pDOP = 0;      ///< Position DOP [0.01]. Combines 3D position accuracy.
    uint16_t tDOP = 0;      ///< Time DOP [0.01]. Time accuracy.
    uint16_t vDOP = 0;      ///< Vertical DOP [0.01]. Altitude accuracy.
    uint16_t hDOP = 0;      ///< Horizontal DOP [0.01]. Latitude/Longitude accuracy.
    uint16_t nDOP = 0;      ///< Northing DOP [0.01].
    uint16_t eDOP = 0;      ///< Easting DOP [0.01].
  };

  /**
   * @brief Single satellite information.
   */
  struct SatInfo {
    /**
     * @brief GNSS identifier.
     * 0: GPS, 1: SBAS, 2: Galileo, 3: BeiDou, 4: IMES, 5: QZSS, 6: GLONASS.
     */
    uint8_t gnssId = 0;

    /**
     * @brief Satellite identifier.
     * The unique ID of the satellite within its GNSS system (e.g. GPS PRN 1-32).
     */
    uint8_t svId = 0;

    uint8_t cno = 0;        ///< Carrier to noise ratio (signal strength) [dBHz]
    int8_t elev = 0;       ///< Elevation [deg]
    int16_t azim = 0;      ///< Azimuth [deg]
    int16_t prRes = 0;     ///< Pseudo range residual [0.1m]
    uint32_t flags = 0;     ///< Bitmask
  };

  /**
   * @brief Dynamic platform model.
   *
   * These models optimize the navigation engine for specific use cases by adjusting
   * filtering parameters (Kalman filter) based on expected motion dynamics.
   */
  enum class DynamicModel : uint8_t {
    PORTABLE = 0,    ///< Balanced performance for most handheld devices.
    STATIONARY = 2,  ///< Optimized for zero velocity; eliminates "drift" while standing still.
    PEDESTRIAN = 3,  ///< For slow motion (walking, up to 30km/h).
    AUTOMOTIVE = 4,  ///< For road vehicles with high acceleration/deceleration.
    SEA = 5,         ///< For maritime applications (assumes near-zero vertical motion).
    AIRBORNE_1G = 6, ///< High-dynamic scenarios (up to 1g, 500m/s).
    AIRBORNE_2G = 7, ///< Up to 2g.
    AIRBORNE_4G = 8, ///< Up to 4g.
    WRIST = 9,       ///< For wrist-worn devices.
    BIKE = 10        ///< For bicycles.
  };

  /**
   * @brief Power optimization modes for the M10 engine.
   */
  enum class OperatingMode : uint8_t {
    /**
     * @brief Full performance mode.
     * Pros: Fastest acquisition, highest accuracy in difficult conditions.
     * Cons: Maximum power consumption (approx. 15-20mW).
     */
    CONTINUOUS = 0,

    /**
     * @brief Power optimized mode.
     * Pros: Reduces power consumption (approx. 10-12mW) by duty-cycling parts of the chip.
     * Cons: Slightly slower acquisition or reduced accuracy in marginal signals.
     */
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
   * @brief Construct a new MaxM10sSensor object.
   *
   * @param bus_manager Reference to the I2C bus manager.
   * @param address I2C address of the sensor (default: 0x42).
   */
  explicit MaxM10sSensor(I2CBusManager& bus_manager, uint8_t address = 0x42);
  ~MaxM10sSensor();

  MaxM10sSensor() = delete;
  MaxM10sSensor(const MaxM10sSensor&) = delete;
  MaxM10sSensor& operator=(const MaxM10sSensor&) = delete;
  MaxM10sSensor(MaxM10sSensor&&) = delete;
  MaxM10sSensor& operator=(MaxM10sSensor&&) = delete;

  /**
   * @brief Initialize communication and basic configuration.
   *
   * @param cb Optional callback called when initialization is complete.
   */
  void Init(Callback cb = nullptr);

  /**
   * @brief Close the driver.
   */
  void Close();

  /**
   * @brief Poll for new data and update internal state. Non-blocking.
   *
   * @param cb Optional callback called when update is complete.
   */
  void Update(Callback cb = nullptr);

  // Getters for latest data
  PVTData GetPVT() const;
  DOPData GetDOP() const;
  std::vector<SatInfo> GetSatellites() const;

  // Configuration Methods
  void SetMeasurementRate(uint16_t rate_ms, Callback cb = nullptr);
  void SetNavigationRate(uint16_t cycles, Callback cb = nullptr);
  void SetDynamicModel(DynamicModel model, Callback cb = nullptr);
  void SetGNSSSystems(bool gps, bool galileo, bool beidou, bool glonass, Callback cb = nullptr);

  /**
   * @brief Set the operating mode (Continuous vs Balanced).
   */
  void SetOperatingMode(OperatingMode mode, Callback cb = nullptr);

  /**
   * @brief Put the device into software standby (Inactive mode).
   *
   * @param duration_ms Time to stay asleep. 0 means stay asleep until Wake() is called.
   * @param cb Optional callback.
   */
  void Standby(uint32_t duration_ms = 0, Callback cb = nullptr);

  /**
   * @brief Put the device into hardware backup (Deep sleep).
   *
   * @param duration_ms Time to stay asleep.
   * @param cb Optional callback.
   */
  void Hibernate(uint32_t duration_ms = 0, Callback cb = nullptr);

  /**
   * @brief Wake the device from Standby or Hibernate.
   *
   * @param cb Optional callback.
   */
  void Wake(Callback cb = nullptr);

  /**
   * @brief Generic configuration using UBX-CFG-VALSET.
   */
  void SetConfig(uint32_t key, uint8_t value, Callback cb = nullptr);
  void SetConfig(uint32_t key, uint16_t value, Callback cb = nullptr);
  void SetConfig(uint32_t key, uint32_t value, Callback cb = nullptr);
  void SetConfig(uint32_t key, bool value, Callback cb = nullptr);

private:
  esp_err_t SendUBX(BusToken& token, uint8_t msgClass, uint8_t msgID, const uint8_t* payload, uint16_t len);
  esp_err_t UpdateInternal(BusToken& token);
  esp_err_t SetConfigInternal(BusToken& token, uint32_t key, uint8_t value);
  esp_err_t SetConfigInternal(BusToken& token, uint32_t key, uint16_t value);
  esp_err_t SetConfigInternal(BusToken& token, uint32_t key, uint32_t value);

  void ProcessByte(uint8_t byte);
  void HandleMessage(uint8_t msgClass, uint8_t msgID, const uint8_t* payload, uint16_t len);

  I2CBusManager& bus_manager_;
  uint8_t address_;
  mutable std::recursive_mutex mutex_;

  PVTData pvt_data_;
  DOPData dop_data_;
  std::vector<SatInfo> sat_data_;

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
