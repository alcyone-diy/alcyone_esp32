#pragma once

#include <vector>
#include <memory>

#include "esp_err.h"
#include "esp_wifi.h"

#include <functional>
#include <mutex>

namespace ALC {

class ESP32Timer;

/**
 * @brief Manages WiFi connections, including scanning, connecting, and handling retries.
 */
class WifiController {
public:
  /**
   * @brief Current state of the WiFi controller.
   */
  enum class State {
    kIdle,         ///< WiFi is not connected and not trying to connect.
    kConnecting,   ///< Attempting to connect to an access point.
    kConnected,    ///< Connected to an access point, but may not have an IP yet.
    kGotIp,        ///< Successfully connected and received an IP address.
    kReconnecting, ///< Connection lost, attempting to reconnect.
  };

  /**
   * @brief Reason for the last connection attempt result.
   */
  enum class LastConnection {
    kNoError,      ///< Last operation was successful.
    kUnknownError, ///< An unspecified error occurred.
    kErrorAuth,    ///< Authentication failed (e.g., wrong password).
    kWifiNotFound, ///< The specified SSID was not found.
  };

  /**
   * @brief WiFi credentials (SSID and password).
   */
  struct Credential {
    std::string ssid;
    std::string password;
  };

  using StateCallback = std::function<void(State current_state, State previous_state)>;
  using ScanCallback = std::function<void(const char* ssid, int8_t rssi)>;

  /**
   * @brief Construct a new WifiController object.
   */
  WifiController();

  /**
   * @brief Destroy the WifiController object.
   */
  ~WifiController();

  // Delete copy/move constructors and assignment operators
  WifiController(const WifiController&) = delete;
  WifiController& operator=(const WifiController&) = delete;
  WifiController(WifiController&&) = delete;
  WifiController& operator=(WifiController&&) = delete;

  /**
   * @brief Initialize the WiFi controller.
   *
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t Init();

  /**
   * @brief Start an asynchronous scan for available access points.
   *
   * @return esp_err_t ESP_OK if the scan started successfully.
   */
  esp_err_t Scan();

  /**
   * @brief Add a credential to the list of known networks.
   *
   * @param credential The credential to add.
   */
  void AddCredential(const Credential& credential);

  /**
   * @brief Select the next credential in the list for the next connection attempt.
   */
  void SetNextCredential() {
    if (++credential_index_ >= credential_list_.size()) {
      credential_index_ = 0;
    }
  }

  /**
   * @brief Remove a credential from the list of known networks.
   *
   * @param ssid The SSID of the network to remove.
   * @return true if the credential was removed, false if it was not found.
   */
  bool RemoveCredential(const char* ssid);

  /**
   * @brief Enable or disable automatic reconnection on disconnection.
   *
   * @param reconnect True to enable auto-reconnect, false to disable.
   */
  void SetAutoReconnect(bool reconnect) { auto_reconnect_ = reconnect; }

  /**
   * @brief Connect to the WiFi using the current credentials.
   *
   * @return esp_err_t ESP_OK if the connection attempt started.
   */
  esp_err_t Connect();

  /**
   * @brief Disconnect from the current WiFi network.
   *
   * @return esp_err_t ESP_OK if disconnection started.
   */
  esp_err_t Disconnect();

  /**
   * @brief Set a callback to be notified of WiFi state changes.
   *
   * @param cb The callback function.
   */
  void SetStateCallback(StateCallback cb) { state_callback_ = cb; }

  /**
   * @brief Set a callback to be notified of scan results.
   *
   * @param cb The callback function.
   */
  void SetScanCallback(ScanCallback cb) { scan_callback_ = cb; }

private:
  esp_err_t SetDefaultWifi();
  void UpdateState(State state, LastConnection last_connection);

  void OnStaStartEvent();
  void OnStaConnectedEvent(wifi_event_sta_connected_t* event_data);
  void OnStaGotIp(ip_event_got_ip_t *event_data);
  void OnStaDisconnectEvent(wifi_event_sta_disconnected_t *event_data);
  void OnScanDoneEvent(wifi_event_sta_scan_done_t *event_data);
  
  void HandleRetry();

  static void EventHandler(void* arg, esp_event_base_t event_base, int32_t event_id,
                           void* event_data);
  
  esp_err_t last_error_ = ESP_OK;
  std::vector<Credential> credential_list_;
  size_t credential_index_ = 0;
  State state_ = State::kIdle;
  LastConnection last_connection_ = LastConnection::kNoError;
  std::unique_ptr<ESP32Timer> retry_timer_;  
  int retry_num_ = 0;
  StateCallback state_callback_ = nullptr;
  ScanCallback scan_callback_ = nullptr;
  bool auto_reconnect_ = true;
  std::mutex data_mutex_;
  esp_event_handler_instance_t wifi_event_handler_instance_;
  esp_event_handler_instance_t ip_event_handler_instance_;
};

}  // namespace ALC
