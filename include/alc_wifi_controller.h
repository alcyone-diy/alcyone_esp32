#pragma once

#include <vector>
#include <memory>

#include "esp_err.h"
#include "esp_wifi.h"

#include <functional>
#include <mutex>

namespace ALC {

class ESP32Timer;

class WifiController {
public:
  enum class State {
    kIdle,
    kConnecting,
    kConnected,
    kGotIp,
    kReconnecting,
  };
  enum class LastConnection {
    kNoError,
    kUnknownError,
    kErrorAuth,
    kWifiNotFound,
  };
  struct Credential {
    std::string ssid;
    std::string password;
  };

  using StateCallback = std::function<void(State current_state, State previous_state)>;
  using ScanCallback = std::function<void(const char *ssid, int8_t rssi)>;

  WifiController();
  ~WifiController();
  esp_err_t Init();

  esp_err_t Scan();

  void AddCredential(const Credential &credential);
  void SetNextCredential() { if (++credential_index_ >= credential_list_.size()) { credential_index_ = 0; } };
  bool RemoveCredential(const char *ssid);
  void SetAutoReconnect(bool reconnect) { auto_reconnect_ = reconnect; };
  esp_err_t Connect();
  esp_err_t Disconnect();
  void SetStateCallback(StateCallback cb) { state_callback_ = cb; }
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

  static void EventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
  
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
