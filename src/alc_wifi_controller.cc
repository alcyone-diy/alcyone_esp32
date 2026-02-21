#include "alc_wifi_controller.h"

#include "alc_esp32_timer.h"

#include "esp_wifi.h"
#include "esp_log.h"

#define TAG "ALC_WifiController"

namespace {

constexpr int kMaxRetry = 5;

}  // namespace

namespace ALC {

WifiController::WifiController() {}

WifiController::~WifiController() {
  esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &wifi_event_handler_instance_);
  esp_event_handler_instance_unregister(IP_EVENT, ESP_EVENT_ANY_ID, &ip_event_handler_instance_);
}

esp_err_t WifiController::Init() {
  retry_timer_ = std::make_unique<ESP32Timer>([this]() { this->HandleRetry(); });
  last_error_ = esp_netif_init();
  if (last_error_ != ESP_OK) {
    ESP_LOGE(TAG, "esp_netif_init() error: %s", esp_err_to_name(last_error_));
    return last_error_;
  }

  esp_netif_create_default_wifi_sta();

  // Configuration of the Wi-Fi driver with default settings
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  last_error_ = esp_wifi_init(&cfg);
  if (last_error_ != ESP_OK) {
    ESP_LOGE(TAG, "esp_wifi_init() error: %s", esp_err_to_name(last_error_));
    return last_error_;
  }

  last_error_ = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                    &WifiController::EventHandler, this,
                                                    &wifi_event_handler_instance_);
  if (last_error_ != ESP_OK) {
    ESP_LOGE(TAG, "esp_event_handler_instance_register(WIFI_EVENT) error: %s",
             esp_err_to_name(last_error_));
    return last_error_;
  }

  last_error_ = esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                                    &WifiController::EventHandler, this,
                                                    &ip_event_handler_instance_);
  if (last_error_ != ESP_OK) {
    ESP_LOGE(TAG, "esp_event_handler_instance_register(IP_EVENT) error: %s",
             esp_err_to_name(last_error_));
    return last_error_;
  }

  last_error_ = esp_wifi_set_mode(WIFI_MODE_STA);
  if (last_error_ != ESP_OK) {
    ESP_LOGE(TAG, "esp_wifi_set_mode() error: %s", esp_err_to_name(last_error_));
    return last_error_;
  }

  last_error_ = esp_wifi_start();
  if (last_error_ != ESP_OK) {
    ESP_LOGE(TAG, "esp_wifi_start() error: %s", esp_err_to_name(last_error_));
    return last_error_;
  }
  return last_error_;
}

void WifiController::AddCredential(const WifiController::Credential& credential) {
  auto it = std::find_if(credential_list_.begin(), credential_list_.end(),
                         [&](const Credential& n) { return n.ssid == credential.ssid; });

  if (it == credential_list_.end()) {
    credential_list_.push_back(credential);
  } else {
    *it = credential;
  }
}

bool WifiController::RemoveCredential(const char* ssid) {
  auto it = std::find_if(credential_list_.begin(), credential_list_.end(),
                         [&](const Credential& n) { return n.ssid == ssid; });

  if (it == credential_list_.end()) {
    return false;
  }

  size_t removed_index = std::distance(credential_list_.begin(), it);

  credential_list_.erase(it);

  // Adjustment of the current index
  if (credential_index_ > removed_index) {
    credential_index_--;
  } else if (credential_index_ == removed_index) {
    credential_index_ = 0;
  }

  return true;
}

esp_err_t WifiController::Connect() {
  last_error_ = SetDefaultWifi();
  if (last_error_ != ESP_OK) {
    return last_error_;
  }
  last_error_ = esp_wifi_connect();
  if (last_error_ != ESP_OK) {
    return last_error_;
  }
  UpdateState(State::kConnecting, LastConnection::kUnknownError);
  return last_error_;
}

esp_err_t WifiController::SetDefaultWifi() {
  if (credential_list_.empty()) {
    return ESP_ERR_INVALID_ARG;
  }
  if (credential_list_.size() <= credential_index_) {
    credential_index_ = 0;
  }
  Credential credential = credential_list_[credential_index_];

  ESP_LOGI(TAG, "Connect: %s", credential.ssid.c_str());
  wifi_config_t wifi_config = {}; // Initialize everything to zero

  strlcpy(reinterpret_cast<char*>(wifi_config.sta.ssid), credential.ssid.c_str(),
          sizeof(wifi_config.sta.ssid));
  strlcpy(reinterpret_cast<char*>(wifi_config.sta.password), credential.password.c_str(),
          sizeof(wifi_config.sta.password));

  wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
  wifi_config.sta.pmf_cfg.capable = true;
  wifi_config.sta.pmf_cfg.required = false;

  last_error_ = esp_wifi_set_mode(WIFI_MODE_STA);
  if (last_error_ != ESP_OK) {
    ESP_LOGW(TAG, "esp_wifi_set_mode error\n");
    return last_error_;
  }

  last_error_ = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  if (last_error_ != ESP_OK) {
    ESP_LOGW(TAG, "esp_wifi_set_config error\n");
  }
  return last_error_;
}

void WifiController::UpdateState(WifiController::State state,
                                 WifiController::LastConnection last_connection) {
  State previous_state = state_;
  state_ = state;
  last_connection_ = last_connection;
  StateCallback state_callback = nullptr;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    state_callback = state_callback_;
  }
  if (state_callback) {
    state_callback(state, previous_state);
  }
  switch (state_) {
    case State::kIdle:
      retry_num_ = 0;
      retry_timer_->Stop();
      break;
    case State::kConnecting:
      retry_num_ = 0;
      retry_timer_->Stop();
      break;
    case State::kConnected:
      retry_num_ = 0;
      retry_timer_->Stop();
      break;
    case State::kGotIp:
      retry_num_ = 0;
      retry_timer_->Stop();
      break;
    case State::kReconnecting:
      break;
  }
}

void WifiController::OnStaStartEvent() {
  UpdateState(State::kIdle, LastConnection::kUnknownError);
}

void WifiController::OnStaConnectedEvent(wifi_event_sta_connected_t* event_data) {
  UpdateState(State::kConnected, LastConnection::kUnknownError);
}

void WifiController::OnStaGotIp(ip_event_got_ip_t* event) {
  UpdateState(State::kGotIp, LastConnection::kUnknownError);
}

void WifiController::OnStaDisconnectEvent(wifi_event_sta_disconnected_t* event_data) {
  wifi_err_reason_t reason = static_cast<wifi_err_reason_t>(event_data->reason);
  ESP_LOGW(TAG, "Disconnected. Reason: %d", reason);

  State new_state = (state_ == State::kGotIp || state_ == State::kReconnecting)
                        ? State::kReconnecting
                        : State::kIdle;
  if (retry_num_ < kMaxRetry) {
    new_state = State::kIdle;
  }
  LastConnection error = LastConnection::kUnknownError;

  if (reason == WIFI_REASON_AUTH_FAIL) {
    error = LastConnection::kErrorAuth;
    ESP_LOGE(TAG, "Invalid password. Giving up.");
  } else if (reason == WIFI_REASON_NO_AP_FOUND) {
    error = LastConnection::kWifiNotFound;
  }

  if (new_state == State::kReconnecting) {
    retry_num_++;
    ESP_LOGW(TAG, "Reconnect attempt %d", retry_num_);
    uint32_t retry_delay_us = (retry_num_ <= 2) ? 2000000 : 5000000;

    if (reason > 100) {
      ESP_LOGD(TAG, "Internal error detected, light radio reset.");
    }

    assert(retry_timer_.get());
    retry_timer_->StartOnce(retry_delay_us);
  }
  this->UpdateState(new_state, error);
}

void WifiController::OnScanDoneEvent(wifi_event_sta_scan_done_t* event_data) {
  uint16_t ap_count = 0;
  esp_wifi_scan_get_ap_num(&ap_count);

  if (ap_count == 0) {
    return;
  }

  std::vector<wifi_ap_record_t> ap_records(ap_count);

  if (esp_wifi_scan_get_ap_records(&ap_count, ap_records.data()) == ESP_OK) {
    for (const auto& ap : ap_records) {
      if (scan_callback_) {
        scan_callback_(reinterpret_cast<const char*>(ap.ssid), ap.rssi);
      }
    }
  }
  if (scan_callback_) {
    scan_callback_(nullptr, 0);
  }
}

void WifiController::HandleRetry() {
  ESP_LOGI(TAG, "Timer expired, attempting reconnection...");
  esp_wifi_connect();
}

// static
void WifiController::EventHandler(void* arg, esp_event_base_t event_base, int32_t event_id,
                                  void* event_data) {
  ESP_LOGW(TAG, "EventHandler %s", event_base);
  WifiController* wifi_controller = static_cast<WifiController*>(arg);
  if (!wifi_controller) {
    return;
  }
  if (event_base == WIFI_EVENT) {
    wifi_event_t wifi_event = static_cast<wifi_event_t>(event_id);
    switch (wifi_event) {
      case WIFI_EVENT_WIFI_READY:
        ESP_LOGV(TAG, "WIFI_EVENT_WIFI_READY");
        break;
      case WIFI_EVENT_STA_START:
        ESP_LOGV(TAG, "WIFI_EVENT_STA_START");
        wifi_controller->OnStaStartEvent();
        break;
      case WIFI_EVENT_STA_STOP:
        ESP_LOGV(TAG, "WIFI_EVENT_STA_STOP");
        break;
      case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGV(TAG, "WIFI_EVENT_STA_CONNECTED");
        wifi_controller->OnStaConnectedEvent(static_cast<wifi_event_sta_connected_t*>(event_data));
        break;
      case WIFI_EVENT_STA_DISCONNECTED:
        ESP_LOGV(TAG, "WIFI_EVENT_STA_DISCONNECTED");
        wifi_controller->OnStaDisconnectEvent(
            static_cast<wifi_event_sta_disconnected_t*>(event_data));
        break;
      case WIFI_EVENT_SCAN_DONE:
        ESP_LOGV(TAG, "WIFI_EVENT_SCAN_DONE");
        wifi_controller->OnScanDoneEvent(static_cast<wifi_event_sta_scan_done_t*>(event_data));
        break;
      case WIFI_EVENT_HOME_CHANNEL_CHANGE:
        ESP_LOGV(TAG, "WIFI_EVENT_HOME_CHANNEL_CHANGE");
        break;
      default:
        break;
    }
  } else if (event_base == IP_EVENT) {
    switch (event_id) {
      case IP_EVENT_STA_GOT_IP:
        wifi_controller->OnStaGotIp(static_cast<ip_event_got_ip_t*>(event_data));
        break;
    }
  }
}

}  // namespace ALC
