#include "alc_esp32_timer.h"

#include "esp_log.h"

#include <map>

namespace {

constexpr char kTag[] = "ALC_ESP32Timer";

std::mutex state_map_mutex;
std::map<ALC::ESP32Timer*, std::weak_ptr<ALC::ESP32Timer::State>> timer_states;

}  // namespace

namespace ALC {

ESP32Timer::ESP32Timer(TimerCallback callback) {
  assert(callback);
  state_ = std::make_shared<State>();
  {
    std::lock_guard<std::mutex> main_lock(state_map_mutex);
    timer_states[this] = state_;
  }
  state_->callback = std::move(callback);

  esp_timer_create_args_t args = {};
  args.callback = &ESP32Timer::StaticCallback;
  args.arg = static_cast<void*>(this);
  args.name = nullptr;
  args.dispatch_method = ESP_TIMER_TASK;

  esp_err_t err = esp_timer_create(&args, &state_->handle);
  ESP_ERROR_CHECK(err);
}

ESP32Timer::~ESP32Timer() {
  {
    std::lock_guard<std::mutex> main_lock(state_map_mutex);
    timer_states.erase(this);
  }
  if (state_->handle) {
    esp_timer_stop(state_->handle);
    esp_timer_delete(state_->handle);
  }
}

void ESP32Timer::StartOnce(uint64_t timeout_us) {
  std::lock_guard<std::recursive_mutex> lock(state_->mutex);
  assert(state_->handle);
  Stop();
  state_->is_periodic = false;
  esp_err_t err = esp_timer_start_once(state_->handle, timeout_us);
  ESP_ERROR_CHECK(err);
  state_->is_running = true;
}

void ESP32Timer::StartPeriodic(uint64_t period_us) {
  std::lock_guard<std::recursive_mutex> lock(state_->mutex);
  assert(state_->handle);
  Stop();
  state_->is_periodic = true;
  esp_err_t err = esp_timer_start_periodic(state_->handle, period_us);
  ESP_ERROR_CHECK(err);
  state_->is_running = true;
}

void ESP32Timer::Stop() {
  std::lock_guard<std::recursive_mutex> lock(state_->mutex);
  if (!state_->handle || !state_->is_running) {
    return;
  }
  esp_err_t err = esp_timer_stop(state_->handle);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGW(kTag, "Unexpected error during Stop: %s", esp_err_to_name(err));
  }
  state_->is_running = false;
}

void ESP32Timer::StaticCallback(void* arg) {
  ESP32Timer* timer = static_cast<ESP32Timer*>(arg);
  std::shared_ptr<State> locked_state;
  {
    std::lock_guard<std::mutex> main_lock(state_map_mutex);
    auto it = timer_states.find(timer);
    if (it != timer_states.end()) {
      locked_state = it->second.lock();
    }
  }
  if (!locked_state) {
    return;
  }
  {
    std::lock_guard<std::recursive_mutex> lock(locked_state->mutex);
    if (!locked_state->is_running) {
      return;
    }
    if (!locked_state->is_periodic) {
      locked_state->is_running = false;
    }
  }
  locked_state->callback();
}

}  // namespace ALC
