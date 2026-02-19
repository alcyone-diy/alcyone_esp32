#pragma once

#include "esp_timer.h"
#include <functional>
#include <mutex>
#include <memory>

namespace ALC {

class ESP32Timer {
public:
  using TimerCallback = std::function<void()>;
  struct State {
    std::recursive_mutex mutex;
    TimerCallback callback;
    esp_timer_handle_t handle{nullptr};
    bool is_running{false};
    bool is_periodic{false};
  };

  explicit ESP32Timer(TimerCallback callback);
  ~ESP32Timer();

  ESP32Timer(const ESP32Timer&) = delete;
  ESP32Timer& operator=(const ESP32Timer&) = delete;

  void StartOnce(uint64_t timeout_us);
  void StartPeriodic(uint64_t period_us);
  void Stop();

private:
  static void StaticCallback(void* arg);
  
  std::shared_ptr<State> state_;
};

}  // namespace ALC
