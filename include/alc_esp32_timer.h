#pragma once

#include "esp_timer.h"
#include <functional>
#include <mutex>
#include <memory>

namespace ALC {

/**
 * @brief A wrapper for the ESP-IDF esp_timer to provide a C++ interface.
 */
class ESP32Timer {
public:
  using TimerCallback = std::function<void()>;

  /**
   * @brief Internal state of the timer, shared with the callback.
   */
  struct State {
    std::recursive_mutex mutex;
    TimerCallback callback;
    esp_timer_handle_t handle{nullptr};
    bool is_running{false};
    bool is_periodic{false};
  };

  /**
   * @brief Construct a new ESP32Timer object.
   *
   * @param callback The function to call when the timer expires.
   */
  explicit ESP32Timer(TimerCallback callback);

  /**
   * @brief Destroy the ESP32Timer object.
   */
  ~ESP32Timer();

  // Delete copy/move constructors and assignment operators
  ESP32Timer(const ESP32Timer&) = delete;
  ESP32Timer& operator=(const ESP32Timer&) = delete;
  ESP32Timer(ESP32Timer&&) = delete;
  ESP32Timer& operator=(ESP32Timer&&) = delete;

  /**
   * @brief Start the timer to run once.
   *
   * @param timeout_us The timeout in microseconds.
   */
  void StartOnce(uint64_t timeout_us);

  /**
   * @brief Start the timer to run periodically.
   *
   * @param period_us The period in microseconds.
   */
  void StartPeriodic(uint64_t period_us);

  /**
   * @brief Stop the timer if it is running.
   */
  void Stop();

private:
  static void StaticCallback(void* arg);

  std::shared_ptr<State> state_;
};

}  // namespace ALC
