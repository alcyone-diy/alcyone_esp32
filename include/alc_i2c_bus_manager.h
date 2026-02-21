#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <functional>
#include <vector>
#include <list>
#include <mutex>

namespace ALC {

/**
 * @brief Manages an I2C bus and serializes requests through a dedicated task.
 */
class I2CBusManager {
public:
  using Operation = std::function<esp_err_t(i2c_port_t)>;
  using Callback = std::function<void(esp_err_t)>;

  struct Request {
    Operation op;
    Callback callback;
    TickType_t schedule_time;
  };

  /**
   * @brief Construct a new I2CBusManager object.
   * @param port I2C port number.
   */
  explicit I2CBusManager(i2c_port_t port);

  ~I2CBusManager();

  // Delete copy/move
  I2CBusManager(const I2CBusManager&) = delete;
  I2CBusManager& operator=(const I2CBusManager&) = delete;

  /**
   * @brief Initialize the I2C driver and start the management task.
   * @param sda_pin SDA GPIO number.
   * @param scl_pin SCL GPIO number.
   * @param clk_speed I2C clock speed in Hz.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t Init(int sda_pin, int scl_pin, uint32_t clk_speed = 100000);

  /**
   * @brief Enqueue an I2C request.
   * @param op The I2C operation to perform.
   * @param cb Callback called when operation is complete.
   * @param delay_ticks Optional delay before executing the request.
   */
  void Enqueue(Operation op, Callback cb = nullptr, TickType_t delay_ticks = 0);

  /**
   * @brief Perform a synchronous I2C write.
   *
   * This method is context-aware:
   * - If called from within an Enqueue operation (i.e., from the I2CBusManager task),
   *   it executes the I2C transaction immediately on the bus.
   * - If called from another task, it enqueues the request and blocks the calling
   *   task until the transaction is complete.
   *
   * @param address I2C device address.
   * @param data Pointer to the data to write.
   * @param len Number of bytes to write.
   * @param timeout_ms I2C operation timeout in milliseconds.
   * @return esp_err_t ESP_OK on success, or an error code.
   */
  esp_err_t Write(uint8_t address, const uint8_t* data, size_t len, uint32_t timeout_ms = 100);

  /**
   * @brief Perform a synchronous I2C read.
   *
   * This method is context-aware:
   * - If called from within an Enqueue operation, it executes immediately.
   * - If called from another task, it enqueues the request and blocks the caller.
   *
   * @param address I2C device address.
   * @param buffer Buffer to store read data.
   * @param len Number of bytes to read.
   * @param timeout_ms I2C operation timeout in milliseconds.
   * @return esp_err_t ESP_OK on success, or an error code.
   */
  esp_err_t Read(uint8_t address, uint8_t* buffer, size_t len, uint32_t timeout_ms = 100);

  /**
   * @brief Perform a synchronous I2C write followed by a read.
   *
   * This method is context-aware:
   * - If called from within an Enqueue operation, it executes immediately.
   * - If called from another task, it enqueues the request and blocks the caller.
   *
   * @param address I2C device address.
   * @param write_data Pointer to the data to write.
   * @param write_len Number of bytes to write.
   * @param read_buffer Buffer to store read data.
   * @param read_len Number of bytes to read.
   * @param timeout_ms I2C operation timeout in milliseconds.
   * @return esp_err_t ESP_OK on success, or an error code.
   */
  esp_err_t WriteRead(uint8_t address, const uint8_t* write_data, size_t write_len,
                     uint8_t* read_buffer, size_t read_len, uint32_t timeout_ms = 100);

private:
  static void TaskEntry(void* pvParameters);
  void TaskLoop();

  i2c_port_t port_;
  TaskHandle_t task_handle_{nullptr};
  SemaphoreHandle_t wake_sem_{nullptr};
  std::mutex mutex_;

  std::vector<Request> immediate_requests_;
  std::list<Request> delayed_requests_;
};

} // namespace ALC
