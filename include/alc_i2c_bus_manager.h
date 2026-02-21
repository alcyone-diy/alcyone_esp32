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
   * @brief Perform an asynchronous I2C write.
   *
   * This method is safe as it takes ownership of the data by moving the vector.
   *
   * @param address I2C device address.
   * @param data Data to write. Will be moved into the operation.
   * @param cb Optional callback called when operation is complete.
   * @param delay_ticks Optional delay before executing the request.
   * @param timeout_ms I2C operation timeout in milliseconds.
   */
  void Write(uint8_t address, std::vector<uint8_t> data, Callback cb = nullptr,
             TickType_t delay_ticks = 0, uint32_t timeout_ms = 100);

  /**
   * @brief Perform an asynchronous I2C read.
   *
   * @note The caller MUST ensure that the provided buffer remains valid until
   * the callback is executed. This is a zero-copy operation.
   *
   * @param address I2C device address.
   * @param buffer Buffer to store read data.
   * @param len Number of bytes to read.
   * @param cb Optional callback called when operation is complete.
   * @param delay_ticks Optional delay before executing the request.
   * @param timeout_ms I2C operation timeout in milliseconds.
   */
  void Read(uint8_t address, uint8_t* buffer, size_t len, Callback cb = nullptr,
            TickType_t delay_ticks = 0, uint32_t timeout_ms = 100);

  /**
   * @brief Perform an asynchronous I2C write followed by a read.
   *
   * Often used for reading registers (write register address, then read value).
   *
   * @note The caller MUST ensure that the read_buffer remains valid until
   * the callback is executed. The write_data is safely moved.
   *
   * @param address I2C device address.
   * @param write_data Data to write. Will be moved into the operation.
   * @param read_buffer Buffer to store read data.
   * @param read_len Number of bytes to read.
   * @param cb Optional callback called when operation is complete.
   * @param delay_ticks Optional delay before executing the request.
   * @param timeout_ms I2C operation timeout in milliseconds.
   */
  void WriteRead(uint8_t address, std::vector<uint8_t> write_data,
                 uint8_t* read_buffer, size_t read_len, Callback cb = nullptr,
                 TickType_t delay_ticks = 0, uint32_t timeout_ms = 100);

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
