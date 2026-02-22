#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <functional>
#include <vector>
#include <list>
#include <mutex>
#include <map>

namespace ALC {

/**
 * @brief Manages an I2C bus and serializes requests through a dedicated task.
 */
class I2CBusManager {
public:
  /**
   * @brief Token required to perform I2C operations.
   *
   * This token can only be created by I2CBusManager and is passed to enqueued
   * operations. This ensures that I2C operations can only be performed from
   * within the I2CBusManager task context.
   */
  class BusToken {
  public:
    struct Key {
      friend class I2CBusManager;
    private:
      Key() = default;
    };

    BusToken(Key, I2CBusManager* mgr) : manager_(mgr), valid_(true) {}
    BusToken(const BusToken&) = delete;
    BusToken& operator=(const BusToken&) = delete;

    bool is_valid() const { return valid_; }
    I2CBusManager* manager() const { return manager_; }
    void invalidate() { valid_ = false; }

  private:
    I2CBusManager* manager_;
    bool valid_;
  };

  using Operation = std::function<esp_err_t(BusToken&)>;
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

  // Delete copy/move constructors and assignment operators
  I2CBusManager(const I2CBusManager&) = delete;
  I2CBusManager& operator=(const I2CBusManager&) = delete;
  I2CBusManager(I2CBusManager&&) = delete;
  I2CBusManager& operator=(I2CBusManager&&) = delete;

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
   * @note This method can only be called from within an Enqueue operation,
   * as it requires a BusToken. The token must be valid and belong to this
   * manager. Multiple operations can be performed with the same token
   * within a single callback.
   *
   * @param token The BusToken provided to the enqueued operation.
   * @param address I2C device address.
   * @param data Pointer to the data to write.
   * @param len Number of bytes to write.
   * @param timeout_ms I2C operation timeout in milliseconds.
   * @return esp_err_t ESP_OK on success, or an error code.
   */
  esp_err_t Write(BusToken& token, uint16_t address, const uint8_t* data, size_t len,
                  uint32_t timeout_ms = 100);

  /**
   * @brief Perform a synchronous I2C read.
   *
   * @note This method can only be called from within an Enqueue operation,
   * as it requires a BusToken. The token must be valid and belong to this
   * manager. Multiple operations can be performed with the same token
   * within a single callback.
   *
   * @param token The BusToken provided to the enqueued operation.
   * @param address I2C device address.
   * @param buffer Buffer to store read data.
   * @param len Number of bytes to read.
   * @param timeout_ms I2C operation timeout in milliseconds.
   * @return esp_err_t ESP_OK on success, or an error code.
   */
  esp_err_t Read(BusToken& token, uint16_t address, uint8_t* buffer, size_t len,
                 uint32_t timeout_ms = 100);

  /**
   * @brief Perform a synchronous I2C write followed by a read.
   *
   * @note This method can only be called from within an Enqueue operation,
   * as it requires a BusToken. The token must be valid and belong to this
   * manager. Multiple operations can be performed with the same token
   * within a single callback.
   *
   * @param token The BusToken provided to the enqueued operation.
   * @param address I2C device address.
   * @param write_data Pointer to the data to write.
   * @param write_len Number of bytes to write.
   * @param read_buffer Buffer to store read data.
   * @param read_len Number of bytes to read.
   * @param timeout_ms I2C operation timeout in milliseconds.
   * @return esp_err_t ESP_OK on success, or an error code.
   */
  esp_err_t WriteRead(BusToken& token, uint16_t address, const uint8_t* write_data,
                      size_t write_len, uint8_t* read_buffer, size_t read_len,
                      uint32_t timeout_ms = 100);

  /**
   * @brief Write a single byte to a register.
   *
   * @param token The BusToken provided to the enqueued operation.
   * @param address I2C device address.
   * @param reg Register address.
   * @param value Value to write.
   * @param timeout_ms I2C operation timeout in milliseconds.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t WriteRegister(BusToken& token, uint16_t address, uint8_t reg, uint8_t value,
                          uint32_t timeout_ms = 100);

  /**
   * @brief Read a single byte from a register.
   *
   * @param token The BusToken provided to the enqueued operation.
   * @param address I2C device address.
   * @param reg Register address.
   * @param value Pointer to store the read value.
   * @param timeout_ms I2C operation timeout in milliseconds.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t ReadRegister(BusToken& token, uint16_t address, uint8_t reg, uint8_t* value,
                         uint32_t timeout_ms = 100);

  /**
   * @brief Read multiple bytes starting from a register address.
   *
   * @param token The BusToken provided to the enqueued operation.
   * @param address I2C device address.
   * @param reg Starting register address.
   * @param data Buffer to store read data.
   * @param len Number of bytes to read.
   * @param timeout_ms I2C operation timeout in milliseconds.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t ReadRegisters(BusToken& token, uint16_t address, uint8_t reg, uint8_t* data, size_t len,
                          uint32_t timeout_ms = 100);

private:
  static void TaskEntry(void* pvParameters);
  void TaskLoop();

  /**
   * @brief Get or create a device handle for the given address.
   * @param address I2C device address.
   * @param handle Pointer to store the device handle.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t GetOrCreateDevice(uint16_t address, i2c_master_dev_handle_t* handle);

  i2c_port_t port_;
  i2c_master_bus_handle_t bus_handle_{nullptr};
  bool running_{false};
  std::map<uint16_t, i2c_master_dev_handle_t> dev_handles_;
  uint32_t default_clk_speed_{100000};

  TaskHandle_t task_handle_{nullptr};
  SemaphoreHandle_t wake_sem_{nullptr};
  SemaphoreHandle_t done_sem_{nullptr};
  std::mutex mutex_;

  std::vector<Request> immediate_requests_;
  std::list<Request> delayed_requests_;
};

} // namespace ALC
