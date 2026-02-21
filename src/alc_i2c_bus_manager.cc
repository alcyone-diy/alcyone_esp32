#include "alc_i2c_bus_manager.h"
#include "esp_log.h"

static const char* TAG = "I2CBusManager";

namespace ALC {

I2CBusManager::I2CBusManager(i2c_port_t port) : port_(port) {
  wake_sem_ = xSemaphoreCreateBinary();
}

I2CBusManager::~I2CBusManager() {
  if (task_handle_) {
    vTaskDelete(task_handle_);
  }
  if (wake_sem_) {
    vSemaphoreDelete(wake_sem_);
  }
  i2c_driver_delete(port_);
}

esp_err_t I2CBusManager::Init(int sda_pin, int scl_pin, uint32_t clk_speed) {
  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = sda_pin;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = scl_pin;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = clk_speed;
  conf.clk_flags = 0;

  esp_err_t err = i2c_param_config(port_, &conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
    return err;
  }

  err = i2c_driver_install(port_, conf.mode, 0, 0, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
    return err;
  }

  BaseType_t ret = xTaskCreate(TaskEntry, "i2c_bus_mgr", 4096, this, 5, &task_handle_);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create task");
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "I2CBusManager initialized on port %d", port_);
  return ESP_OK;
}

void I2CBusManager::Enqueue(Operation op, Callback cb, TickType_t delay_ticks) {
  TickType_t now = xTaskGetTickCount();
  Request req = {std::move(op), std::move(cb), now + delay_ticks};

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (delay_ticks == 0) {
      immediate_requests_.push_back(std::move(req));
    } else {
      delayed_requests_.push_back(std::move(req));
    }
  }
  xSemaphoreGive(wake_sem_);
}

esp_err_t I2CBusManager::Write(BusToken& token, uint8_t address, const uint8_t* data, size_t len,
                               uint32_t timeout_ms) {
  if (token.manager() != this) return ESP_ERR_INVALID_ARG;
  if (!token.is_valid()) return ESP_ERR_INVALID_STATE;
  return i2c_master_write_to_device(port_, address, data, len, pdMS_TO_TICKS(timeout_ms));
}

esp_err_t I2CBusManager::Read(BusToken& token, uint8_t address, uint8_t* buffer, size_t len,
                              uint32_t timeout_ms) {
  if (token.manager() != this) return ESP_ERR_INVALID_ARG;
  if (!token.is_valid()) return ESP_ERR_INVALID_STATE;
  return i2c_master_read_from_device(port_, address, buffer, len, pdMS_TO_TICKS(timeout_ms));
}

esp_err_t I2CBusManager::WriteRead(BusToken& token, uint8_t address, const uint8_t* write_data,
                                   size_t write_len, uint8_t* read_buffer, size_t read_len,
                                   uint32_t timeout_ms) {
  if (token.manager() != this) return ESP_ERR_INVALID_ARG;
  if (!token.is_valid()) return ESP_ERR_INVALID_STATE;
  return i2c_master_write_read_device(port_, address, write_data, write_len, read_buffer, read_len,
                                      pdMS_TO_TICKS(timeout_ms));
}

esp_err_t I2CBusManager::WriteRegister(BusToken& token, uint8_t address, uint8_t reg, uint8_t value,
                                       uint32_t timeout_ms) {
  uint8_t data[2] = {reg, value};
  return Write(token, address, data, 2, timeout_ms);
}

esp_err_t I2CBusManager::ReadRegister(BusToken& token, uint8_t address, uint8_t reg, uint8_t* value,
                                      uint32_t timeout_ms) {
  return ReadRegisters(token, address, reg, value, 1, timeout_ms);
}

esp_err_t I2CBusManager::ReadRegisters(BusToken& token, uint8_t address, uint8_t reg, uint8_t* data,
                                       size_t len, uint32_t timeout_ms) {
  return WriteRead(token, address, &reg, 1, data, len, timeout_ms);
}

void I2CBusManager::TaskEntry(void* pvParameters) {
  static_cast<I2CBusManager*>(pvParameters)->TaskLoop();
}

void I2CBusManager::TaskLoop() {
  std::vector<Request> to_run;
  while (true) {
    TickType_t now = xTaskGetTickCount();
    TickType_t wait_ticks = portMAX_DELAY;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!immediate_requests_.empty()) {
        wait_ticks = 0;
      } else {
        for (const auto& req : delayed_requests_) {
          TickType_t rem;
          TickType_t sched = req.schedule_time;
          // Wrap-around safe comparison: is sched in the past or now?
          if ((TickType_t)(now - sched) < ((TickType_t)-1 / 2)) {
            rem = 0;
          } else {
            rem = sched - now;
          }
          if (rem < wait_ticks) {
            wait_ticks = rem;
          }
        }
      }
    }

    if (wait_ticks > 0) {
      xSemaphoreTake(wake_sem_, wait_ticks);
    }

    // Process requests
    to_run.clear();
    now = xTaskGetTickCount();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!immediate_requests_.empty()) {
        to_run.swap(immediate_requests_);
      }

      auto it = delayed_requests_.begin();
      while (it != delayed_requests_.end()) {
        TickType_t sched = it->schedule_time;
        if ((TickType_t)(now - sched) < ((TickType_t)-1 / 2)) {
             to_run.push_back(std::move(*it));
             it = delayed_requests_.erase(it);
        } else {
          ++it;
        }
      }
    }

    for (auto& req : to_run) {
      BusToken token(BusToken::Key{}, this);
      esp_err_t err = req.op(token);
      token.invalidate();
      if (req.callback) {
        req.callback(err);
      }
    }
  }
}

} // namespace ALC
