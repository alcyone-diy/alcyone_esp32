#include "i2c/alc_i2c_bus_manager.h"
#include "esp_log.h"

namespace {
constexpr char kTag[] = "ALC_I2CBusManager";
}

namespace ALC {

I2CBusManager::I2CBusManager(i2c_port_t port) : port_(port) {
  wake_sem_ = xSemaphoreCreateBinary();
  done_sem_ = xSemaphoreCreateBinary();
}

I2CBusManager::~I2CBusManager() {
  running_ = false;
  if (wake_sem_) {
    xSemaphoreGive(wake_sem_);
  }

  if (task_handle_ && done_sem_) {
    xSemaphoreTake(done_sem_, pdMS_TO_TICKS(100));
  }

  if (done_sem_) {
    vSemaphoreDelete(done_sem_);
    done_sem_ = nullptr;
  }

  if (wake_sem_) {
    vSemaphoreDelete(wake_sem_);
    wake_sem_ = nullptr;
  }

  task_handle_ = nullptr;

  std::lock_guard<std::mutex> lock(mutex_);
  immediate_requests_.clear();
  delayed_requests_.clear();

  for (auto const& [addr, handle] : dev_handles_) {
    i2c_master_bus_rm_device(handle);
  }
  dev_handles_.clear();

  if (bus_handle_) {
    i2c_del_master_bus(bus_handle_);
    bus_handle_ = nullptr;
  }
}

esp_err_t I2CBusManager::Init(int sda_pin, int scl_pin, uint32_t clk_speed) {
  default_clk_speed_ = clk_speed;

  i2c_master_bus_config_t bus_conf = {};
  bus_conf.i2c_port = port_;
  bus_conf.sda_io_num = static_cast<gpio_num_t>(sda_pin);
  bus_conf.scl_io_num = static_cast<gpio_num_t>(scl_pin);
  bus_conf.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_conf.glitch_ignore_cnt = 7;
  bus_conf.flags.enable_internal_pullup = true;

  esp_err_t err = i2c_new_master_bus(&bus_conf, &bus_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "I2C new master bus failed: %s", esp_err_to_name(err));
    return err;
  }

  running_ = true;
  BaseType_t ret = xTaskCreate(TaskEntry, "i2c_bus_mgr", 4096, this, 5, &task_handle_);
  if (ret != pdPASS) {
    ESP_LOGE(kTag, "Failed to create task");
    return ESP_FAIL;
  }

  ESP_LOGI(kTag, "I2CBusManager initialized on port %d", port_);
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

esp_err_t I2CBusManager::Write(BusToken& token, uint16_t address, const uint8_t* data, size_t len,
                               uint32_t timeout_ms) {
  if (token.manager() != this) return ESP_ERR_INVALID_ARG;
  if (!token.is_valid()) return ESP_ERR_INVALID_STATE;

  i2c_master_dev_handle_t dev_handle;
  esp_err_t err = GetOrCreateDevice(address, &dev_handle);
  if (err != ESP_OK) return err;

  return i2c_master_transmit(dev_handle, data, len, static_cast<int>(timeout_ms));
}

esp_err_t I2CBusManager::Read(BusToken& token, uint16_t address, uint8_t* buffer, size_t len,
                              uint32_t timeout_ms) {
  if (token.manager() != this) return ESP_ERR_INVALID_ARG;
  if (!token.is_valid()) return ESP_ERR_INVALID_STATE;

  i2c_master_dev_handle_t dev_handle;
  esp_err_t err = GetOrCreateDevice(address, &dev_handle);
  if (err != ESP_OK) return err;

  return i2c_master_receive(dev_handle, buffer, len, static_cast<int>(timeout_ms));
}

esp_err_t I2CBusManager::WriteRead(BusToken& token, uint16_t address, const uint8_t* write_data,
                                   size_t write_len, uint8_t* read_buffer, size_t read_len,
                                   uint32_t timeout_ms) {
  if (token.manager() != this) return ESP_ERR_INVALID_ARG;
  if (!token.is_valid()) return ESP_ERR_INVALID_STATE;

  i2c_master_dev_handle_t dev_handle;
  esp_err_t err = GetOrCreateDevice(address, &dev_handle);
  if (err != ESP_OK) return err;

  return i2c_master_transmit_receive(dev_handle, write_data, write_len, read_buffer, read_len,
                                      static_cast<int>(timeout_ms));
}

esp_err_t I2CBusManager::WriteRegister(BusToken& token, uint16_t address, uint8_t reg, uint8_t value,
                                       uint32_t timeout_ms) {
  uint8_t data[2] = {reg, value};
  return Write(token, address, data, 2, timeout_ms);
}

esp_err_t I2CBusManager::ReadRegister(BusToken& token, uint16_t address, uint8_t reg, uint8_t* value,
                                      uint32_t timeout_ms) {
  return ReadRegisters(token, address, reg, value, 1, timeout_ms);
}

esp_err_t I2CBusManager::ReadRegisters(BusToken& token, uint16_t address, uint8_t reg, uint8_t* data,
                                       size_t len, uint32_t timeout_ms) {
  return WriteRead(token, address, &reg, 1, data, len, timeout_ms);
}

esp_err_t I2CBusManager::GetOrCreateDevice(uint16_t address, i2c_master_dev_handle_t* handle) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = dev_handles_.find(address);
  if (it != dev_handles_.end()) {
    *handle = it->second;
    return ESP_OK;
  }

  i2c_device_config_t dev_conf = {};
  dev_conf.dev_addr_length = (address > 0x7F) ? I2C_ADDR_BIT_LEN_10 : I2C_ADDR_BIT_LEN_7;
  dev_conf.device_address = address;
  dev_conf.scl_speed_hz = default_clk_speed_;

  esp_err_t err = i2c_master_bus_add_device(bus_handle_, &dev_conf, handle);
  if (err == ESP_OK) {
    dev_handles_[address] = *handle;
  }
  return err;
}

void I2CBusManager::TaskEntry(void* pvParameters) {
  static_cast<I2CBusManager*>(pvParameters)->TaskLoop();
}

void I2CBusManager::TaskLoop() {
  std::vector<Request> to_run;
  while (running_) {
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
  if (done_sem_) {
    xSemaphoreGive(done_sem_);
  }
  vTaskDelete(NULL);
}

} // namespace ALC
