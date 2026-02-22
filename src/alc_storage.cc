#include "alc_storage.h"
#include <cstring>
#include <vector>
#include "esp_log.h"
#include "nvs_flash.h"

namespace {
constexpr char kTag[] = "ALC_Storage";
}

namespace ALC {

Storage::Storage(const std::string& name_space) : name_space_(name_space) {}

Storage::~Storage() {
  Close();
}

esp_err_t Storage::Open() {
  Close();
  if (name_space_.empty()) {
    return ESP_ERR_INVALID_STATE;
  }
  esp_err_t err = nvs_open(name_space_.c_str(), NVS_READWRITE, &handle_);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "Error (%s) opening NVS handle for namespace %s!", esp_err_to_name(err),
             name_space_.c_str());
  } else {
    opened_ = true;
  }
  return err;
}

esp_err_t Storage::Close() {
  if (opened_) {
    nvs_close(handle_);
    opened_ = false;
  }
  return ESP_OK;
}

bool Storage::KeyExists(const std::string& key) {
  if (!opened_) {
    return false;
  }

  nvs_iterator_t it = NULL;
  esp_err_t err = nvs_entry_find(NVS_DEFAULT_PART_NAME, name_space_.c_str(), NVS_TYPE_ANY, &it);
  while (err == ESP_OK && it != NULL) {
    nvs_entry_info_t info;
    nvs_entry_info(it, &info);
    if (key == info.key) {
      nvs_release_iterator(it);
      return true;
    }
    err = nvs_entry_next(&it);
  }

  if (it != NULL) {
    nvs_release_iterator(it);
  }
  return false;
}

esp_err_t Storage::SetInt(const std::string& key, int32_t value) {
  if (!opened_) {
    return ESP_ERR_INVALID_STATE;
  }
  esp_err_t err = nvs_set_i32(handle_, key.c_str(), value);
  if (err == ESP_OK) {
    err = nvs_commit(handle_);
  }
  return err;
}

esp_err_t Storage::GetInt(const std::string& key, int32_t& value) {
  if (!opened_) {
    return ESP_ERR_INVALID_STATE;
  }
  return nvs_get_i32(handle_, key.c_str(), &value);
}

esp_err_t Storage::SetFloat(const std::string& key, float value) {
  if (!opened_) {
    return ESP_ERR_INVALID_STATE;
  }
  uint32_t u32_val;
  std::memcpy(&u32_val, &value, sizeof(float));
  esp_err_t err = nvs_set_u32(handle_, key.c_str(), u32_val);
  if (err == ESP_OK) {
    err = nvs_commit(handle_);
  }
  return err;
}

esp_err_t Storage::GetFloat(const std::string& key, float& value) {
  if (!opened_) {
    return ESP_ERR_INVALID_STATE;
  }
  uint32_t u32_val;
  esp_err_t err = nvs_get_u32(handle_, key.c_str(), &u32_val);
  if (err == ESP_OK) {
    std::memcpy(&value, &u32_val, sizeof(float));
  }
  return err;
}

esp_err_t Storage::SetString(const std::string& key, const std::string& value) {
  if (!opened_) {
    return ESP_ERR_INVALID_STATE;
  }
  esp_err_t err = nvs_set_str(handle_, key.c_str(), value.c_str());
  if (err == ESP_OK) {
    err = nvs_commit(handle_);
  }
  return err;
}

esp_err_t Storage::GetString(const std::string& key, std::string& value) {
  if (!opened_) {
    return ESP_ERR_INVALID_STATE;
  }
  size_t required_size;
  esp_err_t err = nvs_get_str(handle_, key.c_str(), NULL, &required_size);
  if (err != ESP_OK) {
    return err;
  }

  std::vector<char> buffer(required_size);
  err = nvs_get_str(handle_, key.c_str(), buffer.data(), &required_size);
  if (err == ESP_OK) {
    value.assign(buffer.data(), required_size - 1); // Exclude null terminator
  }
  return err;
}

esp_err_t Storage::SetJSON(const std::string& key, const cJSON* json) {
  if (!opened_) {
    return ESP_ERR_INVALID_STATE;
  }
  char* str = cJSON_PrintUnformatted(json);
  if (str == NULL) {
    return ESP_ERR_NO_MEM;
  }
  esp_err_t err = SetString(key, str);
  free(str);
  return err;
}

cJSON* Storage::GetJSON(const std::string& key) {
  std::string s;
  if (GetString(key, s) != ESP_OK) {
    return NULL;
  }
  return cJSON_Parse(s.c_str());
}

esp_err_t Storage::EraseKey(const std::string& key) {
  if (!opened_) {
    return ESP_ERR_INVALID_STATE;
  }
  esp_err_t err = nvs_erase_key(handle_, key.c_str());
  if (err == ESP_OK) {
    err = nvs_commit(handle_);
  }
  return err;
}

esp_err_t Storage::EraseAll() {
  if (!opened_) {
    return ESP_ERR_INVALID_STATE;
  }
  esp_err_t err = nvs_erase_all(handle_);
  if (err == ESP_OK) {
    err = nvs_commit(handle_);
  }
  return err;
}

} // namespace ALC
