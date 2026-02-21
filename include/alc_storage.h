#pragma once

#include <string>
#include <vector>
#include "esp_err.h"
#include "nvs.h"
#include "cJSON.h"

namespace ALC {

/**
 * @brief Simple key-value storage class using ESP32 NVS.
 *
 * Supports storing integers, floats, strings, arrays, and dictionaries.
 * Arrays and dictionaries are stored as JSON strings.
 *
 * Note: NVS key length is limited to 15 characters.
 */
class Storage {
public:
  /**
   * @brief Construct an empty Storage object. Use Open() to initialize.
   */
  Storage() = default;

  /**
   * @brief Construct a Storage object with a namespace. Use Open() to initialize.
   *
   * @param name_space NVS namespace name (max 15 characters).
   */
  explicit Storage(const std::string& name_space);

  /**
   * @brief Destroy the Storage object and close the NVS handle.
   */
  ~Storage();

  // Delete copy/move constructors and assignment operators
  Storage(const Storage&) = delete;
  Storage& operator=(const Storage&) = delete;
  Storage(Storage&&) = delete;
  Storage& operator=(Storage&&) = delete;

  /**
   * @brief Open the NVS namespace.
   *
   * @return esp_err_t ESP_OK on success, or an error code.
   */
  esp_err_t Open();

  /**
   * @brief Close the current NVS namespace.
   *
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t Close();

  /**
   * @brief Check if a key exists in the current namespace.
   *
   * @param key Key name (max 15 characters).
   * @return true if the key exists, false otherwise.
   */
  bool KeyExists(const std::string& key);

  /**
   * @brief Store an integer value.
   *
   * @param key Key name.
   * @param value Value to store.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t SetInt(const std::string& key, int32_t value);

  /**
   * @brief Retrieve an integer value.
   *
   * @param key Key name.
   * @param value Reference to store the retrieved value.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t GetInt(const std::string& key, int32_t& value);

  /**
   * @brief Store a float value (stored as uint32 bit-pattern).
   *
   * @param key Key name.
   * @param value Value to store.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t SetFloat(const std::string& key, float value);

  /**
   * @brief Retrieve a float value.
   *
   * @param key Key name.
   * @param value Reference to store the retrieved value.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t GetFloat(const std::string& key, float& value);

  /**
   * @brief Store a string value.
   *
   * @param key Key name.
   * @param value Value to store.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t SetString(const std::string& key, const std::string& value);

  /**
   * @brief Retrieve a string value.
   *
   * @param key Key name.
   * @param value Reference to store the retrieved value.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t GetString(const std::string& key, std::string& value);

  /**
   * @brief Store an array or dictionary as a JSON object.
   *
   * @param key Key name.
   * @param json cJSON object to store.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t SetJSON(const std::string& key, const cJSON* json);

  /**
   * @brief Retrieve an array or dictionary as a JSON object.
   *
   * @param key Key name.
   * @return cJSON* The parsed JSON object, or NULL if not found or invalid.
   *                The caller is responsible for calling cJSON_Delete() on the returned object.
   */
  cJSON* GetJSON(const std::string& key);

  /**
   * @brief Store an array as a JSON object.
   *
   * @param key Key name.
   * @param array cJSON array to store.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t SetArray(const std::string& key, const cJSON* array) { return SetJSON(key, array); }

  /**
   * @brief Retrieve an array as a JSON object.
   *
   * @param key Key name.
   * @return cJSON* The parsed JSON array, or NULL if not found or invalid.
   */
  cJSON* GetArray(const std::string& key) { return GetJSON(key); }

  /**
   * @brief Store a dictionary as a JSON object.
   *
   * @param key Key name.
   * @param dict cJSON dictionary to store.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t SetDictionary(const std::string& key, const cJSON* dict) { return SetJSON(key, dict); }

  /**
   * @brief Retrieve a dictionary as a JSON object.
   *
   * @param key Key name.
   * @return cJSON* The parsed JSON dictionary, or NULL if not found or invalid.
   */
  cJSON* GetDictionary(const std::string& key) { return GetJSON(key); }

  /**
   * @brief Erase a specific key from storage.
   *
   * @param key Key name.
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t EraseKey(const std::string& key);

  /**
   * @brief Erase all keys in the current namespace.
   *
   * @return esp_err_t ESP_OK on success.
   */
  esp_err_t EraseAll();

private:
  std::string name_space_;
  nvs_handle_t handle_{0};
  bool opened_{false};
};

} // namespace ALC
