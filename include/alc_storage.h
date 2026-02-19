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
     * @brief Construct a new Storage object for a given namespace.
     *
     * @param name_space NVS namespace name (max 15 characters).
     */
    Storage(const std::string& name_space);

    /**
     * @brief Destroy the Storage object and close the NVS handle.
     */
    ~Storage();

    /**
     * @brief Open a specific NVS namespace.
     *
     * @param name_space NVS namespace name (max 15 characters).
     * @return esp_err_t ESP_OK on success, or an error code.
     */
    esp_err_t Open(const std::string& name_space);

    /**
     * @brief Check if a key exists in the current namespace.
     *
     * @param key Key name (max 15 characters).
     * @return true if the key exists, false otherwise.
     */
    bool KeyExists(const std::string& key);

    // Integer support
    esp_err_t SetInt(const std::string& key, int32_t value);
    esp_err_t GetInt(const std::string& key, int32_t& value);

    // Float support (stored as uint32 bit-pattern)
    esp_err_t SetFloat(const std::string& key, float value);
    esp_err_t GetFloat(const std::string& key, float& value);

    // String support
    esp_err_t SetString(const std::string& key, const std::string& value);
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

    // Specific aliases for clarity if desired
    esp_err_t SetArray(const std::string& key, const cJSON* array) { return SetJSON(key, array); }
    cJSON* GetArray(const std::string& key) { return GetJSON(key); }
    esp_err_t SetDictionary(const std::string& key, const cJSON* dict) { return SetJSON(key, dict); }
    cJSON* GetDictionary(const std::string& key) { return GetJSON(key); }

    /**
     * @brief Erase a specific key from storage.
     */
    esp_err_t EraseKey(const std::string& key);

    /**
     * @brief Erase all keys in the current namespace.
     */
    esp_err_t EraseAll();

private:
    std::string name_space_;
    nvs_handle_t handle_{0};
    bool opened_{false};
};

} // namespace ALC
