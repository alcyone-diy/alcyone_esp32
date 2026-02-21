#pragma once

#include "esp_heap_caps.h"
#include "esp_err.h"
#include "alc_wifi_controller.h"
#include "alc_storage.h"

namespace ALC {

/**
 * @brief Custom deleter for memory allocated with heap_caps_malloc.
 * Useful for smart pointers managing PSRAM memory.
 */
struct PsramDeleter {
  void operator()(uint8_t* p) const {
    if (p) {
      heap_caps_free(p);
    }
  }
};

/**
 * @brief Initialize general ALC utilities.
 */
void Init();

/**
 * @brief Print current heap and PSRAM memory usage to stdout.
 */
void PrintMemory();

/**
 * @brief Add or update a WiFi credential in storage.
 *
 * The storage instance must be opened before calling this method.
 *
 * @param storage An opened Storage instance.
 * @param credential Credential to add or update.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t AddWifiCredential(Storage& storage, const WifiController::Credential& credential);

/**
 * @brief Remove a WiFi credential from storage.
 *
 * The storage instance must be opened before calling this method.
 *
 * @param storage An opened Storage instance.
 * @param ssid SSID of the credential to remove.
 * @return esp_err_t ESP_OK if removed, ESP_ERR_NOT_FOUND if not found, or an error code.
 */
esp_err_t RemoveWifiCredential(Storage& storage, const char* ssid);

/**
 * @brief Load all WiFi credentials from storage and add them to the WifiController.
 *
 * The storage instance must be opened before calling this method.
 *
 * @param storage An opened Storage instance.
 * @param wifi_controller WifiController instance.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t LoadWifiCredentials(Storage& storage, WifiController& wifi_controller);

}  // namespace ALC
