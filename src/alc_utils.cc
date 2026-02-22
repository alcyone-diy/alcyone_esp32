#include "alc_utils.h"

#include "esp_err.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "esp_log.h"
#include <cstring>
#include <inttypes.h>

namespace ALC {

namespace {

constexpr char kTag[] = "ALC_Utils";

constexpr char kWifiCredsKey[] = "wifi_creds";
constexpr char kSsidKey[] = "ssid";
constexpr char kPasswordKey[] = "password";

typedef struct {
  uint32_t free_heap;
  uint32_t minimum_free_heap;
  uint32_t free_psram;
} MemoryValues;

MemoryValues GetMemoryValues() {
  MemoryValues result;
  result.free_heap = esp_get_free_heap_size();
  result.minimum_free_heap = esp_get_minimum_free_heap_size();
  result.free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  return result;
}

}  // namespace

void Init() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ret = esp_event_loop_create_default();
  ESP_ERROR_CHECK(ret);
}

void PrintMemory() {
  static MemoryValues minimum = {};
  static MemoryValues maximum = {};

  MemoryValues current = GetMemoryValues();
  if (minimum.free_heap == 0) {
    minimum = current;
    maximum = current;
  }
  if (minimum.free_heap > current.free_heap) {
    minimum.free_heap = current.free_heap;
  }
  if (minimum.minimum_free_heap > current.minimum_free_heap) {
    minimum.minimum_free_heap = current.minimum_free_heap;
  }
  if (minimum.free_psram > current.free_psram) {
    minimum.free_psram = current.free_psram;
  }
  if (maximum.free_heap < current.free_heap) {
    maximum.free_heap = current.free_heap;
  }
  if (maximum.minimum_free_heap < current.minimum_free_heap) {
    maximum.minimum_free_heap = current.minimum_free_heap;
  }
  if (maximum.free_psram < current.free_psram) {
    maximum.free_psram = current.free_psram;
  }

  printf("Minimum:\n");
  printf("\tFree heap:         %" PRIu32 " bytes\n", minimum.free_heap);
  printf("\tMinimum free heap: %" PRIu32 " bytes\n", minimum.minimum_free_heap);
  printf("\tFree PSRAM:        %" PRIu32 " bytes\n", minimum.free_psram);
  printf("Current:\n");
  printf("\tFree heap:         %" PRIu32 " bytes\n", current.free_heap);
  printf("\tMinimum free heap: %" PRIu32 " bytes\n", current.minimum_free_heap);
  printf("\tFree PSRAM:        %" PRIu32 " bytes\n", current.free_psram);
  printf("Maximum:\n");
  printf("\tFree heap:         %" PRIu32 " bytes\n", maximum.free_heap);
  printf("\tMinimum free heap: %" PRIu32 " bytes\n", maximum.minimum_free_heap);
  printf("\tFree PSRAM:        %" PRIu32 " bytes\n", maximum.free_psram);
}

esp_err_t AddWifiCredential(Storage& storage, const WifiController::Credential& credential) {
  cJSON* array = storage.GetArray(kWifiCredsKey);
  if (array == nullptr) {
    array = cJSON_CreateArray();
    if (array == nullptr) {
      return ESP_ERR_NO_MEM;
    }
  }

  bool found = false;
  cJSON* item = nullptr;
  cJSON_ArrayForEach(item, array) {
    cJSON* ssid = cJSON_GetObjectItem(item, kSsidKey);
    if (cJSON_IsString(ssid) && credential.ssid == ssid->valuestring) {
      cJSON_ReplaceItemInObject(item, kPasswordKey, cJSON_CreateString(credential.password.c_str()));
      found = true;
      break;
    }
  }

  if (!found) {
    cJSON* new_item = cJSON_CreateObject();
    if (new_item != nullptr) {
      cJSON_AddStringToObject(new_item, kSsidKey, credential.ssid.c_str());
      cJSON_AddStringToObject(new_item, kPasswordKey, credential.password.c_str());
      cJSON_AddItemToArray(array, new_item);
    }
  }

  esp_err_t err = storage.SetArray(kWifiCredsKey, array);
  cJSON_Delete(array);
  return err;
}

esp_err_t RemoveWifiCredential(Storage& storage, const char* ssid) {
  cJSON* array = storage.GetArray(kWifiCredsKey);
  if (array == nullptr) {
    return ESP_ERR_NOT_FOUND;
  }

  int index = 0;
  bool found = false;
  cJSON* item = nullptr;
  cJSON_ArrayForEach(item, array) {
    cJSON* ssid_item = cJSON_GetObjectItem(item, kSsidKey);
    if (cJSON_IsString(ssid_item) && strcmp(ssid, ssid_item->valuestring) == 0) {
      cJSON_DeleteItemFromArray(array, index);
      found = true;
      break;
    }
    index++;
  }

  esp_err_t err = ESP_OK;
  if (found) {
    err = storage.SetArray(kWifiCredsKey, array);
  } else {
    err = ESP_ERR_NOT_FOUND;
  }

  cJSON_Delete(array);
  return err;
}

esp_err_t LoadWifiCredentials(Storage& storage, WifiController& wifi_controller) {
  cJSON* array = storage.GetArray(kWifiCredsKey);
  if (array == nullptr) {
    return ESP_OK;
  }

  cJSON* item = nullptr;
  cJSON_ArrayForEach(item, array) {
    cJSON* ssid = cJSON_GetObjectItem(item, kSsidKey);
    cJSON* password = cJSON_GetObjectItem(item, kPasswordKey);
    if (cJSON_IsString(ssid) && cJSON_IsString(password)) {
      wifi_controller.AddCredential({ssid->valuestring, password->valuestring});
    }
  }

  cJSON_Delete(array);
  return ESP_OK;
}

}  // namespace ALC
