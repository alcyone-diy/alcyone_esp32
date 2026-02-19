#include "alc_utils.h"

#include "esp_err.h"
#include "esp_event.h"
#include "nvs_flash.h"

namespace {

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

namespace ALC {

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
  // Mémoire libre totale
  printf("Minimum:\n");
  printf("\tFree heap:         %lu bytes\n", minimum.free_heap);
  printf("\tMinimum free heap: %lu bytes\n", minimum.minimum_free_heap);
  printf("\tFree PSRAM:        %lu bytes\n", minimum.free_psram);
  printf("Current:\n");
  printf("\tFree heap:         %lu bytes\n", current.free_heap);
  printf("\tMinimum free heap: %lu bytes\n", current.minimum_free_heap);
  printf("\tFree PSRAM:        %lu bytes\n", current.free_psram);
  printf("Maximum:\n");
  printf("\tFree heap:         %lu bytes\n", minimum.free_heap);
  printf("\tMinimum free heap: %lu bytes\n", minimum.minimum_free_heap);
  printf("\tFree PSRAM:        %lu bytes\n", minimum.free_psram);
}

}  // namespace ALC
