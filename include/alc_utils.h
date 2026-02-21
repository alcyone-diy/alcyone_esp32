#pragma once

#include "esp_heap_caps.h"

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

}  // namespace ALC
