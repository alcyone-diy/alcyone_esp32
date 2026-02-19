#pragma once

#include "esp_heap_caps.h"

namespace ALC {

struct PsramDeleter {
  void operator()(uint8_t* p) const {
    if (p) {
      heap_caps_free(p);
    }
  }
};

void Init();
void PrintMemory();

}  // namespace ALC
