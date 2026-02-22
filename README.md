# ALC ESP32 Component

This repository contains a set of ESP-IDF components for ESP32 development, focusing on I2C sensor management, persistent storage, and WiFi connectivity.

## Supported ESP-IDF Version
This project is developed and tested using **ESP-IDF v6.1.0-dev**.

## Components

### I2C Bus Manager
Centralizes I2C communication into a single task to ensure serialized access and non-blocking operations. It supports both 7-bit and 10-bit addressing and uses the modern `driver/i2c_master.h` API.

### Sensors
All sensors follow an asynchronous API pattern and depend on the `I2CBusManager`.
- **BME280Sensor**: Temperature, Humidity, and Pressure sensor.
- **BNO086Sensor**: 9-axis IMU with integrated sensor fusion.
- **MaxM10sSensor**: u-blox GNSS module.

### Utilities
- **WifiController**: Manages WiFi connections, scanning, and automatic reconnection.
- **Storage**: Key-value storage wrapper for NVS, supporting complex data types via JSON.
- **ESP32Timer**: C++ wrapper for `esp_timer`.
- **ALC Utils**: General helper functions for memory monitoring and WiFi credential management.

## Coding Conventions
- All classes are within the `ALC` namespace.
- Sensor classes are suffixed with `Sensor`.
- Logging tags follow the `ALC_ClassName` pattern and are defined in anonymous namespaces.
- Asynchronous-only APIs are preferred for I/O operations.
