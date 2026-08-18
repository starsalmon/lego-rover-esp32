#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#ifndef ROVER_I2C_HZ
#define ROVER_I2C_HZ 100000
#endif
#ifndef ROVER_I2C_TIMEOUT_MS
#define ROVER_I2C_TIMEOUT_MS 50
#endif

// MPU6050, MCP23008, and PCA9685 share one Wire bus but are now touched from
// two different tasks (main loop + pan-servo task). Wire is not reentrant, so
// every transaction must hold this mutex for its whole begin/write/end (or
// begin/write/end/requestFrom/read) sequence.
inline SemaphoreHandle_t rover_i2c_mutex_handle() {
  static SemaphoreHandle_t m = xSemaphoreCreateMutex();
  return m;
}

/** RAII guard: take the shared I2C mutex on construction, release on scope exit. */
struct RoverI2cGuard {
  RoverI2cGuard() { xSemaphoreTake(rover_i2c_mutex_handle(), portMAX_DELAY); }
  ~RoverI2cGuard() { xSemaphoreGive(rover_i2c_mutex_handle()); }
  RoverI2cGuard(const RoverI2cGuard &) = delete;
  RoverI2cGuard &operator=(const RoverI2cGuard &) = delete;
};

/** One shared I2C bus for MPU6050, MCP23008, PCA9685. Always applies a Wire timeout. */
inline void rover_i2c_begin(int sda, int scl) {
  static bool inited = false;
  static int last_sda = -1;
  static int last_scl = -1;
  if (!inited || last_sda != sda || last_scl != scl) {
    Wire.begin(sda, scl);
    delay(10);
    last_sda = sda;
    last_scl = scl;
    inited = true;
  }
  Wire.setClock(ROVER_I2C_HZ);
  Wire.setTimeOut(ROVER_I2C_TIMEOUT_MS);
}

inline void rover_i2c_touch() {
  Wire.setTimeOut(ROVER_I2C_TIMEOUT_MS);
}
