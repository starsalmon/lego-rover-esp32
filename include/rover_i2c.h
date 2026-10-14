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

/** RAII guard: take the shared I2C mutex on construction, release on scope exit.
 *  Optional timeout: if take fails, ok() is false and destructor is a no-op.
 *  IMU uses a short timeout so a long L8 dump cannot stall the 20 Hz tick. */
struct RoverI2cGuard {
  explicit RoverI2cGuard(TickType_t ticks = portMAX_DELAY)
      : _ok(xSemaphoreTake(rover_i2c_mutex_handle(), ticks) == pdTRUE) {}
  ~RoverI2cGuard() {
    if (_ok) {
      xSemaphoreGive(rover_i2c_mutex_handle());
    }
  }
  bool ok() const { return _ok; }
  explicit operator bool() const { return _ok; }
  RoverI2cGuard(const RoverI2cGuard &) = delete;
  RoverI2cGuard &operator=(const RoverI2cGuard &) = delete;

 private:
  bool _ok;
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
