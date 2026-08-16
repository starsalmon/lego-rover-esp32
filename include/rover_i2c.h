#pragma once

#include <Arduino.h>
#include <Wire.h>

#ifndef ROVER_I2C_HZ
#define ROVER_I2C_HZ 100000
#endif
#ifndef ROVER_I2C_TIMEOUT_MS
#define ROVER_I2C_TIMEOUT_MS 50
#endif

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
