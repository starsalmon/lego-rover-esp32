#pragma once
#include <Arduino.h>

class Mpu6050 {
 public:
  bool begin(int sda, int scl);
  bool ok() const { return _ok; }
  bool read(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
 private:
  bool _ok = false;
  bool _write(uint8_t reg, uint8_t val);
  bool _read(uint8_t reg, uint8_t *buf, size_t len);
};
