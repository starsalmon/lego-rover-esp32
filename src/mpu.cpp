#include "mpu.h"

#include <Wire.h>

#include "rover_i2c.h"

static constexpr uint8_t MPU_ADDR = 0x68;
static constexpr uint8_t kWhoAmI = 0x75;
static constexpr uint8_t kWhoAmIExpected = 0x68;

bool Mpu6050::begin(int sda, int scl) {
  _ok = false;
  rover_i2c_begin(sda, scl);
  delay(50);

  uint8_t who = 0;
  if (!_read(kWhoAmI, &who, 1) || who != kWhoAmIExpected) {
    Serial.printf("MPU6050 WHO_AM_I=0x%02x (expected 0x%02x)\n", who, kWhoAmIExpected);
    return false;
  }

  if (!_write(0x6B, 0x00)) return false;  // wake
  delay(20);
  if (!_write(0x1B, 0x08)) return false;  // ±500 °/s
  if (!_write(0x1C, 0x10)) return false;  // ±8g
  rover_i2c_touch();
  _ok = true;
  return true;
}

bool Mpu6050::read(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  if (!_ok) return false;
  rover_i2c_touch();
  uint8_t d[14];
  if (!_read(0x3B, d, 14)) return false;
  int16_t rax = (int16_t)((d[0] << 8) | d[1]);
  int16_t ray = (int16_t)((d[2] << 8) | d[3]);
  int16_t raz = (int16_t)((d[4] << 8) | d[5]);
  int16_t rgx = (int16_t)((d[8] << 8) | d[9]);
  int16_t rgy = (int16_t)((d[10] << 8) | d[11]);
  int16_t rgz = (int16_t)((d[12] << 8) | d[13]);
  ax = rax / 4096.0f;
  ay = ray / 4096.0f;
  az = raz / 4096.0f;
  gx = rgx / 65.5f * (PI / 180.0f);
  gy = rgy / 65.5f * (PI / 180.0f);
  gz = rgz / 65.5f * (PI / 180.0f);
  return true;
}

bool Mpu6050::_write(uint8_t reg, uint8_t val) {
  RoverI2cGuard guard;
  rover_i2c_touch();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool Mpu6050::_read(uint8_t reg, uint8_t *buf, size_t len) {
  RoverI2cGuard guard;
  rover_i2c_touch();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, (int)len) != (int)len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}
