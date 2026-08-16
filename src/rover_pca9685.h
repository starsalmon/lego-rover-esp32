#pragma once

#include <stdint.h>

// PCA9685 @ 0x40 on shared I2C (MPU6050 / MCP23008 bus).

#ifndef PCA9685_ADDR
#define PCA9685_ADDR 0x40
#endif

#ifndef ROVER_SERVO_CHANNEL
#define ROVER_SERVO_CHANNEL 0
#endif

class RoverPca9685 {
 public:
  bool begin(uint8_t addr = PCA9685_ADDR);
  bool ok() const { return _addr != 0; }

  void setPulseUs(uint8_t channel, uint16_t pulse_us);
  void setAngle(uint8_t channel, float deg, float min_deg, float max_deg, uint16_t min_us,
                uint16_t max_us);

 private:
  bool writeReg(uint8_t reg, uint8_t val);
  bool writeReg4(uint8_t reg, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3);
  bool readReg(uint8_t reg, uint8_t &val) const;

  uint8_t _addr = 0;
};
