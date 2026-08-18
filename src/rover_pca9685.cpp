#include "rover_pca9685.h"

#include <Wire.h>

#include "rover_i2c.h"

namespace {

constexpr uint8_t kMode1 = 0x00;
constexpr uint8_t kMode2 = 0x01;
constexpr uint8_t kPrescale = 0xFE;
constexpr uint8_t kLed0OnL = 0x06;

}  // namespace

bool RoverPca9685::writeReg(uint8_t reg, uint8_t val) {
  RoverI2cGuard guard;
  rover_i2c_touch();
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

// PCA9685 auto-increments its register pointer within one transaction, so all
// 4 bytes of an ON/OFF pair can go out atomically. Writing them as 4 separate
// start/stop transactions (the old code) leaves a window where a dropped or
// delayed byte — e.g. from bus contention with the MPU6050/MCP23008 sharing
// this I2C bus every loop tick — tears the ON/OFF pair apart and produces one
// bad PWM cycle: a visible servo jump, not a hardware fault.
bool RoverPca9685::writeReg4(uint8_t reg, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
  RoverI2cGuard guard;
  rover_i2c_touch();
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  Wire.write(b0);
  Wire.write(b1);
  Wire.write(b2);
  Wire.write(b3);
  return Wire.endTransmission() == 0;
}

bool RoverPca9685::readReg(uint8_t reg, uint8_t &val) const {
  RoverI2cGuard guard;
  rover_i2c_touch();
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(static_cast<int>(_addr), 1) != 1) return false;
  val = static_cast<uint8_t>(Wire.read());
  return true;
}

bool RoverPca9685::begin(uint8_t addr) {
  _addr = addr;
  uint8_t mode1 = 0;
  if (!readReg(kMode1, mode1)) {
    _addr = 0;
    return false;
  }

  constexpr int freq_hz = 50;
  const uint8_t prescale = static_cast<uint8_t>(25000000UL / (4096UL * freq_hz) - 1);

  writeReg(kMode1, 0x00);
  writeReg(kMode2, 0x04);
  if (!readReg(kMode1, mode1)) {
    _addr = 0;
    return false;
  }
  const uint8_t old_mode1 = mode1;
  writeReg(kMode1, static_cast<uint8_t>((old_mode1 & 0x7F) | 0x10));
  writeReg(kPrescale, prescale);
  writeReg(kMode1, static_cast<uint8_t>(old_mode1 & ~0x10));
  delay(5);
  writeReg(kMode1, static_cast<uint8_t>((old_mode1 & ~0x10) | 0x80));
  delay(5);
  writeReg(kMode1, static_cast<uint8_t>((old_mode1 & ~0x10) | 0x20));
  return true;
}

void RoverPca9685::setPulseUs(uint8_t channel, uint16_t pulse_us) {
  if (!_addr || channel > 15) return;
  const uint16_t counts = static_cast<uint16_t>((static_cast<uint32_t>(pulse_us) * 4096UL) / 20000UL);
  const uint8_t reg = static_cast<uint8_t>(kLed0OnL + 4 * channel);
  writeReg4(reg, 0, 0, static_cast<uint8_t>(counts & 0xFF),
            static_cast<uint8_t>((counts >> 8) & 0xFF));
}

void RoverPca9685::setAngle(uint8_t channel, float deg, float min_deg, float max_deg,
                            uint16_t min_us, uint16_t max_us) {
  if (max_deg <= min_deg) {
    setPulseUs(channel, min_us);
    return;
  }
  deg = deg < min_deg ? min_deg : (deg > max_deg ? max_deg : deg);
  const float t = (deg - min_deg) / (max_deg - min_deg);
  const uint16_t pulse = static_cast<uint16_t>(min_us + (max_us - min_us) * t);
  setPulseUs(channel, pulse);
}
