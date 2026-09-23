#include "rover_mcp_ir.h"

#include <Wire.h>

#include "rover_i2c.h"
#include "rover_pins_s3.h"

namespace {

constexpr uint8_t kRegIodir = 0x00;
constexpr uint8_t kRegOlat = 0x0A;
constexpr uint8_t kRegGpio = 0x09;

constexpr uint8_t kBodySchmittInMask =
    static_cast<uint8_t>((1u << BODY_IN_SPARE) | (1u << BODY_IN_WHEEL_L) |
                         (1u << BODY_IN_WHEEL_R));
constexpr uint8_t kBodyEmitterOutMask =
    static_cast<uint8_t>((1u << BODY_OUT_WHEEL_L) | (1u << BODY_OUT_WHEEL_R));
constexpr uint8_t kBodyVl53OutMask =
    static_cast<uint8_t>((1u << BODY_OUT_VL53_L8) | (1u << BODY_OUT_VL53_L) |
                         (1u << BODY_OUT_VL53_R));
constexpr uint8_t kBodyAllOutMask =
    static_cast<uint8_t>(kBodyEmitterOutMask | kBodyVl53OutMask);
constexpr uint8_t kBodyIodir =
    static_cast<uint8_t>(0xFFu & static_cast<uint8_t>(~kBodyAllOutMask));
static_assert(BODY_IN_SPARE == 0 && BODY_IN_WHEEL_L == 1 && BODY_IN_WHEEL_R == 2);
static_assert(BODY_OUT_VL53_L8 == 3 && BODY_OUT_WHEEL_L == 4 && BODY_OUT_WHEEL_R == 5);
static_assert(BODY_OUT_VL53_L == 6 && BODY_OUT_VL53_R == 7);
static_assert(kBodySchmittInMask == 0x07, "GPA0-2 inputs");
static_assert(kBodyEmitterOutMask == 0x30, "GPA4-5 PNP wheel emitters");
static_assert(kBodyVl53OutMask == 0xC8, "GPA3 L8 LPn + GPA6-7 XSHUT");
static_assert(kBodyIodir == 0x07, "IODIR: in 0-2, out 3-7");

}  // namespace

bool Mcp23008::writeReg(uint8_t reg, uint8_t val) {
  RoverI2cGuard guard;
  rover_i2c_touch();
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool Mcp23008::readReg(uint8_t reg, uint8_t &val) const {
  RoverI2cGuard guard;
  rover_i2c_touch();
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(static_cast<int>(_addr), 1) != 1) return false;
  val = static_cast<uint8_t>(Wire.read());
  return true;
}

bool Mcp23008::begin(uint8_t addr) {
  _addr = addr;
  _iodir = 0xFF;
  _olat = 0x00;
  if (!writeReg(kRegIodir, _iodir)) {
    _addr = 0;
    return false;
  }
  return writeReg(kRegOlat, _olat);
}

void Mcp23008::pinMode(uint8_t pin, bool output) {
  if (!_addr || pin > 7) return;
  if (output) {
    _iodir &= static_cast<uint8_t>(~(1u << pin));
  } else {
    _iodir |= static_cast<uint8_t>(1u << pin);
  }
  writeReg(kRegIodir, _iodir);
}

bool Mcp23008::configureIo(uint8_t iodir, uint8_t olat) {
  if (!_addr) return false;
  _iodir = iodir;
  _olat = olat;
  if (!writeReg(kRegOlat, _olat)) return false;
  return writeReg(kRegIodir, _iodir);
}

bool Mcp23008::writeOutputLatch(uint8_t olat) {
  if (!_addr) return false;
  _olat = olat;
  return writeReg(kRegOlat, _olat);
}

bool Mcp23008::readIoState(uint8_t &iodir, uint8_t &olat, uint8_t &gpio) const {
  iodir = 0;
  olat = 0;
  gpio = 0;
  if (!_addr) return false;
  if (!readReg(kRegIodir, iodir)) return false;
  if (!readReg(kRegOlat, olat)) return false;
  return readReg(kRegGpio, gpio);
}

bool Mcp23008::readPin(uint8_t pin) const {
  if (!_addr || pin > 7) return false;
  uint8_t gpio = 0;
  if (!readReg(kRegGpio, gpio)) return false;
  return (gpio & (1u << pin)) != 0;
}

bool RoverMcpIr::beginBody() {
  if (!_body.ok()) {
    if (!_body.begin(MCP_BODY_ADDR)) return false;
    _body_out_shadow =
        static_cast<uint8_t>(kBodyPnpLowMask | kBodySchmittInMask);
    if (!_body.configureIo(kBodyIodir, _body_out_shadow)) return false;
  }
  setBodyEmitter(BODY_OUT_WHEEL_L, true);
  setBodyEmitter(BODY_OUT_WHEEL_R, true);
  set_vl53l8_lpn(false);
  applyBodyOutputs();
  _wheels_on = true;
  return true;
}

bool RoverMcpIr::begin(int sda, int scl) {
  rover_i2c_begin(sda, scl);
  delay(50);
  return beginBody();
}

void RoverMcpIr::applyBodyOutputs() {
  if (!_body.ok()) return;
  _body_out_shadow |= kBodySchmittInMask;
  _body.configureIo(kBodyIodir, _body_out_shadow);
}

void RoverMcpIr::readBodyIoState(uint8_t &iodir, uint8_t &olat, uint8_t &gpio) const {
  iodir = 0;
  olat = 0;
  gpio = 0;
  if (!_body.ok()) return;
  _body.readIoState(iodir, olat, gpio);
}

void RoverMcpIr::logBodyIoState() const {
  if (!_body.ok()) {
    Serial.println("MCP body: not present");
    return;
  }
  uint8_t iodir = 0;
  uint8_t olat = 0;
  uint8_t gpio = 0;
  if (!_body.readIoState(iodir, olat, gpio)) {
    Serial.printf("MCP body @0x%02x: readback failed\n", _body.addr());
    return;
  }
  Serial.printf(
      "MCP body @0x%02x: IODIR=0x%02x OLAT=0x%02x GPIO=0x%02x want IODIR=0x%02x OLAT=0x%02x "
      "(GPA3=L8 LPn, GPA4-5 PNP wheels)\n",
      _body.addr(), iodir, olat, gpio, kBodyIodir, static_cast<unsigned>(_body_out_shadow));
}

void RoverMcpIr::setBodyEmitter(uint8_t pin, bool emitter_on) {
  if (!_body.ok() || pin > 7) return;
  if ((kBodyPnpLowMask & (1u << pin)) == 0) return;
  const bool pin_high = !emitter_on;
  if (pin_high) {
    _body_out_shadow |= static_cast<uint8_t>(1u << pin);
  } else {
    _body_out_shadow &= static_cast<uint8_t>(~(1u << pin));
  }
  _body_out_shadow |= kBodySchmittInMask;
}

bool RoverMcpIr::readBodyInput(uint8_t pin) const {
  return _body.readPin(pin);
}

void RoverMcpIr::set_wheels_enabled(bool on) {
  if (!_body.ok()) return;
  _wheels_on = on;
  setBodyEmitter(BODY_OUT_WHEEL_L, true);
  setBodyEmitter(BODY_OUT_WHEEL_R, true);
  applyBodyOutputs();
}

uint8_t RoverMcpIr::read_inputs() const {
  uint8_t mask = 0;
  if (_body.ok()) {
    if (readBodyInput(BODY_IN_WHEEL_L)) mask |= 0x01;
    if (readBodyInput(BODY_IN_WHEEL_R)) mask |= 0x02;
  }
  return mask;
}

void RoverMcpIr::tick(uint32_t now_ms) {
  (void)now_ms;
  if (!_body.ok()) return;

  const bool wl = readBodyInput(BODY_IN_WHEEL_L);
  const bool wr = readBodyInput(BODY_IN_WHEEL_R);
  if (wl && !_wheel_l_prev) {
    _wheel_l_ticks++;
  }
  if (wr && !_wheel_r_prev) {
    _wheel_r_ticks++;
  }
  _wheel_l_prev = wl;
  _wheel_r_prev = wr;
}

void RoverMcpIr::set_vl53_xshut(bool left_on, bool right_on) {
  if (!_body.ok()) return;
  if (left_on) {
    _body_out_shadow |= static_cast<uint8_t>(1u << BODY_OUT_VL53_L);
  } else {
    _body_out_shadow &= static_cast<uint8_t>(~(1u << BODY_OUT_VL53_L));
  }
  if (right_on) {
    _body_out_shadow |= static_cast<uint8_t>(1u << BODY_OUT_VL53_R);
  } else {
    _body_out_shadow &= static_cast<uint8_t>(~(1u << BODY_OUT_VL53_R));
  }
  applyBodyOutputs();
}

void RoverMcpIr::set_vl53l8_lpn(bool on) {
  if (!_body.ok()) return;
  if (on) {
    _body_out_shadow |= static_cast<uint8_t>(1u << BODY_OUT_VL53_L8);
  } else {
    _body_out_shadow &= static_cast<uint8_t>(~(1u << BODY_OUT_VL53_L8));
  }
  applyBodyOutputs();
}

I2cScanResult scanI2cBus() {
  RoverI2cGuard guard;
  I2cScanResult result;
  for (uint8_t addr = 0x08; addr <= 0x77 && result.count < 8; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      result.addrs[result.count++] = addr;
    }
  }
  return result;
}
