#include "rover_mcp_ir.h"

#include <Wire.h>

#include "rover_i2c.h"
#include "rover_pins_s3.h"

namespace {

constexpr uint8_t kRegIodir = 0x00;
constexpr uint8_t kRegOlat = 0x0A;  // MCP23008 OLAT — NOT 0x05 (that is IOCON)
constexpr uint8_t kRegGpio = 0x09;

// Front @ 0x20: GPA0-1 in, GPA2-3 out (BC547 gates), GPA4-7 spare inputs.
constexpr uint8_t kFrontIodir = 0xF3;
// Body @ 0x21 — GPA0-2 Schmitt INPUTS, GPA3-5 BC557 emitter OUTPUTS.
// MCP IODIR: 1=input. Only clear bits 3-5 in IODIR (outputs); 0-2,6-7 stay input.
constexpr uint8_t kBodySchmittInMask =
    static_cast<uint8_t>((1u << BODY_IN_AUX) | (1u << BODY_IN_WHEEL_L) |
                         (1u << BODY_IN_WHEEL_R));
constexpr uint8_t kBodyEmitterOutMask =
    static_cast<uint8_t>((1u << BODY_OUT_AUX) | (1u << BODY_OUT_WHEEL_L) |
                         (1u << BODY_OUT_WHEEL_R));
constexpr uint8_t kBodyIodir =
    static_cast<uint8_t>(0xFFu & static_cast<uint8_t>(~kBodyEmitterOutMask));
static_assert(BODY_IN_AUX == 0 && BODY_IN_WHEEL_L == 1 && BODY_IN_WHEEL_R == 2);
static_assert(BODY_OUT_AUX == 3 && BODY_OUT_WHEEL_L == 4 && BODY_OUT_WHEEL_R == 5);
static_assert(kBodySchmittInMask == 0x07, "GPA0-2 inputs");
static_assert(kBodyEmitterOutMask == 0x38, "GPA3-5 outputs");
static_assert(kBodyIodir == 0xC7, "IODIR: in 0-2,6-7 out 3-5");

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
  // Set latch before direction so outputs do not glitch low when enabled.
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

bool RoverMcpIr::beginFront() {
  if (!_front.ok()) {
    if (!_front.begin(MCP_FRONT_ADDR)) return false;
    if (!_front.configureIo(kFrontIodir, 0x00)) return false;
  }
  setFrontEmitters(true);
  return true;
}

bool RoverMcpIr::beginBody() {
  if (!_body.ok()) {
    if (!_body.begin(MCP_BODY_ADDR)) return false;
    // Emitters off (GPA3-5 high); never latch-low GPA0-2 Schmitt inputs.
    _body_out_shadow =
        static_cast<uint8_t>(kBodyPnpLowMask | kBodySchmittInMask);  // 0x3F
    if (!_body.configureIo(kBodyIodir, _body_out_shadow)) return false;
  }
  // Wheel IR always on (odometer); aux off until sample_aux() pulses it.
  setBodyEmitter(BODY_OUT_WHEEL_L, true);
  setBodyEmitter(BODY_OUT_WHEEL_R, true);
  setBodyEmitter(BODY_OUT_AUX, false);
  applyBodyOutputs();
  _wheels_on = true;
  return true;
}

bool RoverMcpIr::begin(int sda, int scl) {
  rover_i2c_begin(sda, scl);
  delay(50);

  const bool front = beginFront();
  const bool body = beginBody();
  return front || body;
}

void RoverMcpIr::applyFrontOutputs() {
  if (!_front.ok()) return;
  _front.configureIo(kFrontIodir, _front_out_shadow);
}

void RoverMcpIr::applyBodyOutputs() {
  if (!_body.ok()) return;
  // OLAT bits 0-2 are inputs — keep them high; only GPA3-5 drive emitters.
  _body_out_shadow |= kBodySchmittInMask;
  _body.configureIo(kBodyIodir, _body_out_shadow);
}

void RoverMcpIr::readFrontIoState(uint8_t &iodir, uint8_t &olat, uint8_t &gpio) const {
  iodir = 0;
  olat = 0;
  gpio = 0;
  if (!_front.ok()) return;
  _front.readIoState(iodir, olat, gpio);
}

bool RoverMcpIr::front_emitting() const {
  if (!_front.ok() || !_front_emitters_on) return false;
  uint8_t iodir = 0;
  uint8_t olat = 0;
  uint8_t gpio = 0;
  if (!_front.readIoState(iodir, olat, gpio)) return false;
  const uint8_t out_mask =
      static_cast<uint8_t>((1u << FRONT_OUT_L) | (1u << FRONT_OUT_R));
  if ((iodir & out_mask) != 0) return false;
  return (gpio & out_mask) == out_mask;
}

bool RoverMcpIr::body_aux_emitting() const {
  if (!_body.ok()) return false;
  uint8_t iodir = 0;
  uint8_t olat = 0;
  uint8_t gpio = 0;
  if (!_body.readIoState(iodir, olat, gpio)) return false;
  const uint8_t mask = static_cast<uint8_t>(1u << BODY_OUT_AUX);
  if ((iodir & mask) != 0) return false;
  // PNP / active-low gate: MCP pin LOW = emitter on.
  return (gpio & mask) == 0;
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
      "MCP body @0x%02x: IODIR=0x%02x OLAT=0x%02x GPIO=0x%02x want IODIR=0x%02x OLAT=0x%02x (aux "
      "BC557 PNP-low)\n",
      _body.addr(), iodir, olat, gpio, kBodyIodir, static_cast<unsigned>(_body_out_shadow));
}

void RoverMcpIr::logFrontIoState() const {
  if (!_front.ok()) {
    Serial.println("MCP front: not present");
    return;
  }
  uint8_t iodir = 0;
  uint8_t olat = 0;
  uint8_t gpio = 0;
  if (!_front.readIoState(iodir, olat, gpio)) {
    Serial.printf("MCP front @0x%02x: readback failed\n", _front.addr());
    return;
  }
  Serial.printf("MCP front @0x%02x: IODIR=0x%02x OLAT=0x%02x GPIO=0x%02x want IODIR=0x%02x OLAT=0x%02x\n",
                _front.addr(), iodir, olat, gpio, kFrontIodir,
                static_cast<unsigned>(_front_out_shadow));
}

void RoverMcpIr::setFrontEmitters(bool on) {
  if (!_front.ok()) return;
  _front_emitters_on = on;
  if (on) {
    _front_out_shadow |= static_cast<uint8_t>((1u << FRONT_OUT_L) | (1u << FRONT_OUT_R));
  } else {
    _front_out_shadow &= static_cast<uint8_t>(~((1u << FRONT_OUT_L) | (1u << FRONT_OUT_R)));
  }
  applyFrontOutputs();
}

void RoverMcpIr::setBodyEmitter(uint8_t pin, bool emitter_on) {
  if (!_body.ok() || pin > 7) return;
  if ((kBodyPnpLowMask & (1u << pin)) == 0) return;
  const bool pin_high = !emitter_on;  // BC557 PNP: MCP low = emitter on
  if (pin_high) {
    _body_out_shadow |= static_cast<uint8_t>(1u << pin);
  } else {
    _body_out_shadow &= static_cast<uint8_t>(~(1u << pin));
  }
  _body_out_shadow |= kBodySchmittInMask;
}

void RoverMcpIr::setAllBodyEmitters(bool on) {
  if (!_body.ok()) return;
  if (on) {
    // GPA3-5 low (emitters on), GPA0-2 untouched in latch (stay high).
    _body_out_shadow = kBodySchmittInMask;  // 0x07
  } else {
    _body_out_shadow =
        static_cast<uint8_t>(kBodySchmittInMask | kBodyPnpLowMask);  // 0x3F
  }
  applyBodyOutputs();
}

bool RoverMcpIr::readFrontInput(uint8_t pin) const {
  return _front.readPin(pin);
}

bool RoverMcpIr::readBodyInput(uint8_t pin) const {
  return _body.readPin(pin);
}

void RoverMcpIr::set_wheels_enabled(bool on) {
  if (!_body.ok()) return;
  _wheels_on = on;
  // Keep wheel emitters on — pulsing them breaks tick counting.
  setBodyEmitter(BODY_OUT_WHEEL_L, true);
  setBodyEmitter(BODY_OUT_WHEEL_R, true);
  applyBodyOutputs();
}

uint8_t RoverMcpIr::read_inputs() const {
  uint8_t mask = 0;
  if (_front.ok()) {
    if (readFrontInput(FRONT_IN_L)) mask |= 0x01;
    if (readFrontInput(FRONT_IN_R)) mask |= 0x02;
  }
  if (_body.ok()) {
    if (readBodyInput(BODY_IN_WHEEL_L)) mask |= 0x04;
    if (readBodyInput(BODY_IN_WHEEL_R)) mask |= 0x08;
    if (readBodyInput(BODY_IN_AUX)) mask |= 0x10;
  }
  return mask;
}

void RoverMcpIr::tick(uint32_t now_ms) {
  if (_front.ok()) {
    static uint32_t last_sample_ms = 0;
    if (now_ms - last_sample_ms >= 250) {
      last_sample_ms = now_ms;
      bool off_hit = false;
      bool on_hit = false;
      sample_front(MCP_IR_FRONT_SAMPLE_OFF_MS, MCP_IR_FRONT_SAMPLE_ON_MS, off_hit, on_hit);
    }
  } else {
    _front_hit_l = false;
    _front_hit_r = false;
  }

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

  // Aux should be off between samples; wheels stay on.
  if (body_aux_emitting()) {
    setBodyEmitter(BODY_OUT_AUX, false);
    applyBodyOutputs();
  }
}

void RoverMcpIr::sample_front(uint16_t off_ms, uint16_t on_ms, bool &off_hit, bool &on_hit) {
  off_hit = false;
  on_hit = false;
  _front_hit_l = false;
  _front_hit_r = false;
  if (!_front.ok()) return;

  setFrontEmitters(false);
  delay(off_ms);
  const bool off_l = readFrontInput(FRONT_IN_L);
  const bool off_r = readFrontInput(FRONT_IN_R);
  off_hit = off_l || off_r;

  setFrontEmitters(true);
  delay(on_ms);
  const bool on_l = readFrontInput(FRONT_IN_L);
  const bool on_r = readFrontInput(FRONT_IN_R);
  on_hit = on_l || on_r;

  // Reflective bumper — trust delta (on & !off), not ambient / stray 38 kHz.
  _front_hit_l = on_l && !off_l;
  _front_hit_r = on_r && !off_r;
}

bool RoverMcpIr::rear_obstacle() {
  bool off_hit = false;
  bool on_hit = false;
  sample_aux(MCP_IR_AUX_OFF_MS, MCP_IR_AUX_ON_MS, off_hit, on_hit);
  return on_hit && !off_hit;
}

void RoverMcpIr::sample_aux(uint16_t off_ms, uint16_t on_ms, bool &off_hit, bool &on_hit) {
  off_hit = false;
  on_hit = false;
  if (!_body.ok()) return;

  setBodyEmitter(BODY_OUT_AUX, false);
  applyBodyOutputs();
  delay(off_ms);
  off_hit = readBodyInput(BODY_IN_AUX);

  setBodyEmitter(BODY_OUT_AUX, true);
  applyBodyOutputs();
  delay(on_ms);
  on_hit = readBodyInput(BODY_IN_AUX);

  setBodyEmitter(BODY_OUT_AUX, false);
  applyBodyOutputs();
}

I2cScanResult scanI2cBus() {
  I2cScanResult result;
  for (uint8_t addr = 0x08; addr <= 0x77 && result.count < 8; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      result.addrs[result.count++] = addr;
    }
  }
  return result;
}
