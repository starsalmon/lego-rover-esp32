#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "rover_pins_s3.h"

struct I2cScanResult {
  uint8_t addrs[8] = {};
  uint8_t count = 0;
};

I2cScanResult scanI2cBus();

class Mcp23008 {
 public:
  bool begin(uint8_t addr);
  bool ok() const { return _addr != 0; }
  uint8_t addr() const { return _addr; }

  void pinMode(uint8_t pin, bool output);
  bool configureIo(uint8_t iodir, uint8_t olat);
  bool writeOutputLatch(uint8_t olat);

  bool readPin(uint8_t pin) const;
  bool readIoState(uint8_t &iodir, uint8_t &olat, uint8_t &gpio) const;

 private:
  bool writeReg(uint8_t reg, uint8_t val);
  bool readReg(uint8_t reg, uint8_t &val) const;

  uint8_t _addr = 0;
  uint8_t _iodir = 0xFF;
  uint8_t _olat = 0x00;
};

// Body MCP23008 @ 0x21: wheel IR + VL53 XSHUT / L8 LPn.
class RoverMcpIr {
 public:
  bool begin(int sda, int scl);
  bool ok() const { return _body.ok(); }
  bool body_ok() const { return _body.ok(); }

  void tick(uint32_t now_ms);

  void set_wheels_enabled(bool on);
  bool wheels_enabled() const { return _wheels_on; }
  void readBodyIoState(uint8_t &iodir, uint8_t &olat, uint8_t &gpio) const;
  void logBodyIoState() const;

  // Bitmask: WL=0 WR=1
  uint8_t read_inputs() const;

  void set_vl53_xshut(bool left_on, bool right_on);
  void set_vl53l8_lpn(bool on);

  uint32_t wheel_left_ticks() const { return _wheel_l_ticks; }
  uint32_t wheel_right_ticks() const { return _wheel_r_ticks; }
  void reset_wheel_ticks() { _wheel_l_ticks = 0; _wheel_r_ticks = 0; }

  // TSOP on GPA0 — LOW at the receiver, HIGH at MCP when idle.
  bool tsop_active() const;

 private:
  static constexpr uint8_t kBodyPnpLowMask =
      static_cast<uint8_t>((1u << BODY_OUT_WHEEL_L) | (1u << BODY_OUT_WHEEL_R));

  bool beginBody();
  void applyBodyOutputs();
  void setBodyEmitter(uint8_t pin, bool on);
  bool readBodyInput(uint8_t pin) const;

  Mcp23008 _body;
  uint8_t _body_out_shadow = 0;
  bool _wheels_on = false;

  bool _wheel_l_prev = false;
  bool _wheel_r_prev = false;
  uint32_t _wheel_l_ticks = 0;
  uint32_t _wheel_r_ticks = 0;
};
