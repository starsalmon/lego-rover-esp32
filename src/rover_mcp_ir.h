#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "rover_pins_s3.h"

// Aux delta sample — blocks the main loop for off_ms + on_ms each request.
// IR LEDs need ~40 ms before output is useful; was wrongly shortened to 25 ms on MCP bring-up.
constexpr uint16_t MCP_IR_AUX_OFF_MS = 10;
constexpr uint16_t MCP_IR_AUX_ON_MS = 100;
// On-demand front gate test (diagnostic only — normal run keeps gates ON).
constexpr uint16_t MCP_IR_FRONT_SAMPLE_OFF_MS = 50;
constexpr uint16_t MCP_IR_FRONT_SAMPLE_ON_MS = 10;

// Aux emitter is pulsed for radar delta sampling. Wheel emitters stay on for odometer.

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

// Two MCP23008 expanders on the I2C bus (see rover_pins_s3.h):
//   Front bumper board — front L/R detect + BC547 gates (555 carrier, gates stay ON)
//   ESP breadboard     — aux + wheel detect + emitter gates
class RoverMcpIr {
 public:
  bool begin(int sda, int scl);
  bool ok() const { return _front.ok() || _body.ok(); }
  bool front_ok() const { return _front.ok(); }
  bool body_ok() const { return _body.ok(); }

  void tick(uint32_t now_ms);

  /** Blocking delta sample — rear reflective obstacle (aux emitter). */
  bool rear_obstacle();

  bool front_hit() const { return _front_hit_l || _front_hit_r; }
  bool front_left_hit() const { return _front_hit_l; }
  bool front_right_hit() const { return _front_hit_r; }

  void set_wheels_enabled(bool on);
  bool wheels_enabled() const { return _wheels_on; }
  bool front_emitting() const;
  bool body_aux_emitting() const;
  void readFrontIoState(uint8_t &iodir, uint8_t &olat, uint8_t &gpio) const;
  void readBodyIoState(uint8_t &iodir, uint8_t &olat, uint8_t &gpio) const;
  void logFrontIoState() const;
  void logBodyIoState() const;

  // Bitmask for display: FL=0 FR=1 WL=2 WR=3 AX=4
  uint8_t read_inputs() const;

  void sample_aux(uint16_t off_ms, uint16_t on_ms, bool &off_hit, bool &on_hit);
  void sample_front(uint16_t off_ms, uint16_t on_ms, bool &off_hit, bool &on_hit);

  bool front_off_hit() const { return _front_hit_l || _front_hit_r; }
  bool front_on_hit() const { return _front_hit_l || _front_hit_r; }

  uint32_t wheel_left_ticks() const { return _wheel_l_ticks; }
  uint32_t wheel_right_ticks() const { return _wheel_r_ticks; }
  void reset_wheel_ticks() { _wheel_l_ticks = 0; _wheel_r_ticks = 0; }

 private:
  // Body emitters @ GPA3-5: BC557 PNP gates — MCP LOW = on, MCP HIGH = off.
  static constexpr uint8_t kBodyPnpLowMask =
      static_cast<uint8_t>((1u << BODY_OUT_AUX) | (1u << BODY_OUT_WHEEL_L) |
                           (1u << BODY_OUT_WHEEL_R));

  bool beginFront();
  bool beginBody();
  void applyFrontOutputs();
  void applyBodyOutputs();
  void setFrontEmitters(bool on);
  void setAllBodyEmitters(bool on);
  void setBodyEmitter(uint8_t pin, bool on);
  bool readFrontInput(uint8_t pin) const;
  bool readBodyInput(uint8_t pin) const;

  Mcp23008 _front;
  Mcp23008 _body;
  uint8_t _front_out_shadow = 0;
  uint8_t _body_out_shadow = 0;
  bool _front_emitters_on = false;
  bool _wheels_on = false;

  bool _front_hit_l = false;
  bool _front_hit_r = false;

  bool _wheel_l_prev = false;
  bool _wheel_r_prev = false;
  uint32_t _wheel_l_ticks = 0;
  uint32_t _wheel_r_ticks = 0;
};
