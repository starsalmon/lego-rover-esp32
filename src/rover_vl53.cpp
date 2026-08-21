#include "rover_vl53.h"

#include <Arduino.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <Wire.h>

#include "rover_diag.h"
#include "rover_i2c.h"
#include "rover_mcp_ir.h"
#include "rover_pins_s3.h"

#ifndef VL53L_ADDR_L
#define VL53L_ADDR_L 0x30
#endif
#ifndef VL53L_ADDR_R
#define VL53L_ADDR_R 0x31
#endif
#ifndef VL53L_TIMING_BUDGET_US
#define VL53L_TIMING_BUDGET_US 20000
#endif
#ifndef VL53L_CONTINUOUS_MS
#define VL53L_CONTINUOUS_MS 50
#endif

enum class Vl53Chip : uint8_t { kNone, kL1x, kL0x };

static Vl53Chip g_chip = Vl53Chip::kNone;
static VL53L1X g_l1_left;
static VL53L1X g_l1_right;
static VL53L0X g_l0_left;
static VL53L0X g_l0_right;

static bool i2c_present(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static float mm_to_m(uint16_t mm) {
  if (mm == 0) {
    return -1.0f;
  }
  return static_cast<float>(mm) / 1000.0f;
}

/** L0X max ~2 m; larger values are "no target" / crosstalk garbage (e.g. 8190 mm). */
static bool l0x_mm_valid(uint16_t mm) {
  return mm >= 40 && mm <= 2000;
}

static uint16_t read_l0x_mm(VL53L0X& sensor) {
  const uint16_t mm = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred() || !l0x_mm_valid(mm)) {
    return 0;
  }
  return mm;
}

static uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) {
    const uint16_t t = a;
    a = b;
    b = t;
  }
  if (b > c) {
    const uint16_t t = b;
    b = c;
    c = t;
  }
  if (a > b) {
    const uint16_t t = a;
    a = b;
    b = t;
  }
  return b;
}

static float l0x_filtered_m(VL53L0X& sensor, uint16_t hist[3], uint8_t& hist_idx, float& last_m,
                            uint8_t& invalid_n) {
  // IMPORTANT: when the target moves far away, L0X can return invalids for long
  // stretches. If we keep the last valid value forever, the UI (and brain, via
  // ROS) will think we're still near a wall and steer badly.
  const uint16_t mm = read_l0x_mm(sensor);
  if (mm == 0) {
    if (invalid_n < 255) invalid_n++;
    if (invalid_n >= 4) {  // ~200 ms at 50 ms continuous
      last_m = -1.0f;
    }
    return last_m;
  }
  invalid_n = 0;
  hist[hist_idx++ % 3] = mm;
  const uint16_t med = median3(hist[0], hist[1], hist[2]);
  last_m = l0x_mm_valid(med) ? mm_to_m(med) : -1.0f;
  return last_m;
}

static bool setup_one_l1x(VL53L1X& sensor, uint8_t addr) {
  sensor.setTimeout(500);
  if (!sensor.init()) {
    return false;
  }
  sensor.setAddress(addr);
  sensor.setDistanceMode(VL53L1X::Short);
  sensor.setMeasurementTimingBudget(VL53L_TIMING_BUDGET_US);
  sensor.startContinuous(VL53L_CONTINUOUS_MS);
  delay(60);
  return true;
}

static bool setup_one_l0x(VL53L0X& sensor, uint8_t addr) {
  sensor.setTimeout(500);
  if (!sensor.init()) {
    return false;
  }
  sensor.setAddress(addr);
  sensor.startContinuous(VL53L_CONTINUOUS_MS);
  delay(60);
  return true;
}

static bool try_single_l1x(bool* out_ok) {
  *out_ok = false;
  if (!i2c_present(0x29)) {
    return false;
  }
  *out_ok = setup_one_l1x(g_l1_left, VL53L_ADDR_L);
  return *out_ok;
}

static bool try_single_l0x(bool* out_ok) {
  *out_ok = false;
  if (!i2c_present(0x29)) {
    return false;
  }
  *out_ok = setup_one_l0x(g_l0_left, VL53L_ADDR_L);
  return *out_ok;
}

static bool begin_chip_l1x(RoverMcpIr* mcp, bool* left_ok, bool* right_ok) {
  *left_ok = false;
  *right_ok = false;

  if (mcp) {
    mcp->set_vl53_xshut(true, false);
    delay(20);
    {
      RoverI2cGuard guard;
      *left_ok = setup_one_l1x(g_l1_left, VL53L_ADDR_L);
    }
    mcp->set_vl53_xshut(true, true);
    delay(20);
    {
      RoverI2cGuard guard;
      *right_ok = setup_one_l1x(g_l1_right, VL53L_ADDR_R);
    }
  }
  if (!*left_ok && !*right_ok) {
    RoverI2cGuard guard;
    if (try_single_l1x(left_ok)) {
      Serial.println("VL53L1X: single-sensor fallback (XSHUT always high?)");
    }
  }
  return *left_ok || *right_ok;
}

static bool begin_chip_l0x(RoverMcpIr* mcp, bool* left_ok, bool* right_ok) {
  *left_ok = false;
  *right_ok = false;

  if (mcp) {
    mcp->set_vl53_xshut(true, false);
    delay(20);
    {
      RoverI2cGuard guard;
      *left_ok = setup_one_l0x(g_l0_left, VL53L_ADDR_L);
    }
    mcp->set_vl53_xshut(true, true);
    delay(20);
    {
      RoverI2cGuard guard;
      *right_ok = setup_one_l0x(g_l0_right, VL53L_ADDR_R);
    }
  }
  if (!*left_ok && !*right_ok) {
    RoverI2cGuard guard;
    if (try_single_l0x(left_ok)) {
      Serial.println("VL53L0X: single-sensor fallback (XSHUT always high?)");
    }
  }
  return *left_ok || *right_ok;
}

void RoverVl53::set_xshut(bool left_on, bool right_on) {
  if (_mcp) {
    _mcp->set_vl53_xshut(left_on, right_on);
  }
}

bool RoverVl53::begin(RoverMcpIr* mcp) {
  g_chip = Vl53Chip::kNone;
  _ok = false;
  _left_ok = false;
  _right_ok = false;
  _mcp = mcp;
  _left_m = -1.0f;
  _right_m = -1.0f;

  set_xshut(false, false);
  delay(20);

  bool p29 = false;
  {
    RoverI2cGuard guard;
    p29 = i2c_present(0x29);
    const bool p30 = i2c_present(0x30);
    const bool p31 = i2c_present(0x31);
    Serial.printf("VL53 I2C pre-init: 0x29=%s 0x30=%s 0x31=%s\n", p29 ? "yes" : "no",
                  p30 ? "yes" : "no", p31 ? "yes" : "no");
  }

  if (begin_chip_l1x(_mcp, &_left_ok, &_right_ok)) {
    g_chip = Vl53Chip::kL1x;
  } else {
    set_xshut(false, false);
    delay(20);
    if (begin_chip_l0x(_mcp, &_left_ok, &_right_ok)) {
      g_chip = Vl53Chip::kL0x;
      Serial.println("VL53: using VL53L0X driver (L1X init failed)");
    }
  }

  if (_left_ok || _right_ok) {
    set_xshut(_left_ok, _right_ok);
  }

  _ok = g_chip != Vl53Chip::kNone;
  const char* chip =
      g_chip == Vl53Chip::kL1x ? "L1X" : (g_chip == Vl53Chip::kL0x ? "L0X" : "none");
  Serial.printf("VL53 %s left=%s right=%s xshut MCP GPA%d/%d (addr 0x%02x/0x%02x)\n", chip,
                _left_ok ? "OK" : "--", _right_ok ? "OK" : "--", BODY_OUT_VL53_L, BODY_OUT_VL53_R,
                VL53L_ADDR_L, VL53L_ADDR_R);
  {
    RoverI2cGuard guard;
    rover_diag_event("vl53 %s L=%s R=%s i2c29=%d", chip, _left_ok ? "OK" : "--",
                     _right_ok ? "OK" : "--", i2c_present(0x29) ? 1 : 0);
  }
  return _ok;
}

void RoverVl53::poll() {
  if (!_ok) {
    return;
  }
  RoverI2cGuard guard;
  if (g_chip == Vl53Chip::kL1x) {
    if (_left_ok) {
      _left_m = mm_to_m(g_l1_left.read(false));
    }
    if (_right_ok) {
      _right_m = mm_to_m(g_l1_right.read(false));
    }
  } else if (g_chip == Vl53Chip::kL0x) {
    static uint16_t l_hist[3] = {};
    static uint16_t r_hist[3] = {};
    static uint8_t l_idx = 0;
    static uint8_t r_idx = 0;
    static uint8_t l_invalid = 0;
    static uint8_t r_invalid = 0;
    static float l_last = -1.0f;
    static float r_last = -1.0f;
    if (_left_ok) {
      _left_m = l0x_filtered_m(g_l0_left, l_hist, l_idx, l_last, l_invalid);
    }
    if (_right_ok) {
      _right_m = l0x_filtered_m(g_l0_right, r_hist, r_idx, r_last, r_invalid);
    }
  }
}
