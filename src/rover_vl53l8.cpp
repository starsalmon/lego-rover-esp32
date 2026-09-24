#include "rover_vl53l8.h"

#include <math.h>

#include <vl53l8cx.h>

#include "rover_i2c.h"
#include "rover_pins_s3.h"

#ifndef L8_STOP_M
#define L8_STOP_M 0.24f
#endif
#ifndef ROVER_L8_SWAP_LR
#define ROVER_L8_SWAP_LR 1
#endif

namespace {

VL53L8CX g_sensor(&Wire, -1);
VL53L8CX_ResultsData g_results;

bool zone_ok(const VL53L8CX_ResultsData &d, uint8_t z) {
  if (z >= VL53L8CX_RESOLUTION_8X8) {
    return false;
  }
  if (d.nb_target_detected[z] < 1) {
    return false;
  }
  const uint8_t st = d.target_status[VL53L8CX_NB_TARGET_PER_ZONE * z];
  return st == 5 || st == 6 || st == 9;
}

float zone_m(const VL53L8CX_ResultsData &d, uint8_t z) {
  const uint16_t mm = d.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * z];
  if (mm < 20 || mm > 4000) {
    return -1.0f;
  }
  return static_cast<float>(mm) * 0.001f;
}

}  // namespace

bool RoverVl53L8::begin() {
  _ok = false;
  _center_m = -1.0f;
  _left_m = -1.0f;
  _right_m = -1.0f;
  for (uint8_t i = 0; i < kCols; i++) {
    _col_m[i] = -1.0f;
  }

  uint8_t alive = 0;
  {
    RoverI2cGuard guard;
    // Firmware load wants 400 kHz. Do not call rover_i2c_touch() here — it
    // slams the timeout back to 50 ms under the download.
    Wire.setClock(400000);
    Wire.setTimeOut(400);
    g_sensor.begin();
    if (g_sensor.is_alive(&alive) != 0 || alive == 0) {
      Serial.println("VL53L8CX not alive at 0x29");
      Wire.setClock(ROVER_I2C_HZ);
      Wire.setTimeOut(ROVER_I2C_TIMEOUT_MS);
      return false;
    }
    Serial.println("VL53L8CX init (firmware load, a few seconds)...");
    const uint8_t st = g_sensor.init();
    if (st != 0) {
      Serial.printf("VL53L8CX init failed status=%u\n", (unsigned)st);
      Wire.setClock(ROVER_I2C_HZ);
      Wire.setTimeOut(ROVER_I2C_TIMEOUT_MS);
      return false;
    }
    g_sensor.set_resolution(VL53L8CX_RESOLUTION_8X8);
    g_sensor.set_ranging_frequency_hz(10);
    g_sensor.set_ranging_mode(VL53L8CX_RANGING_MODE_CONTINUOUS);
    if (g_sensor.start_ranging() != 0) {
      Serial.println("VL53L8CX start_ranging failed");
      Wire.setClock(ROVER_I2C_HZ);
      Wire.setTimeOut(ROVER_I2C_TIMEOUT_MS);
      return false;
    }
    Wire.setClock(ROVER_I2C_HZ);
    Wire.setTimeOut(ROVER_I2C_TIMEOUT_MS);
  }

  _ok = true;
  Serial.println("VL53L8CX 8x8 ranging");
  return true;
}

void RoverVl53L8::poll() {
  if (!_ok) {
    return;
  }
  uint8_t ready = 0;
  RoverI2cGuard guard;
  // Keep the bus at 100 kHz. Only relax the timeout so an 8×8 dump can finish.
  Wire.setTimeOut(200);
  const uint8_t ready_st = g_sensor.check_data_ready(&ready);
  if (ready_st != 0 || ready == 0) {
    Wire.setTimeOut(ROVER_I2C_TIMEOUT_MS);
    return;
  }
  const uint8_t data_st = g_sensor.get_ranging_data(&g_results);
  Wire.setTimeOut(ROVER_I2C_TIMEOUT_MS);
  if (data_st != 0) {
    return;
  }

  // Row 0 = top, row 7 = bottom (floor). Only rows 0–4 are the forward cone;
  // lower rows are carpet and were making columns flip between "open" (3.5 m)
  // and a wall hit every frame.
  constexpr uint8_t kObstRowMax = 4;
  float grid[8][8];
  for (uint8_t row = 0; row < 8; row++) {
    for (uint8_t col = 0; col < 8; col++) {
      grid[row][col] = -1.0f;
      const uint8_t z = static_cast<uint8_t>(row * 8 + col);
      if (!zone_ok(g_results, z)) {
        continue;
      }
      grid[row][col] = zone_m(g_results, z);
    }
  }

  float inner = -1.0f;
  float cols[kCols];
  for (uint8_t col = 0; col < kCols; col++) {
    float obst = -1.0f;
    for (uint8_t row = 0; row <= kObstRowMax; row++) {
      const float v = grid[row][col];
      if (v > 0.02f) {
        obst = (obst < 0.0f) ? v : fminf(obst, v);
      }
    }
    cols[col] = obst;

    if (col >= 2 && col <= 5 && obst > 0.0f) {
      inner = (inner < 0.0f) ? obst : fminf(inner, obst);
    }
  }

  static float inner_filt = -1.0f;
  if (inner > 0.0f) {
    inner_filt = (inner_filt < 0.0f) ? inner : (0.55f * inner_filt + 0.45f * inner);
  } else if (inner_filt > 0.0f) {
    inner_filt = fmaxf(inner_filt * 0.82f, inner_filt - 0.04f);
    if (inner_filt < 0.12f) {
      inner_filt = -1.0f;
    }
  }

#if ROVER_L8_SWAP_LR
  for (uint8_t i = 0; i < kCols / 2; i++) {
    const float t = cols[i];
    cols[i] = cols[kCols - 1 - i];
    cols[kCols - 1 - i] = t;
  }
#endif

  float left = -1.0f;
  float right = -1.0f;
  for (uint8_t i = 0; i < 3; i++) {
    if (cols[i] > 0.02f) {
      left = (left < 0.0f) ? cols[i] : fminf(left, cols[i]);
    }
    if (cols[kCols - 1 - i] > 0.02f) {
      right = (right < 0.0f) ? cols[kCols - 1 - i] : fminf(right, cols[kCols - 1 - i]);
    }
  }

  _center_m = inner_filt;
  _left_m = left;
  _right_m = right;
  for (uint8_t i = 0; i < kCols; i++) {
    _col_m[i] = cols[i];
  }
}

bool RoverVl53L8::close_ahead() const {
  if (!_ok) {
    return false;
  }
  return _center_m > 0.02f && _center_m < L8_STOP_M;
}
