#include "rover_vl53l8.h"

#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <vl53l8cx.h>

#include "rover_i2c.h"
#include "rover_diag.h"
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
  if (z >= VL53L8CX_RESOLUTION_4X4) {
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
    // 4x4 ≈1/4 the I2C payload vs 8x8 (~38 ms @400 kHz) — keeps IMU ≥18 Hz on shared Wire.
    g_sensor.set_resolution(VL53L8CX_RESOLUTION_4X4);
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
  Serial.println("VL53L8CX 4x4 ranging (rate-friendly dump)");
  return true;
}

void RoverVl53L8::poll() {
  if (!_ok) {
    return;
  }
  uint8_t ready = 0;
  // 1) Short lock: data-ready probe only.
  {
    RoverI2cGuard guard;
    if (!guard) {
      return;
    }
    Wire.setTimeOut(40);
    const uint8_t ready_st = g_sensor.check_data_ready(&ready);
    Wire.setTimeOut(ROVER_I2C_TIMEOUT_MS);
    if (ready_st != 0 || ready == 0) {
      return;
    }
  }
  // Let higher-prio imu_diag grab Wire before the long 8x8 dump (~38 ms).
  vTaskDelay(pdMS_TO_TICKS(5));  // short yield; 4x4 dump ~15 ms
  // 2) Dump under lock, then release BEFORE CPU grid work (was holding mutex
  // through the whole 8x8 reduction — starved MPU / DIAG ~2x).
  {
    RoverI2cGuard guard;
    if (!guard) {
      return;
    }
    // 400 kHz for the ranging dump only; restore 100 kHz after (4x4 payload).
    Wire.setClock(400000);
    Wire.setTimeOut(80);
    const uint32_t t0 = micros();
    const uint8_t data_st = g_sensor.get_ranging_data(&g_results);
    const uint32_t dump_us = micros() - t0;
    Wire.setClock(ROVER_I2C_HZ);
    Wire.setTimeOut(ROVER_I2C_TIMEOUT_MS);
    if (data_st != 0) {
      return;
    }
    static uint32_t last_dump_log_ms = 0;
    const uint32_t now = millis();
    if (now - last_dump_log_ms >= 5000) {
      last_dump_log_ms = now;
      rover_diag_event("l8_dump_us=%lu", static_cast<unsigned long>(dump_us));
    }
  }

  // 4x4 grid: rows 0–1 = forward cone (skip lower carpet-ish rows 2–3).
  constexpr uint8_t kObstRowMax = 1;
  float grid4[4][4];
  for (uint8_t row = 0; row < 4; row++) {
    for (uint8_t col = 0; col < 4; col++) {
      grid4[row][col] = -1.0f;
      const uint8_t z = static_cast<uint8_t>(row * 4 + col);
      if (!zone_ok(g_results, z)) {
        continue;
      }
      grid4[row][col] = zone_m(g_results, z);
    }
  }

  float inner = -1.0f;
  float cols4[4];
  for (uint8_t col = 0; col < 4; col++) {
    float obst = -1.0f;
    for (uint8_t row = 0; row <= kObstRowMax; row++) {
      const float v = grid4[row][col];
      if (v > 0.02f) {
        obst = (obst < 0.0f) ? v : fminf(obst, v);
      }
    }
    // Bottom rows see the floor at a steady ~18 cm. Only a closer return is an obstacle.
    float floor_hit = -1.0f;
    for (uint8_t row = 2; row < 4; row++) {
      const float v = grid4[row][col];
      if (v > 0.02f && v < 0.12f) {
        floor_hit = (floor_hit < 0.0f) ? v : fminf(floor_hit, v);
      }
    }
    if (floor_hit > 0.0f) {
      obst = (obst < 0.0f) ? floor_hit : fminf(obst, floor_hit);
    }
    cols4[col] = obst;
    // Center two columns of 4x4 ≈ old inner band.
    if (col >= 1 && col <= 2 && obst > 0.0f) {
      inner = (inner < 0.0f) ? obst : fminf(inner, obst);
    }
  }
  // Upsample 4 → 8 columns for existing ROS /rover/tof/l8_cols API.
  float cols[kCols];
  for (uint8_t c = 0; c < 4; c++) {
    cols[c * 2] = cols4[c];
    cols[c * 2 + 1] = cols4[c];
  }

  // No hold: a close wall must replace a far reading immediately.
  const float inner_filt = inner;

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
