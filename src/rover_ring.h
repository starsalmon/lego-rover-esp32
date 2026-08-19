#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "rover_pins_s3.h"

/** 8× WS2812 ring on GPIO — modes + live sonar distance map. */
class RoverRing {
 public:
  bool begin(int pin, uint8_t count = ROVER_RING_COUNT, uint8_t brightness = ROVER_RING_BRIGHTNESS);
  void tick(uint32_t now_ms);

  void set_mode(uint8_t mode_id);
  void flash(uint8_t flash_id, uint32_t duration_ms = 700);
  void sonar_frame(const uint8_t* dist_cm, uint8_t count, uint8_t sweep_led);

  static constexpr uint8_t kModeStandby = 0;
  static constexpr uint8_t kModeReady = 1;
  static constexpr uint8_t kModeAuto = 2;
  static constexpr uint8_t kModeEscape = 3;
  static constexpr uint8_t kModeSonar = 4;

  static constexpr uint8_t kFlashBump = 1;
  static constexpr uint8_t kFlashStall = 2;
  static constexpr uint8_t kFlashStart = 3;
  static constexpr uint8_t kFlashStop = 4;

 private:
  void render(uint32_t now_ms);
  void set_pixel(uint8_t i, uint8_t r, uint8_t g, uint8_t b);
  void hsv_color(float h, float s, float v, uint8_t& r, uint8_t& g, uint8_t& b) const;

  int _pin = -1;
  uint8_t _count = ROVER_RING_COUNT;
  bool _ok = false;
  uint8_t _mode = kModeStandby;
  uint8_t _flash = 0;
  uint32_t _flash_until_ms = 0;
  float _phase = 0.0f;
  uint8_t _sonar_bins[ROVER_RING_COUNT];
  uint8_t _sonar_sweep = 0;
};
