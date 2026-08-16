#include "rover_wander.h"

#include <math.h>

#include "rover_pins_s3.h"

#ifndef ROVER_WANDER_MAX_LIN
#define ROVER_WANDER_MAX_LIN ROVER_CRUISE_MAX_LIN
#endif
#ifndef ROVER_WANDER_MIN_LIN
#define ROVER_WANDER_MIN_LIN (ROVER_CRUISE_MAX_LIN * 0.65f)
#endif
#ifndef ROVER_WANDER_MAX_ANG
#define ROVER_WANDER_MAX_ANG 0.08f
#endif

void RoverWander::begin() { reset(); }

void RoverWander::reset() {
  _mode_until_ms = 0;
  _lin = 0.0f;
  _ang = 0.0f;
  pick_next(0);
}

void RoverWander::pick_next(uint32_t now_ms) {
  const float span = ROVER_WANDER_MAX_LIN - ROVER_WANDER_MIN_LIN;
  _lin = ROVER_WANDER_MIN_LIN + span * (static_cast<float>(random(0, 1000)) / 1000.0f);
  const float steer = ROVER_WANDER_MAX_ANG * (static_cast<float>(random(0, 1000)) / 500.0f - 1.0f);
  _ang = steer;
  const uint32_t dur = static_cast<uint32_t>(random(2500, 6500));
  _mode_until_ms = now_ms + dur;
}

bool RoverWander::tick(uint32_t now_ms, float* out_lin, float* out_ang) {
  if (!out_lin || !out_ang) {
    return false;
  }
  if (_mode_until_ms == 0) {
    pick_next(now_ms);
  } else if (now_ms >= _mode_until_ms) {
    pick_next(now_ms);
  }
  *out_lin = _lin;
  *out_ang = _ang;
  return true;
}
