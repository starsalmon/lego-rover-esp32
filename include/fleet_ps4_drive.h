#pragma once

#include "fleet_ps4_pkt.h"

#include <math.h>

#ifndef FLEET_PS4_STEER_EXPO
#define FLEET_PS4_STEER_EXPO 0.6f
#endif

#ifndef FLEET_PS4_THROTTLE_BRAKE_EPS
#define FLEET_PS4_THROTTLE_BRAKE_EPS 0.35f
#endif

// R2 forward, L2 brake-then-reverse; net R2-L2 when not braking forward.
static inline float fleetPs4ThrottleFromTriggers(uint8_t l2, uint8_t r2, float max_throttle,
                                                 float throttle_now) {
  const float r = (float)r2 / 255.0f;
  const float l = (float)l2 / 255.0f;
  if (l > 0.02f && throttle_now > FLEET_PS4_THROTTLE_BRAKE_EPS && throttle_now > 0.0f && l >= r) {
    return 0.0f;
  }
  const float net = r - l;
  if (fabsf(net) < 0.02f) {
    return 0.0f;
  }
  return net * max_throttle;
}

static inline float fleetPs4SteerExpo(int8_t steer_x, float max_steering,
                                      float expo = FLEET_PS4_STEER_EXPO) {
  const float x = fleetPs4AxisNorm(steer_x);
  const float y = x * (1.0f - expo) + x * x * x * expo;
  return y * max_steering;
}

struct FleetPs4DriveSetpoint {
  float throttle;
  float steering;
};

// steer_x = left thumbstick X (fleet Jazz/Roby/Rover map).
static inline FleetPs4DriveSetpoint fleetPs4ScaleManual(uint8_t l2, uint8_t r2, int8_t steer_x,
                                                        float max_throttle, float max_steering,
                                                        float throttle_now) {
  FleetPs4DriveSetpoint out{};
  out.throttle = fleetPs4ThrottleFromTriggers(l2, r2, max_throttle, throttle_now);
  out.steering = fleetPs4SteerExpo(steer_x, max_steering);
  return out;
}