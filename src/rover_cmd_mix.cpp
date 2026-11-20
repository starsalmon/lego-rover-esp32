#include "rover_cmd_mix.h"

#include <math.h>

static RoverCmdMixPort g_port{};
static float g_trim_out = 0.0f;
static portMUX_TYPE g_trim_mux = portMUX_INITIALIZER_UNLOCKED;

void rover_cmd_mix_set_port(const RoverCmdMixPort* port) {
  if (port) {
    g_port = *port;
  } else {
    g_port = {};
  }
}

void rover_cmd_mix_reset_trim() {
  portENTER_CRITICAL(&g_trim_mux);
  g_trim_out = 0.0f;
  portEXIT_CRITICAL(&g_trim_mux);
}

float rover_cmd_mix_trim_out() {
  portENTER_CRITICAL(&g_trim_mux);
  const float t = g_trim_out;
  portEXIT_CRITICAL(&g_trim_mux);
  return t;
}

bool rover_cmd_mix_twist(float lin, float ang, float yaw_rate, float dt, bool sonar_override, float* out_l,
                         float* out_r) {
  if (!out_l || !out_r) {
    return false;
  }

  float v = lin;
  float turn = ang;

#ifndef FORWARD_LINEAR_SIGN
#define FORWARD_LINEAR_SIGN 1.0f
#endif
  const bool reversing =
      (FORWARD_LINEAR_SIGN > 0.0f) ? (v < -0.03f) : (v > 0.03f);
  if (!sonar_override) {
#if defined(ROVER_VL53L8)
    if (g_port.front_tof_ok && g_port.front_tof_ok() && !reversing && v > 0.02f) {
      const float fwd = g_port.front_range_m ? g_port.front_range_m() : -1.0f;
      if (fwd > 0.02f && fwd < 0.55f) {
        const float t = (fwd - 0.10f) / 0.45f;
        const float cap = (t < 0.0f) ? 0.04f : (0.04f + t * 0.10f);
        if (v > cap) {
          v = cap;
        }
        if (fwd < 0.16f) {
          v = -0.06f;
          if (fabsf(turn) > 0.05f) {
            turn = (turn > 0.0f) ? 0.05f : -0.05f;
          }
        }
      }
    }
#endif
  }

  float trim = 0.0f;
  if (g_port.mpu_ok && g_port.mpu_ok() && !sonar_override) {
#ifndef HEADING_HOLD
#define HEADING_HOLD 1
#endif
#if HEADING_HOLD
    if (g_port.heading_lock && g_port.heading_correct && g_port.heading_lock()) {
      trim = g_port.heading_correct(yaw_rate, dt, v, turn);
      if (g_port.heading_unlock) {
        g_port.heading_unlock();
      }
    }
#endif
  } else if (g_port.heading_lock && g_port.heading_disarm) {
    if (g_port.heading_lock()) {
      g_port.heading_disarm();
      if (g_port.heading_unlock) {
        g_port.heading_unlock();
      }
    }
  }

  float trim_out = 0.0f;
  const float trim_step = 0.35f * ((dt > 0.0001f && dt < 0.05f) ? dt : 0.01f);
  portENTER_CRITICAL(&g_trim_mux);
  if (trim > g_trim_out + trim_step) {
    g_trim_out += trim_step;
  } else if (trim < g_trim_out - trim_step) {
    g_trim_out -= trim_step;
  } else {
    g_trim_out = trim;
  }
  trim_out = g_trim_out;
  portEXIT_CRITICAL(&g_trim_mux);

  float l = 0.0f;
  float r = 0.0f;
  constexpr float STEER_DEAD = 0.02f;

  // cmd_vel: linear.x cruise, angular.z turn (CMD_VEL_CROSS_FIX=0). MOTOR_INVERT_* in SmoothDrive::_apply only.
  if (fabsf(turn) < STEER_DEAD) {
    l = v + trim_out;
    r = v - trim_out;
  } else if (fabsf(v) < 0.04f && fabsf(turn) >= STEER_DEAD) {
    const float spin = fminf(fabsf(turn), 0.10f);
    turn = (turn > 0.0f) ? spin : -spin;
    l = -turn;
    r = turn;
  } else {
    float max_turn = fminf(0.06f, fabsf(v) * 0.45f);
    if (max_turn < 0.03f) {
      max_turn = 0.03f;
    }
    if (fabsf(turn) > max_turn) {
      turn = (turn > 0.0f) ? max_turn : -max_turn;
    }
    l = v - turn;
    r = v + turn;
  }

  const float peak = fmaxf(fabsf(l), fabsf(r));
  if (peak > 1.0f) {
    l /= peak;
    r /= peak;
  }

  *out_l = constrain(l, -1.0f, 1.0f);
  *out_r = constrain(r, -1.0f, 1.0f);
  return true;
}