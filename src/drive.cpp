#include "drive.h"

namespace {

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)

bool pwm_attach(int pin, uint32_t freq, uint8_t bits) {
  return ledcAttach(pin, freq, bits);
}

void pwm_detach(int pin) { ledcDetach(pin); }

void pwm_write(int pin, uint8_t /*ch*/, uint32_t duty) { ledcWrite(pin, duty); }

#else

bool pwm_attach(int pin, uint8_t ch, uint32_t freq, uint8_t bits) {
  ledcSetup(ch, freq, bits);
  ledcAttachPin(pin, ch);
  return true;
}

void pwm_detach(int pin) { ledcDetachPin(pin); }

void pwm_write(int /*pin*/, uint8_t ch, uint32_t duty) { ledcWrite(ch, duty); }

#endif

}  // namespace

void SmoothDrive::gpio_safe_idle(int dir_l, int pwm_l, int dir_r, int pwm_r) {
  pinMode(dir_l, OUTPUT);
  pinMode(dir_r, OUTPUT);
  digitalWrite(dir_l, LOW);
  digitalWrite(dir_r, LOW);
  pinMode(pwm_l, OUTPUT);
  pinMode(pwm_r, OUTPUT);
  digitalWrite(pwm_l, LOW);
  digitalWrite(pwm_r, LOW);
}

bool SmoothDrive::begin(int dir_l, int pwm_l, int dir_r, int pwm_r) {
  _dir_l = dir_l;
  _pwm_l = pwm_l;
  _dir_r = dir_r;
  _pwm_r = pwm_r;
  _pwm_freq = PWM_FREQ_HZ;
  pinMode(_dir_l, OUTPUT);
  pinMode(_dir_r, OUTPUT);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  _pwm_ok = pwm_attach(_pwm_l, _pwm_freq, _pwm_bits) && pwm_attach(_pwm_r, _pwm_freq, _pwm_bits);
#else
  _pwm_ok = pwm_attach(_pwm_l, _ch_l, _pwm_freq, _pwm_bits)
         && pwm_attach(_pwm_r, _ch_r, _pwm_freq, _pwm_bits);
#endif
  stop();
  return _pwm_ok;
}

bool SmoothDrive::set_pwm_freq(uint32_t hz) {
  if (hz < 8 || hz > 400000) return false;
  pwm_detach(_pwm_l);
  pwm_detach(_pwm_r);
  _pwm_freq = hz;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  _pwm_ok = pwm_attach(_pwm_l, _pwm_freq, _pwm_bits) && pwm_attach(_pwm_r, _pwm_freq, _pwm_bits);
#else
  _pwm_ok = pwm_attach(_pwm_l, _ch_l, _pwm_freq, _pwm_bits)
         && pwm_attach(_pwm_r, _ch_r, _pwm_freq, _pwm_bits);
#endif
  return _pwm_ok;
}

void SmoothDrive::set_target(float left, float right) {
  if (left > 1.0f) left = 1.0f;
  if (left < -1.0f) left = -1.0f;
  if (right > 1.0f) right = 1.0f;
  if (right < -1.0f) right = -1.0f;
  _tgt_l = left;
  _tgt_r = right;
}

void SmoothDrive::tick() {
  const bool spinning = (_tgt_l * _tgt_r < 0.0f) && (fabsf(_tgt_l) > 0.05f || fabsf(_tgt_r) > 0.05f);
  const float max_step = spinning ? SPIN_STEP : MAX_STEP;
  auto step = [&](float &cur, float tgt) {
    // Bleed to zero before reversing — stops DIR-pin judder on fwd↔rev.
    if (cur * tgt < -MOVE_EPS * MOVE_EPS) {
      tgt = 0.0f;
    }
    const float step_lim =
        (cur * tgt < 0.0f && fabsf(cur) > MOVE_EPS) ? SPIN_STEP : max_step;
    float d = tgt - cur;
    if (d > step_lim) d = step_lim;
    if (d < -step_lim) d = -step_lim;
    cur += d;
  };
  step(_cur_l, _tgt_l);
  step(_cur_r, _tgt_r);
  // Keep spin ramp symmetric — avoids one wheel dragging (circles instead of pivot).
  if (spinning) {
    const float mag = fmaxf(fabsf(_cur_l), fabsf(_cur_r));
    if (mag > MOVE_EPS) {
      const float sign = (_tgt_r > 0.0f) ? 1.0f : -1.0f;
      _cur_l = -sign * mag;
      _cur_r = sign * mag;
    }
  }
  _apply(_cur_l, _cur_r);
}

void SmoothDrive::stop() {
  _tgt_l = _tgt_r = _cur_l = _cur_r = 0;
  _apply(0, 0);
}

void SmoothDrive::apply_direct(float left, float right) {
  _tgt_l = left;
  _tgt_r = right;
  _cur_l = left;
  _cur_r = right;
  _apply_direct(left, right);
}

void SmoothDrive::apply_duty_lr(uint8_t duty_l, bool rev_l, uint8_t duty_r, bool rev_r) {
  if (!_pwm_ok) return;
#if MOTOR_INVERT_L
  rev_l = !rev_l;
#endif
#if MOTOR_INVERT_R
  rev_r = !rev_r;
#endif
  digitalWrite(_dir_l, rev_l ? LOW : HIGH);
  digitalWrite(_dir_r, rev_r ? LOW : HIGH);
  pwm_write(_pwm_l, _ch_l, duty_l);
  pwm_write(_pwm_r, _ch_r, duty_r);
  _cur_l = duty_l ? (rev_l ? -1.0f : 1.0f) * (duty_l / 255.0f) : 0.0f;
  _cur_r = duty_r ? (rev_r ? -1.0f : 1.0f) * (duty_r / 255.0f) : 0.0f;
  _tgt_l = _cur_l;
  _tgt_r = _cur_r;
}

float SmoothDrive::_cmd_to_scale(float a, float dead_min) {
  // Map |cmd| 0..1 → duty dead_min..1 (same idea as Arduino map(speed, 3, 128, 150, 255)).
  if (a <= MOVE_EPS) return 0.0f;
  return dead_min + (a * (1.0f - dead_min));
}

static void _motor_map_lr(float &l, float &r) {
#if MOTOR_SWAP_LR
  const float t = l;
  l = r;
  r = t;
#endif
#if MOTOR_INVERT_L
  l = -l;
#endif
#if MOTOR_INVERT_R
  r = -r;
#endif
}

void SmoothDrive::_apply(float l, float r) {
  if (!_pwm_ok) return;
  _motor_map_lr(l, r);

  // Spin: equal wheel magnitudes so MIN_PWM floor doesn't pivot on one side.
  if (l * r < 0.0f) {
    const float mag = fmaxf(fabsf(l), fabsf(r));
    if (mag > MOVE_EPS) {
      l = (l < 0.0f) ? -mag : mag;
      r = (r < 0.0f) ? -mag : mag;
    }
  }

  const uint32_t max_duty = (1u << _pwm_bits) - 1;
  auto one = [&](int dir_pin, int pwm_pin, float v, float min_pwm) {
    v = constrain(v, -1.0f, 1.0f);
    digitalWrite(dir_pin, v >= 0 ? HIGH : LOW);
    float a = fabsf(v);
    uint32_t duty = 0;
    if (a > MOVE_EPS) {
      duty = (uint32_t)(_cmd_to_scale(a, min_pwm) * (float)max_duty);
    }
    pwm_write(pwm_pin, (pwm_pin == _pwm_l) ? _ch_l : _ch_r, duty);
  };
  one(_dir_l, _pwm_l, l, MIN_PWM_L);
  one(_dir_r, _pwm_r, r, MIN_PWM_R);
}

void SmoothDrive::_apply_direct(float l, float r) {
  if (!_pwm_ok) return;
  _motor_map_lr(l, r);

  const uint32_t max_duty = (1u << _pwm_bits) - 1;
  auto one = [&](int dir_pin, int pwm_pin, float v) {
    v = constrain(v, -1.0f, 1.0f);
    const uint8_t ch = (pwm_pin == _pwm_l) ? _ch_l : _ch_r;
    if (fabsf(v) < 0.0005f) {
      digitalWrite(dir_pin, LOW);
      pwm_write(pwm_pin, ch, 0);
      return;
    }
    digitalWrite(dir_pin, v >= 0.0f ? HIGH : LOW);
    const uint32_t duty = (uint32_t)(fabsf(v) * (float)max_duty);
    pwm_write(pwm_pin, ch, duty);
  };
  one(_dir_l, _pwm_l, l);
  one(_dir_r, _pwm_r, r);
}
