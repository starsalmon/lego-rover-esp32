#include "ultrasonic.h"

namespace {

constexpr uint32_t kMinIntervalMs = 60;
constexpr uint32_t kMinPulseUs = 116;
constexpr uint32_t kMaxPulseUs = 25000;
constexpr float kUsToMetres = 343.0f / 2.0f / 1000000.0f;

}  // namespace

bool Ultrasonic::begin(int trig_pin, int echo_pin) {
  _trig = trig_pin;
  _echo = echo_pin;
  pinMode(_trig, OUTPUT);
  pinMode(_echo, INPUT);
  digitalWrite(_trig, LOW);
  delay(60);
  return true;
}

uint32_t Ultrasonic::ping_once(uint32_t timeout_us) {
  if (_trig < 0 || _echo < 0) return 0;

  const uint32_t now_ms = millis();
  if (_last_ping_ms != 0) {
    const uint32_t elapsed = now_ms - _last_ping_ms;
    if (elapsed < kMinIntervalMs) {
      delay(kMinIntervalMs - elapsed);
    }
  }
  _last_ping_ms = millis();

  digitalWrite(_trig, LOW);
  delayMicroseconds(2);
  digitalWrite(_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trig, LOW);

  const uint32_t pulse = pulseIn(_echo, HIGH, timeout_us);
  if (pulse < kMinPulseUs || pulse > kMaxPulseUs) {
    return 0;
  }
  return pulse;
}

float Ultrasonic::read_range_m(uint32_t timeout_us) {
  if (_trig < 0 || _echo < 0) return -1.0f;

  const uint32_t pulse = ping_once(timeout_us);
  if (pulse == 0) {
    _last_pulse_us = 0;
    return -1.0f;
  }
  _last_pulse_us = pulse;
  return static_cast<float>(pulse) * kUsToMetres;
}
