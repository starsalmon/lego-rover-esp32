#include "rover_pi_periph.h"

namespace {

constexpr uint8_t kSync0 = 0xA5;
constexpr uint8_t kSync1 = 0x5A;
constexpr uint8_t kCmdPlay = 0x10;
constexpr uint8_t kCmdRing = 0x11;
constexpr uint8_t kCmdSonarRing = 0x12;

uint8_t checksum(uint8_t cmd, uint8_t arg) { return cmd ^ arg ^ 0xC3; }

uint8_t checksum_ext(uint8_t cmd, uint8_t len, const uint8_t* payload) {
  uint8_t c = cmd ^ len ^ 0xC3;
  for (uint8_t i = 0; i < len; i++) {
    c ^= payload[i];
  }
  return c;
}

}  // namespace

void RoverPiPeriph::begin(HardwareSerial& serial, uint32_t baud) {
  (void)baud;
  _ser = &serial;
}

void RoverPiPeriph::tick(uint32_t now_ms) {
  (void)now_ms;
  while (_ser && _ser->available() > 0) {
    _ser->read();
  }
}

void RoverPiPeriph::send(uint8_t cmd, uint8_t arg) {
  if (!_ser) {
    return;
  }
  const uint8_t frame[] = {kSync0, kSync1, cmd, arg, checksum(cmd, arg)};
  _ser->write(frame, sizeof(frame));
}

void RoverPiPeriph::play(uint8_t melody_id) { send(kCmdPlay, melody_id); }

void RoverPiPeriph::ring_mode(uint8_t mode_id) { send(kCmdRing, mode_id); }

void RoverPiPeriph::sonar_frame(const uint8_t* dist_cm, uint8_t count, uint8_t sweep_led) {
  if (!_ser || !dist_cm || count == 0 || count > 8) {
    return;
  }
  uint8_t payload[9];
  for (uint8_t i = 0; i < count; i++) {
    payload[i] = dist_cm[i];
  }
  payload[count] = sweep_led;
  const uint8_t len = static_cast<uint8_t>(count + 1);
  uint8_t frame[5 + 9];
  frame[0] = kSync0;
  frame[1] = kSync1;
  frame[2] = kCmdSonarRing;
  frame[3] = len;
  for (uint8_t i = 0; i < len; i++) {
    frame[4 + i] = payload[i];
  }
  frame[4 + len] = checksum_ext(kCmdSonarRing, len, payload);
  _ser->write(frame, 5 + len);
}

void RoverPiPeriph::notify_bump() {
  play(kMelodyBump);
  ring_mode(kRingBump);
}

void RoverPiPeriph::notify_stall() {
  play(kMelodyStall);
  ring_mode(kRingStall);
}

void RoverPiPeriph::notify_escape() { ring_mode(kRingEscape); }

void RoverPiPeriph::notify_session(bool active) {
  play(active ? kMelodyAutoStart : kMelodyAutoStop);
  ring_mode(active ? kRingAuto : kRingStandby);
}
