#pragma once

#include <Arduino.h>

#include "fleet_ir_proto.h"

/** Debounced fleet-ping speaker — one bleep-bloop per stable peer encounter. */
class FleetIrPing {
 public:
  void reset() {
    _ping_peer_ = FLEET_IR_PEER_NONE;
    _stable_ = 0;
    _pinged_peer_ = FLEET_IR_PEER_NONE;
    _last_seen_ms_ = 0;
    _last_ping_ms_ = 0;
  }

  bool should_ping(bool peer_seen, uint8_t peer_id, uint8_t self_id, uint32_t now_ms) {
    constexpr uint8_t kStableNeed = 4;
    constexpr uint32_t kCooldownMs = 5000;
    constexpr uint32_t kLossMs = 2000;

    if (!peer_seen || peer_id == self_id || peer_id == FLEET_IR_PEER_NONE) {
      if (_last_seen_ms_ != 0 && now_ms - _last_seen_ms_ > kLossMs) {
        _pinged_peer_ = FLEET_IR_PEER_NONE;
        _stable_ = 0;
        _ping_peer_ = FLEET_IR_PEER_NONE;
      }
      return false;
    }

    _last_seen_ms_ = now_ms;
    if (peer_id == _ping_peer_) {
      if (_stable_ < 255) {
        _stable_++;
      }
    } else {
      _ping_peer_ = peer_id;
      _stable_ = 1;
    }
    if (_stable_ < kStableNeed) {
      return false;
    }
    if (_pinged_peer_ == peer_id && now_ms - _last_ping_ms_ < kCooldownMs) {
      return false;
    }
    _pinged_peer_ = peer_id;
    _last_ping_ms_ = now_ms;
    return true;
  }

 private:
  uint8_t _ping_peer_ = FLEET_IR_PEER_NONE;
  uint8_t _stable_ = 0;
  uint8_t _pinged_peer_ = FLEET_IR_PEER_NONE;
  uint32_t _last_seen_ms_ = 0;
  uint32_t _last_ping_ms_ = 0;
};
