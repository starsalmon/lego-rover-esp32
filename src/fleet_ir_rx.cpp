#include "fleet_ir_rx.h"

namespace {

constexpr uint32_t kMinPulseUs = 220;
constexpr uint32_t kMaxPulseUs = 1100;
constexpr uint32_t kMinBitGapUs = 180;
constexpr uint32_t kMaxBitGapUs = 900;
constexpr uint32_t kFrameIdleUs = 4000;
constexpr uint8_t kMatchNeed = FLEET_IR_MATCH_NEED;
constexpr uint32_t kHoldUs = FLEET_IR_FRAME_US * FLEET_IR_HOLD_MULT;

}  // namespace

void FleetIrRx::reset() {
  _last_on = false;
  _edge_us = 0;
  _phase = Phase::kIdle;
  _phase_start_us = 0;
  _pulse_count = 0;
  _match_streak = 0;
  _detected = false;
  _peer_id = FLEET_IR_PEER_NONE;
  _last_frame_us = 0;
}

void FleetIrRx::poll(bool carrier_on, uint32_t now_us) {
  if (carrier_on != _last_on) {
    const uint32_t dt = now_us - _edge_us;
    if (_last_on) {
      // Falling edge — end of burst.
      if (_phase == Phase::kSync && dt >= kMinPulseUs && dt <= kMaxPulseUs) {
        _phase = Phase::kData;
        _pulse_count = 1;
      } else if (_phase == Phase::kData && dt >= kMinPulseUs && dt <= kMaxPulseUs) {
        if (_pulse_count < 16) {
          _pulse_count++;
        }
      }
    } else {
      // Rising edge — gap after burst.
      if (_phase == Phase::kIdle && dt >= kFrameIdleUs) {
        _phase = Phase::kSync;
        _phase_start_us = now_us;
        _pulse_count = 0;
      } else if (_phase == Phase::kData && dt >= kMinBitGapUs && dt <= kMaxBitGapUs) {
        // still counting bursts on next fall
      } else if (_phase == Phase::kData && dt > kMaxBitGapUs) {
        if (_pulse_count >= 1 && _pulse_count <= (FLEET_IR_MAX_ID + 1)) {
          const uint8_t id = static_cast<uint8_t>(_pulse_count - 1);
          if (id != static_cast<uint8_t>(FLEET_IR_ID)) {
            if (_peer_id == id) {
              if (_match_streak < 255) {
                _match_streak++;
              }
            } else {
              _peer_id = id;
              _match_streak = 1;
            }
            if (_match_streak >= kMatchNeed) {
              _detected = true;
              _last_frame_us = now_us;
            }
          }
        }
        _phase = Phase::kIdle;
        _pulse_count = 0;
      }
    }
    _edge_us = now_us;
    _last_on = carrier_on;
  }

  if (_phase == Phase::kSync && carrier_on && (now_us - _phase_start_us) >= FLEET_IR_SYNC_US) {
    _phase = Phase::kData;
    _pulse_count = 0;
  }

  if (_detected && (now_us - _last_frame_us) > kHoldUs) {
    _detected = false;
    _peer_id = FLEET_IR_PEER_NONE;
    _match_streak = 0;
  }
}
