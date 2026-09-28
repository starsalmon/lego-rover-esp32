#pragma once

#include <Arduino.h>

#include "fleet_ir_proto.h"

// Decode fleet IR frames from a TSOP line (true = 38 kHz carrier present).
class FleetIrRx {
 public:
  void reset();
  void poll(bool carrier_on, uint32_t now_us = micros());

  bool detected() const { return _detected; }
  uint8_t peer_id() const { return _peer_id; }

 private:
  enum class Phase : uint8_t {
    kIdle = 0,
    kSync,
    kData,
    kGap,
  };

  bool _last_on = false;
  uint32_t _edge_us = 0;
  Phase _phase = Phase::kIdle;
  uint32_t _phase_start_us = 0;
  uint8_t _pulse_count = 0;
  uint8_t _match_streak = 0;
  bool _detected = false;
  uint8_t _peer_id = FLEET_IR_PEER_NONE;
  uint32_t _last_frame_us = 0;
};

// Run TX burst timing + RX edge sampling for a few ms each loop (560 µs slots need <1 ms ticks).
template <typename TickTx, typename PollRx>
inline void fleet_ir_service(TickTx tick_tx, PollRx poll_rx, uint32_t budget_us = 5000) {
  const uint32_t end = micros() + budget_us;
  while (static_cast<int32_t>(end - micros()) > 0) {
    tick_tx();
    poll_rx();
  }
}
