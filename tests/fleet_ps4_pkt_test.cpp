// Host-only checksum / scaling sanity (g++ -std=c++17 -I../include tests/fleet_ps4_pkt_test.cpp -lm -o /tmp/fleet_ps4_pkt_test)
#include <cstdio>
#include <cstring>

#include "../include/fleet_ps4_drive.h"
#include "../include/fleet_ps4_pkt.h"

int main() {
  FleetPs4TxPkt tx{};
  tx.flags = FLEET_PS4_FLAG_CTRL_OK;
  tx.seq = 42;
  tx.lx = 64;
  tx.r2 = 200;
  tx.l2 = 0;
  fleetPs4TxSeal(tx);
  if (!fleetPs4TxValid(tx)) {
    fprintf(stderr, "tx valid failed\n");
    return 1;
  }

  FleetPs4AckPkt ack{};
  ack.channel = 6;
  ack.seq = tx.seq;
  memset(ack.mac, 0xAA, 6);
  fleetPs4AckSeal(ack);
  if (!fleetPs4AckValid(ack)) {
    fprintf(stderr, "ack valid failed\n");
    return 1;
  }

  const auto sp = fleetPs4ScaleManual(tx.l2, tx.r2, tx.lx, 0.28f, 0.45f, 0.0f);
  if (sp.throttle < 0.1f || sp.steering < 0.05f) {
    fprintf(stderr, "scale unexpected lin=%f ang=%f\n", sp.throttle, sp.steering);
    return 1;
  }

  puts("fleet_ps4_pkt_test OK");
  return 0;
}