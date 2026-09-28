#pragma once

// Fleet bot-to-bot IR ID — 38 kHz TSOP, frame every ~24 ms.
// TX: 2 ms sync bursts, then (fleet_id + 1) data bursts, then silence.
// RX: count data bursts after sync → peer_id = count - 1.

#ifndef FLEET_IR_ID
#define FLEET_IR_ID 0
#endif

#ifndef FLEET_IR_ROVER
#define FLEET_IR_ROVER 0
#endif
#ifndef FLEET_IR_MINI
#define FLEET_IR_MINI 1
#endif
#ifndef FLEET_IR_TRACKED
#define FLEET_IR_TRACKED 2
#endif

#ifndef FLEET_IR_PEER_NONE
#define FLEET_IR_PEER_NONE 255
#endif

#ifndef FLEET_IR_BURST_US
#define FLEET_IR_BURST_US 560
#endif
#ifndef FLEET_IR_BIT_GAP_US
#define FLEET_IR_BIT_GAP_US 520
#endif
#ifndef FLEET_IR_SYNC_US
#define FLEET_IR_SYNC_US 2000
#endif
#ifndef FLEET_IR_FRAME_US
#define FLEET_IR_FRAME_US 24000
#endif
#ifndef FLEET_IR_MAX_ID
#define FLEET_IR_MAX_ID 7
#endif
#ifndef FLEET_IR_MATCH_NEED
#define FLEET_IR_MATCH_NEED 3
#endif
#ifndef FLEET_IR_HOLD_MULT
#define FLEET_IR_HOLD_MULT 15
#endif
