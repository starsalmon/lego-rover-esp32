#pragma once

// LilyGO T-Display-S3: GPIO15 must be HIGH before the board runs from the
// header "5V" pin (VSYS) or internal battery — not just USB.
void rover_board_power_on();
