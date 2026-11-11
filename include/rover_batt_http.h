// Tiny HTTP GET /battery on the T-Display so the AI cam can read percent at boot.
#pragma once

void rover_batt_http_begin();
void rover_batt_http_tick(float voltage);
