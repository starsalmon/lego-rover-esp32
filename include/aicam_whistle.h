#pragma once
#include <esp_http_server.h>
// Whistle detector on the DFR1154 PDM mic (CLK GPIO38, DATA GPIO39), I2S0 PDM RX at 16 kHz.
// Goertzel over 1.0-3.0 kHz (62.5 Hz bins, 16 ms blocks). A whistle = one dominant tone, pitch steady to
// +/-250 Hz, held >= 400 ms. Ignored while the speaker plays (and 400 ms after).
//   GET /whistle                     {"count","age_ms","freq","dur_ms","rms","ratio","peak_hz","min_ratio","min_rms","min_ms"}
//   GET /whistle?ratio=0.25&rms=40&ms=400   tune thresholds live (RAM only)
void whistle_begin();
void whistle_register(httpd_handle_t server);
