#pragma once
#include <esp_http_server.h>
// MP3 playback on the DFR1154 MAX98357 (ESP32-audioI2S). Runs on its own task;
// HTTP handlers only queue commands.
void audio_begin();                         // mount LittleFS, start audio task
void audio_register(httpd_handle_t server); // /play /stop /volume /upload /sounds
void audio_play_startup();                  // play /startup.mp3 once (after Wi-Fi up)
