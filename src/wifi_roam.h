#pragma once

#include <stdint.h>

#ifndef ROVER_WIFI_ROAM_RSSI_TRIGGER_DBM
#define ROVER_WIFI_ROAM_RSSI_TRIGGER_DBM (-70)
#endif
#ifndef ROVER_WIFI_ROAM_HYSTERESIS_DB
#define ROVER_WIFI_ROAM_HYSTERESIS_DB (8)
#endif
#ifndef ROVER_WIFI_ROAM_CHECK_MS
#define ROVER_WIFI_ROAM_CHECK_MS (10000U)
#endif
#ifndef ROVER_WIFI_ROAM_MIN_INTERVAL_MS
#define ROVER_WIFI_ROAM_MIN_INTERVAL_MS (30000U)
#endif

/** STA policy shared by drive + Pro S3: scan/sort, PS off, optional 802.11k/v. */
void rover_wifi_sta_common_setup(int tx_power_wifi_enum);
/** Call once after WiFi.mode(WIFI_STA) with the rover SSID credentials. */
void rover_wifi_roam_init(const char* ssid, const char* pass);
/** Non-blocking periodic roam check — call from main loop. */
void rover_wifi_roam_tick(uint32_t now_ms);
uint32_t rover_wifi_roam_count();
/** Last octet of current BSSID, or 0 if unknown. */
uint8_t rover_wifi_bssid_last_octet();