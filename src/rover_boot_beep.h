#pragma once

#include <stdint.h>

#ifndef ROVER_BOOT_BEEP_LOCAL_GRACE_MS
#define ROVER_BOOT_BEEP_LOCAL_GRACE_MS 20000U
#endif
#ifndef ROVER_BOOT_BEEP_NETWORK_GRACE_MS
#define ROVER_BOOT_BEEP_NETWORK_GRACE_MS 45000U
#endif

/** millis() captured in rover_boot_beep_start() — shared boot grace anchor. */
uint32_t rover_boot_beep_start_ms();
bool rover_boot_beep_local_grace_over(uint32_t now_ms);
bool rover_boot_beep_network_grace_over(uint32_t now_ms);

/** Non-blocking boot diagnostic beeps on the T-Display passive buzzer.
 *  Cue order: power-on chirp, optional Wi-Fi and agent tones, then either a
 *  cheerful OK when all devices are up, or error codes only after per-device
 *  grace (local ~20 s, Wi-Fi/agent/Pro S3 topics ~45 s). See README.md. */
void rover_boot_beep_start();
void rover_boot_beep_note_wifi();
void rover_boot_beep_note_agent();
void rover_boot_beep_set_local(bool imu_ok, bool encoders_ok);
void rover_boot_beep_set_remote(bool have_status, bool pca_ok, bool sonar_ok,
                                bool l8_ok, bool tof_l_ok, bool tof_r_ok,
                                bool agent_ok);
void rover_boot_beep_tick(uint32_t now_ms);

enum : uint8_t {
  kBootBeepImu = 1,
  kBootBeepEncoders = 2,
  kBootBeepL8 = 3,
  kBootBeepTofLeft = 4,
  kBootBeepTofRight = 5,
  kBootBeepSonar = 6,
  kBootBeepPca = 7,
  kBootBeepAgent = 8,
};