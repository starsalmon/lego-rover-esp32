#pragma once

#include <stdint.h>

/** Non-blocking boot diagnostic beeps on the T-Display passive buzzer.
 *  Cue order: power-on chirp, optional Wi-Fi and agent tones, then either a
 *  cheerful OK or (after ~15 s) a low double beep plus N short beeps for each
 *  missing device. See README.md for the code table. */
void rover_boot_beep_start();           // early chirp; call after speaker begin
void rover_boot_beep_note_wifi();
void rover_boot_beep_note_agent();
void rover_boot_beep_set_local(bool imu_ok, bool encoders_ok);
void rover_boot_beep_set_remote(bool have_status, bool pca_ok, bool sonar_ok,
                                bool l8_ok, bool tof_l_ok, bool tof_r_ok,
                                bool agent_ok);
void rover_boot_beep_tick(uint32_t now_ms);

// Device codes (N short beeps after the low double error lead).
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
