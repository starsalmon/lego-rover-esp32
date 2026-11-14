#include "rover_boot_beep.h"

#include "rover_periph.h"
#include "rover_speaker.h"

#include "rover_diag.h"

#include <Arduino.h>

extern RoverPeriph rover_periph;

namespace {

enum class Phase : uint8_t { kIdle, kWait, kOk, kErr, kDone };

Phase g_phase = Phase::kIdle;
uint32_t g_start_ms = 0;
bool g_wifi = false, g_agent_tone = false;
bool g_imu = false, g_enc = false;
bool g_have_status = false, g_pca = false, g_sonar = false;
bool g_l8 = false, g_tl = false, g_tr = false, g_agent = false;
uint8_t g_fail[8];
uint8_t g_fail_n = 0;
uint8_t g_fail_i = 0;
uint32_t g_err_wait_until = 0;
uint8_t g_played_codes = 0;

uint32_t elapsed_ms(uint32_t now_ms) {
  return (g_start_ms && now_ms >= g_start_ms) ? (now_ms - g_start_ms) : 0;
}

void mark_code_played(uint8_t code) {
  if (code >= 1 && code <= 8) {
    g_played_codes |= (uint8_t)(1U << (code - 1U));
  }
}

bool code_already_played(uint8_t code) {
  if (code < 1 || code > 8) {
    return true;
  }
  return (g_played_codes & (uint8_t)(1U << (code - 1U))) != 0;
}

void collect_fails(uint32_t now_ms) {
  g_fail_n = 0;
  const bool local_grace = elapsed_ms(now_ms) >= ROVER_BOOT_BEEP_LOCAL_GRACE_MS;
  const bool net_grace = elapsed_ms(now_ms) >= ROVER_BOOT_BEEP_NETWORK_GRACE_MS;
  auto add = [&](uint8_t c) {
    if (g_fail_n < 8 && !code_already_played(c)) {
      g_fail[g_fail_n++] = c;
    }
  };
  if (local_grace) {
    if (!g_imu) {
      add(kBootBeepImu);
    }
    if (!g_enc) {
      add(kBootBeepEncoders);
    }
  }
  if (net_grace) {
    if (!g_agent) {
      add(kBootBeepAgent);
    } else {
      if (!g_l8) {
        add(kBootBeepL8);
      }
      if (!g_tl) {
        add(kBootBeepTofLeft);
      }
      if (!g_tr) {
        add(kBootBeepTofRight);
      }
      if (!g_sonar) {
        add(kBootBeepSonar);
      }
      if (!g_pca) {
        add(kBootBeepPca);
      }
    }
  }
}

bool all_ok() {
  return g_imu && g_enc && g_agent && g_l8 && g_tl && g_tr && g_sonar && g_pca;
}

void play_ok(uint32_t now_ms) {
  rover_periph.play(RoverSpeaker::kMelodyHwOk);
  Serial.printf("boot_beep OK imu enc l8 tofL tofR sonar pca (t=%lu)\n",
                (unsigned long)elapsed_ms(now_ms));
  rover_diag_event("boot_beep OK t=%lu", (unsigned long)elapsed_ms(now_ms));
  g_phase = Phase::kOk;
}

}  // namespace

uint32_t rover_boot_beep_start_ms() { return g_start_ms; }

bool rover_boot_beep_local_grace_over(uint32_t now_ms) {
  return g_start_ms != 0 && elapsed_ms(now_ms) >= ROVER_BOOT_BEEP_LOCAL_GRACE_MS;
}

bool rover_boot_beep_network_grace_over(uint32_t now_ms) {
  return g_start_ms != 0 && elapsed_ms(now_ms) >= ROVER_BOOT_BEEP_NETWORK_GRACE_MS;
}

void rover_boot_beep_start() {
  g_phase = Phase::kWait;
  g_start_ms = millis();
  g_wifi = g_agent_tone = false;
  g_imu = g_enc = false;
  g_have_status = g_pca = g_sonar = false;
  g_l8 = g_tl = g_tr = g_agent = false;
  g_fail_n = g_fail_i = 0;
  g_played_codes = 0;
  g_err_wait_until = 0;
  rover_periph.play(RoverSpeaker::kMelodyBootChirp);
  Serial.println("boot_beep chirp");
  rover_diag_event("boot_beep chirp");
}

void rover_boot_beep_note_wifi() {
  if (g_wifi) return;
  g_wifi = true;
  rover_periph.play(RoverSpeaker::kMelodyWifiOk);
  Serial.println("boot_beep wifi");
  rover_diag_event("boot_beep wifi");
}

void rover_boot_beep_note_agent() {
  g_agent = true;
  if (g_agent_tone) return;
  g_agent_tone = true;
  rover_periph.play(RoverSpeaker::kMelodyAgentOk);
  Serial.println("boot_beep agent");
  rover_diag_event("boot_beep agent");
}

void rover_boot_beep_set_local(bool imu_ok, bool encoders_ok) {
  if (imu_ok) g_imu = true;
  if (encoders_ok) g_enc = true;
}

void rover_boot_beep_set_remote(bool have_status, bool pca_ok, bool sonar_ok,
                                bool l8_ok, bool tof_l_ok, bool tof_r_ok,
                                bool agent_ok) {
  if (have_status) g_have_status = true;
  if (pca_ok) g_pca = true;
  if (sonar_ok) g_sonar = true;
  if (l8_ok) g_l8 = true;
  if (tof_l_ok) g_tl = true;
  if (tof_r_ok) g_tr = true;
  if (agent_ok) g_agent = true;
}

void rover_boot_beep_tick(uint32_t now_ms) {
  if (g_phase == Phase::kIdle || g_phase == Phase::kDone || g_phase == Phase::kOk) {
    return;
  }

  if (g_phase == Phase::kWait) {
    if (all_ok()) {
      play_ok(now_ms);
      return;
    }
  if (elapsed_ms(now_ms) < ROVER_BOOT_BEEP_LOCAL_GRACE_MS) {
      return;
    }
    collect_fails(now_ms);
    if (g_fail_n == 0) {
      if (elapsed_ms(now_ms) < ROVER_BOOT_BEEP_NETWORK_GRACE_MS) {
        return;
      }
      g_phase = Phase::kDone;
      return;
    }
    Serial.print("boot_beep ERR");
    for (uint8_t i = 0; i < g_fail_n; i++) {
      Serial.printf(" %u", g_fail[i]);
    }
    Serial.println();
    g_fail_i = 0;
    g_phase = Phase::kErr;
    g_err_wait_until = 0;
  }

  if (g_phase != Phase::kErr) {
    return;
  }

  if (rover_periph.speaker_busy()) {
    g_err_wait_until = 1;
    return;
  }
  if (g_err_wait_until == 1) {
    g_err_wait_until = now_ms + 250;
    return;
  }
  if (g_err_wait_until != 0 && now_ms < g_err_wait_until) {
    return;
  }

  if (g_fail_i >= g_fail_n) {
    if (all_ok()) {
      play_ok(now_ms);
      return;
    }
    if (elapsed_ms(now_ms) < ROVER_BOOT_BEEP_NETWORK_GRACE_MS) {
      g_phase = Phase::kWait;
      g_fail_n = 0;
      g_fail_i = 0;
      g_err_wait_until = 0;
      return;
    }
    collect_fails(now_ms);
    if (g_fail_n > 0) {
      Serial.print("boot_beep ERR more");
      for (uint8_t i = 0; i < g_fail_n; i++) {
        Serial.printf(" %u", g_fail[i]);
      }
      Serial.println();
      g_fail_i = 0;
      g_err_wait_until = 0;
      return;
    }
    g_phase = Phase::kDone;
    Serial.println("boot_beep ERR done");
    rover_diag_event("boot_beep ERR done");
    return;
  }

  const uint8_t code = g_fail[g_fail_i++];
  mark_code_played(code);
  rover_periph.play_error_code(code);
  Serial.printf("boot_beep ERR play code %u\n", code);
  rover_diag_event("boot_beep ERR %u", code);
  g_err_wait_until = 0;
}