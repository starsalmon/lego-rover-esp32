#include "rover_display.h"
#include "rover_button_events.h"

#ifdef ROVER_TDISPLAY_S3

#include <TFT_eSPI.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include "rover_pins_s3.h"

namespace {

static TFT_eSPI tft;
static TFT_eSprite sprite(&tft);

constexpr int kScrW = 320;
constexpr int kScrH = 170;
constexpr int kFaceCx = 248;
constexpr int kFaceCy = 92;
constexpr int kFaceR = 58;
constexpr int kTextX = 6;
constexpr int kLineH2 = 18;  // text size 2 line height

enum class FaceExpr : uint8_t {
  kBoot = 0,
  kWait,
  kExploreWait,
  kConnect,
  kHappy,
  kSad,
  kAngry,
  kSurprise,
  kLost,
  kEstop,
};

static const char *link_label(int s) {
  switch (s) {
    case 1:
      return "connecting";
    case 2:
      return "Server linked";
    case 3:
      return "link LOST";
    default:
      return "wait link";
  }
}

static uint16_t bat_colour(const RoverUiData &d) {
  if (d.bat_critical) return TFT_RED;
  if (d.bat_low) return TFT_ORANGE;
  return TFT_GREEN;
}

static bool feq(float a, float b) { return fabsf(a - b) < 0.005f; }

static FaceExpr face_for(const RoverUiData &d) {
  if (d.estop) return FaceExpr::kEstop;
  if (d.bump) return FaceExpr::kSurprise;
  if (d.stall) return FaceExpr::kAngry;
  if (d.bat_critical) return FaceExpr::kSad;
  if (d.mode && strcmp(d.mode, "low batt") == 0) return FaceExpr::kSad;
  if (d.mode && strcmp(d.mode, "Pi stopped") == 0) return FaceExpr::kWait;
  if (d.mode && strcmp(d.mode, "Standby") == 0) return FaceExpr::kExploreWait;
  if (d.mode_menu) return FaceExpr::kExploreWait;
  if (d.mode && strstr(d.mode, "waiting") != nullptr) return FaceExpr::kExploreWait;
  if (d.mode && strncmp(d.mode, "Explore", 7) == 0) return FaceExpr::kHappy;
  if (d.link_state == 3) return FaceExpr::kLost;
  if (d.link_state == 1) return FaceExpr::kConnect;
#if defined(ENABLE_OTA)
  if (d.ota_state == 1) return FaceExpr::kConnect;
  if (d.ota_state == 3) return FaceExpr::kSad;
#endif
  if (d.link_state == 2) {
    const bool moving = fabsf(d.cmd_lin) > 0.03f || fabsf(d.cmd_ang) > 0.03f ||
                        fabsf(d.motor_l) > 0.03f || fabsf(d.motor_r) > 0.03f;
    return moving ? FaceExpr::kHappy : FaceExpr::kHappy;
  }
  if (d.mode && strcmp(d.mode, "boot") == 0) return FaceExpr::kBoot;
  return FaceExpr::kWait;
}

static uint16_t face_tint(FaceExpr expr) {
  switch (expr) {
    case FaceExpr::kHappy:
      return TFT_YELLOW;
    case FaceExpr::kAngry:
      return TFT_ORANGE;
    case FaceExpr::kSurprise:
      return TFT_MAGENTA;
    case FaceExpr::kSad:
    case FaceExpr::kLost:
      return TFT_BLUE;
    case FaceExpr::kEstop:
      return TFT_RED;
    case FaceExpr::kConnect:
      return TFT_CYAN;
    case FaceExpr::kExploreWait:
      return TFT_CYAN;
    default:
      return TFT_DARKGREY;
  }
}

static void draw_eye(TFT_eSprite &g, int x, int y, int r, bool blink) {
  if (blink) {
    g.drawLine(x - r, y, x + r, y, TFT_BLACK);
    return;
  }
  g.fillCircle(x, y, r, TFT_WHITE);
  g.fillCircle(x, y, r / 2, TFT_BLACK);
}

static void draw_eye_look(TFT_eSprite &g, int x, int y, int r, int look_dx, int look_dy,
                          bool blink) {
  if (blink) {
    g.drawLine(x - r, y, x + r, y, TFT_BLACK);
    return;
  }
  g.fillCircle(x, y, r, TFT_WHITE);
  g.fillCircle(x + look_dx, y + look_dy, r / 2, TFT_BLACK);
}

static void draw_x_eye(TFT_eSprite &g, int x, int y, int r) {
  g.drawLine(x - r, y - r, x + r, y + r, TFT_WHITE);
  g.drawLine(x - r, y + r, x + r, y - r, TFT_WHITE);
}

static int mouth_y() { return kFaceCy + kFaceR / 3; }

// TFT_eSPI drawArc: 0°=3 o'clock, 90°=6 o'clock. Rotate mouth arcs -90° on display.
static uint32_t arc_rot(uint32_t deg) { return (deg + 270U) % 360U; }

static void draw_mouth_flat(TFT_eSprite &g, int cx, int cy, int half_w) {
  g.drawLine(cx - half_w, cy, cx + half_w, cy, TFT_BLACK);
}

static void draw_mouth_smile(TFT_eSprite &g, int cx, int cy, int r, uint16_t bg) {
  g.drawArc(cx, cy - r, r, r - 2, arc_rot(35), arc_rot(145), TFT_BLACK, bg, false);
}

static void draw_mouth_frown(TFT_eSprite &g, int cx, int cy, int r, uint16_t bg) {
  g.drawArc(cx, cy + r, r, r - 2, arc_rot(215), arc_rot(325), TFT_BLACK, bg, false);
}

#if defined(ENABLE_OTA)
static void draw_wifi_icon(TFT_eSprite &g, int ota_state) {
  constexpr int bx = kScrW - 18;
  constexpr int by = 34;

  uint16_t col;
  if (ota_state == 2) {
    col = TFT_GREEN;
  } else if (ota_state == 1) {
    col = TFT_YELLOW;
  } else {
    col = TFT_DARKGREY;
  }

  g.fillCircle(bx, by, 2, col);
  g.drawArc(bx, by, 8, 6, arc_rot(200), arc_rot(340), col, TFT_BLACK, false);
  g.drawArc(bx, by, 13, 11, arc_rot(200), arc_rot(340), col, TFT_BLACK, false);
  g.drawArc(bx, by, 18, 16, arc_rot(200), arc_rot(340), col, TFT_BLACK, false);
}
#endif

static void draw_face(TFT_eSprite &g, FaceExpr expr, uint32_t now_ms) {
  const uint16_t tint = face_tint(expr);
  const bool blink = (expr == FaceExpr::kHappy || expr == FaceExpr::kBoot ||
                      expr == FaceExpr::kWait || expr == FaceExpr::kExploreWait) &&
                     ((now_ms / 3500) % 2 == 0) && ((now_ms % 3500) < 120);

  g.fillCircle(kFaceCx, kFaceCy, kFaceR, tint);
  g.drawCircle(kFaceCx, kFaceCy, kFaceR, TFT_WHITE);

  const int eye_y = kFaceCy - kFaceR / 5;
  const int eye_dx = kFaceR / 3;
  const int eye_r = kFaceR / 7;
  const int mouth = mouth_y();
  const int mouth_hw = kFaceR / 3;
  const int mouth_arc_r = kFaceR / 4;

  switch (expr) {
    case FaceExpr::kEstop:
      draw_x_eye(g, kFaceCx - eye_dx, eye_y, eye_r + 2);
      draw_x_eye(g, kFaceCx + eye_dx, eye_y, eye_r + 2);
      draw_mouth_flat(g, kFaceCx, mouth, mouth_hw);
      break;

    case FaceExpr::kAngry:
      g.drawLine(kFaceCx - eye_dx - eye_r, eye_y - eye_r - 2, kFaceCx - eye_dx + eye_r,
                 eye_y - eye_r + 4, TFT_BLACK);
      g.drawLine(kFaceCx + eye_dx + eye_r, eye_y - eye_r - 2, kFaceCx + eye_dx - eye_r,
                 eye_y - eye_r + 4, TFT_BLACK);
      draw_eye(g, kFaceCx - eye_dx, eye_y, eye_r, false);
      draw_eye(g, kFaceCx + eye_dx, eye_y, eye_r, false);
      draw_mouth_flat(g, kFaceCx, mouth + 2, mouth_hw - 4);
      break;

    case FaceExpr::kSurprise:
      draw_eye(g, kFaceCx - eye_dx, eye_y, eye_r + 2, false);
      draw_eye(g, kFaceCx + eye_dx, eye_y, eye_r + 2, false);
      g.drawCircle(kFaceCx, mouth, kFaceR / 8, TFT_BLACK);
      break;

    case FaceExpr::kSad:
    case FaceExpr::kLost:
      draw_eye(g, kFaceCx - eye_dx, eye_y, eye_r, expr == FaceExpr::kLost);
      draw_eye(g, kFaceCx + eye_dx, eye_y, eye_r, expr == FaceExpr::kLost);
      if (expr == FaceExpr::kLost) {
        draw_mouth_flat(g, kFaceCx, mouth, mouth_hw);
      } else {
        draw_mouth_frown(g, kFaceCx, mouth, mouth_arc_r, tint);
      }
      break;

    case FaceExpr::kConnect:
      draw_eye(g, kFaceCx - eye_dx, eye_y, eye_r, blink);
      draw_eye(g, kFaceCx + eye_dx, eye_y, eye_r, blink);
      draw_mouth_flat(g, kFaceCx, mouth, mouth_hw / 2);
      break;

    case FaceExpr::kExploreWait: {
      const int lx = (eye_r / 2 + 2);
      const int ly = -(eye_r / 2 + 2);
      draw_eye_look(g, kFaceCx - eye_dx, eye_y, eye_r, lx, ly, blink);
      draw_eye_look(g, kFaceCx + eye_dx, eye_y, eye_r, lx, ly, blink);
      draw_mouth_flat(g, kFaceCx, mouth, mouth_hw / 2);
      break;
    }

    case FaceExpr::kWait: {
      const int lx = -(eye_r / 2 + 2);
      const int ly = -(eye_r / 2 + 2);
      draw_eye_look(g, kFaceCx - eye_dx, eye_y, eye_r, lx, ly, blink);
      draw_eye_look(g, kFaceCx + eye_dx, eye_y, eye_r, lx, ly, blink);
      draw_mouth_flat(g, kFaceCx, mouth, mouth_hw / 2);
      break;
    }

    case FaceExpr::kBoot:
      draw_eye(g, kFaceCx - eye_dx, eye_y, eye_r, blink);
      draw_eye(g, kFaceCx + eye_dx, eye_y, eye_r, blink);
      draw_mouth_flat(g, kFaceCx, mouth, mouth_hw / 2);
      break;

    case FaceExpr::kHappy:
    default:
      draw_eye(g, kFaceCx - eye_dx, eye_y, eye_r, blink);
      draw_eye(g, kFaceCx + eye_dx, eye_y, eye_r, blink);
      draw_mouth_smile(g, kFaceCx, mouth, mouth_arc_r, tint);
      break;
  }
}

static const char *stable_mode_label(const char *raw, uint32_t now_ms) {
  static char shown[24] = "boot";
  static char pending[24] = "boot";
  static uint32_t pending_since = 0;

  if (!raw) raw = "?";

  if (strncmp(raw, pending, sizeof(pending)) != 0) {
    strncpy(pending, raw, sizeof(pending) - 1);
    pending[sizeof(pending) - 1] = '\0';
    pending_since = now_ms;
    const bool urgent = strcmp(raw, "E-STOP") == 0 || strcmp(raw, "low batt") == 0 ||
                        strncmp(raw, "Explore", 7) == 0;
    if (urgent) {
      strncpy(shown, pending, sizeof(shown) - 1);
      shown[sizeof(shown) - 1] = '\0';
    }
  }

  if (now_ms - pending_since >= 4000) {
    strncpy(shown, pending, sizeof(shown) - 1);
    shown[sizeof(shown) - 1] = '\0';
  }
  return shown;
}

static const char *drive_mode_name(uint8_t mode) {
  return mode == 1 ? "Wall" : "Explore";
}

static const char *power_action_name(uint8_t action) {
  switch (action) {
    case 1:
      return "Reboot";
    case 2:
      return "Cancel";
    default:
      return "Shutdown";
  }
}

static void print_range_value(TFT_eSprite &g, float range_m) {
  if (range_m < 0.01f) {
    g.print("---");
    return;
  }
  if (range_m < 2.0f) {
    g.printf("%dcm", static_cast<int>(range_m * 100.0f + 0.5f));
  } else {
    g.printf("%.1fm", range_m);
  }
}

static float snap_range_near(const RoverUiData &data, float pan_deg) {
  float best = -1.0f;
  float best_err = 999.0f;
  for (uint8_t i = 0; i < data.sonar_cal_snap_count; i++) {
    const float err = fabsf(data.sonar_cal_snap_pan[i] - pan_deg);
    if (err < best_err) {
      best_err = err;
      best = data.sonar_cal_snap_range[i];
    }
  }
  return (best_err <= 12.0f) ? best : -1.0f;
}

static void draw_stats(TFT_eSprite &g, const RoverUiData &data, int y0, uint32_t now_ms) {
  int y = y0;

  g.setTextSize(2);
  g.setTextColor(TFT_CYAN, TFT_BLACK);
  g.setCursor(kTextX, y);
  if (data.mode_menu) {
    g.print("MODE MENU");
  } else {
    g.print(stable_mode_label(data.mode, now_ms));
  }
  y += 22;

  if (data.boot_reason && data.boot_reason_until_ms > now_ms) {
    g.setTextSize(1);
    g.setTextColor(TFT_ORANGE, TFT_BLACK);
    g.setCursor(kTextX, y);
    g.print(data.boot_reason);
    y += 12;
    g.setTextSize(2);
  }

  if (!data.session_active && !data.mode_menu && data.link_state == 2) {
    g.setTextSize(2);
    g.setTextColor(TFT_YELLOW, TFT_BLACK);
    g.setCursor(kTextX, y);
    g.printf("Mode: %s", drive_mode_name(data.drive_mode));
    y += 22;

    if (data.menu_hint_until_ms > now_ms) {
      g.setTextColor(TFT_ORANGE, TFT_BLACK);
      g.setCursor(kTextX, y);
      g.print("Tap Go -->");
      y += 22;
    }
  }

  g.setTextSize(2);
  g.setTextColor(TFT_WHITE, TFT_BLACK);
  g.setCursor(kTextX, y);
  g.printf("Link %s", link_label(data.link_state));
  y += kLineH2;

  g.setTextColor(bat_colour(data), TFT_BLACK);
  g.setCursor(kTextX, y);
  g.printf("BAT %.1fV%s", data.bat_v, data.bat_critical ? "!" : (data.bat_low ? "?" : ""));
  y += kLineH2;

#if defined(ROVER_VL53L)
  {
    const bool any_ok = data.tof_left_ok || data.tof_right_ok;
    g.setTextColor(any_ok ? TFT_GREEN : TFT_RED, TFT_BLACK);
    g.setCursor(kTextX, y);
    g.print("ToF L ");
    print_range_value(g, data.tof_left_ok ? data.tof_left_m : -1.0f);
    g.print("  R ");
    print_range_value(g, data.tof_right_ok ? data.tof_right_m : -1.0f);
    y += kLineH2;
  }
#endif

  g.setTextColor(TFT_WHITE, TFT_BLACK);
  g.setCursor(kTextX, y);
  g.printf("L %.2f  R %.2f", data.motor_l, data.motor_r);
  y += kLineH2;

  g.setCursor(kTextX, y);
  g.printf("v %.2f  w %.2f", data.cmd_lin, data.cmd_ang);
  y += kLineH2;

  if (data.sonar_enabled) {
    const bool show_summary =
        data.sonar_cal_summary_until_ms > now_ms && data.sonar_cal_snap_count >= 3;
    if (show_summary) {
      const float left_m = snap_range_near(data, 180.0f);
      const float right_m = snap_range_near(data, 0.0f);
      const float fwd_m = snap_range_near(data, 90.0f);
      g.setTextColor(TFT_YELLOW, TFT_BLACK);
      g.setCursor(kTextX, y);
      g.print("L ");
      print_range_value(g, left_m);
      g.print("  R ");
      print_range_value(g, right_m);
      g.print("  F ");
      print_range_value(g, fwd_m);
      y += kLineH2;
    } else if (data.sonar_cal_active) {
      g.setTextColor(TFT_YELLOW, TFT_BLACK);
      g.setCursor(kTextX, y);
      g.print("SCAN ");
      print_range_value(g, data.sonar_range_m);
      g.printf(" @%.0f", data.sonar_pan_deg);
      y += kLineH2;
      if (data.sonar_cal_snap_count > 0) {
        g.setTextSize(1);
        g.setTextColor(TFT_CYAN, TFT_BLACK);
        g.setCursor(kTextX, y);
        g.print("got ");
        for (uint8_t i = 0; i < data.sonar_cal_snap_count; i++) {
          if (i > 0) {
            g.print(" ");
          }
          g.printf("%.0f:", data.sonar_cal_snap_pan[i]);
          print_range_value(g, data.sonar_cal_snap_range[i]);
        }
        y += 12;
        g.setTextSize(2);
      }
    } else {
      uint16_t rng_col = TFT_CYAN;
      if (data.sonar_range_m > 0.01f) {
        if (data.sonar_range_m < 0.15f) {
          rng_col = TFT_RED;
        } else if (data.sonar_range_m < 0.30f) {
          rng_col = TFT_YELLOW;
        }
      } else {
        rng_col = TFT_DARKGREY;
      }
      g.setTextColor(rng_col, TFT_BLACK);
      g.setCursor(kTextX, y);
      g.print("Pan ");
      g.printf("%3.0f  ", data.sonar_pan_deg);
      print_range_value(g, data.sonar_range_m);
      y += kLineH2;
    }
  }

  if (data.stall || data.bump) {
    g.setTextColor(TFT_YELLOW, TFT_BLACK);
    g.setCursor(kTextX, y);
    g.printf("%s%s", data.bump ? "BUMP " : "", data.stall ? "STALL" : "");
    y += kLineH2;
  }

  if (data.ir_enabled) {
    g.setTextSize(1);
#if defined(ROVER_UI_IR_DEBUG)
    if (!data.ir_ok) {
      g.setTextColor(TFT_RED, TFT_BLACK);
      g.setCursor(kTextX, y);
      g.printf("IR FAIL f:%s b:%s", data.ir_in_ok ? "OK" : "--",
               data.ir_out_ok ? "OK" : "--");
    } else {
      g.setTextColor(data.ir_front_hit ? TFT_YELLOW : TFT_GREEN, TFT_BLACK);
      g.setCursor(kTextX, y);
      g.printf("IR f:%s b:%s", data.ir_in_ok ? "OK" : "--", data.ir_out_ok ? "OK" : "--");
      if (data.ir_front_hit) {
        g.print(" HIT");
      } else if (data.ir_front_emit) {
        g.print(" TX");
      }
    }
    y += 12;

    g.setTextColor(TFT_ORANGE, TFT_BLACK);
    g.setCursor(kTextX, y);
    g.printf("I2C SDA%d SCL%d", data.ir_sda_pin, data.ir_scl_pin);
    if (data.ir_i2c_count == 0) {
      g.print(" --");
    } else {
      for (uint8_t i = 0; i < data.ir_i2c_count && i < 8; i++) {
        g.printf(" %02X", data.ir_i2c_addrs[i]);
      }
    }
    y += 12;

    if (data.ir_ok) {
      g.setTextColor(TFT_WHITE, TFT_BLACK);
      g.setCursor(kTextX, y);
      g.printf("FL%c FR%c", (data.ir_inputs & 0x01) ? '1' : '0',
               (data.ir_inputs & 0x02) ? '1' : '0');
      g.setTextColor(TFT_DARKGREY, TFT_BLACK);
      g.printf(" WL%c WR%c AX%c", (data.ir_inputs & 0x04) ? '1' : '0',
               (data.ir_inputs & 0x08) ? '1' : '0', (data.ir_inputs & 0x10) ? '1' : '0');
      if (data.ir_wheels_on) {
        g.setTextColor(TFT_CYAN, TFT_BLACK);
        g.print(" Whl");
      }
      if (data.ir_body_aux_emit) {
        g.setTextColor(TFT_YELLOW, TFT_BLACK);
        g.print(" aTX");
      }
      g.setTextColor(TFT_DARKGREY, TFT_BLACK);
      g.printf(" bD:%02X L:%02X P:%02X", data.ir_body_iodir, data.ir_body_olat,
               data.ir_body_gpio);
      if (!data.ir_front_emit && data.ir_in_ok) {
        y += 12;
        g.setTextColor(TFT_ORANGE, TFT_BLACK);
        g.setCursor(kTextX, y);
        g.printf("MCP D:%02X L:%02X P:%02X", data.ir_front_iodir, data.ir_front_olat,
                 data.ir_front_gpio);
      }
    }
#else
    if (!data.ir_ok) {
      g.setTextColor(TFT_RED, TFT_BLACK);
      g.setCursor(kTextX, y);
      g.printf("IR fault f:%s b:%s", data.ir_in_ok ? "OK" : "--",
               data.ir_out_ok ? "OK" : "--");
      y += 12;
    } else if (data.ir_front_hit) {
      g.setTextColor(TFT_YELLOW, TFT_BLACK);
      g.setCursor(kTextX, y);
      g.print("IR bumper HIT");
      y += 12;
    }
#endif
    g.setTextSize(2);
  }
}

static void draw_estop_hint(TFT_eSprite &g) {
  g.setTextDatum(BR_DATUM);
  g.setTextColor(TFT_ORANGE, TFT_BLACK);
  g.setTextSize(1);
  g.drawString("E Stop -->", kScrW, kScrH);
  g.setTextDatum(TL_DATUM);
}

static void draw_mode_menu_screen(TFT_eSprite &g, const RoverUiData &data) {
  g.fillRect(0, 0, kScrW, kScrH, TFT_NAVY);
  g.drawRect(0, 0, kScrW, kScrH, TFT_CYAN);
  g.drawRect(2, 2, kScrW - 4, kScrH - 4, TFT_CYAN);

  g.setTextDatum(MC_DATUM);
  g.setTextColor(TFT_WHITE, TFT_NAVY);
  g.setTextSize(2);
  g.drawString("SELECT MODE", kScrW / 2, 28);

  g.setTextColor(TFT_YELLOW, TFT_NAVY);
  g.setTextSize(4);
  g.drawString(drive_mode_name(data.drive_mode), kScrW / 2, 82);

  g.setTextColor(TFT_CYAN, TFT_NAVY);
  g.setTextSize(1);
  g.drawString(
    (data.drive_mode == 0) ? "1/2  Explore" : "2/2  Wall follow", kScrW / 2, 118);

  g.setTextColor(TFT_DARKGREY, TFT_NAVY);
  g.setTextSize(2);
  g.drawString("TAP = next", kScrW / 2, 142);
  g.drawString("HOLD = done", kScrW / 2, 162);
  g.setTextDatum(TL_DATUM);
}

static void draw_power_menu_screen(TFT_eSprite &g, const RoverUiData &data) {
  g.fillRect(0, 0, kScrW, kScrH, TFT_MAROON);
  g.drawRect(0, 0, kScrW, kScrH, TFT_RED);
  g.drawRect(2, 2, kScrW - 4, kScrH - 4, TFT_RED);

  g.setTextDatum(MC_DATUM);
  g.setTextColor(TFT_WHITE, TFT_MAROON);
  g.setTextSize(2);
  g.drawString("POWER", kScrW / 2, 24);

  g.setTextColor(TFT_YELLOW, TFT_MAROON);
  g.setTextSize(4);
  g.drawString(power_action_name(data.power_action), kScrW / 2, 82);

  g.setTextColor(TFT_ORANGE, TFT_MAROON);
  g.setTextSize(1);
  g.drawString("Board power only", kScrW / 2, 118);

  g.setTextColor(TFT_DARKGREY, TFT_MAROON);
  g.setTextSize(2);
  g.drawString("TAP = next", kScrW / 2, 142);
  g.drawString("HOLD = confirm", kScrW / 2, 162);
  g.setTextDatum(TL_DATUM);
}

static void draw_action_hint(TFT_eSprite &g, const RoverUiData &data) {
  if (data.link_state != 2 || data.estop || !data.pi_bridge_live) return;
  if (data.mode_menu || data.power_menu) return;

  g.setTextDatum(TR_DATUM);
  g.setTextColor(TFT_ORANGE, TFT_BLACK);
  g.setTextSize(1);
  g.drawString(data.session_active ? "Stop -->" : "Go -->", kScrW, 0);
  g.setTextDatum(TL_DATUM);
}

static void draw_key_flash(TFT_eSprite &g, const RoverUiData &data, uint32_t now_ms) {
  if (data.key_flash_until_ms <= now_ms) return;
  g.drawRect(0, 0, kScrW, kScrH, TFT_WHITE);
  g.drawRect(3, 3, kScrW - 6, kScrH - 6, TFT_YELLOW);
}

static void draw_tap_hint(TFT_eSprite &g, const RoverUiData &data) {
  if (data.tap_progress == 0 || data.mode_menu || data.power_menu || data.session_active) return;
  char buf[20];
  snprintf(buf, sizeof(buf), "Go taps: %u/%u", (unsigned)data.tap_progress,
           (unsigned)ROVER_GO_MULTI_TAPS);
  g.setTextDatum(BC_DATUM);
  g.setTextColor(TFT_YELLOW, TFT_BLACK);
  g.setTextSize(1);
  g.drawString(buf, kScrW / 2, kScrH - 2);
  g.setTextDatum(TL_DATUM);
}

static void render_frame(TFT_eSprite &g, const RoverUiData &data, uint32_t now_ms) {
  g.fillSprite(TFT_BLACK);

  if (data.estop) {
    draw_face(g, FaceExpr::kEstop, now_ms);
    g.setTextColor(TFT_RED, TFT_BLACK);
    g.setTextSize(3);
    g.setCursor(kTextX, 28);
    g.println("E-STOP");
    g.setTextSize(2);
    g.setCursor(kTextX, 62);
    g.println("release btn");
#if defined(ENABLE_OTA)
    draw_wifi_icon(g, data.ota_state);
#endif
    draw_estop_hint(g);
    return;
  }

  if (data.power_menu) {
    draw_power_menu_screen(g, data);
    draw_key_flash(g, data, now_ms);
    return;
  }

  if (data.mode_menu) {
    draw_mode_menu_screen(g, data);
    draw_key_flash(g, data, now_ms);
    return;
  }

  draw_face(g, face_for(data), now_ms);
  draw_stats(g, data, 8, now_ms);

#if defined(ENABLE_OTA)
  draw_wifi_icon(g, data.ota_state);
#endif

  draw_action_hint(g, data);
  draw_estop_hint(g);
  draw_tap_hint(g, data);
  draw_key_flash(g, data, now_ms);
}

}  // namespace

bool RoverDisplay::_changed(const RoverUiData &data) const {
  if (static_cast<uint8_t>(face_for(data)) != _last_face) return true;
  if (data.estop != _last.estop) return true;
  if (data.session_active != _last.session_active) return true;
  if (data.mode_menu != _last.mode_menu) return true;
  if (data.power_menu != _last.power_menu) return true;
  if (data.drive_mode != _last.drive_mode) return true;
  if (data.power_action != _last.power_action) return true;
  if (data.tap_progress != _last.tap_progress) return true;
  if ((data.key_flash_until_ms > 0) != (_last.key_flash_until_ms > 0)) return true;
  if ((data.menu_hint_until_ms > 0) != (_last.menu_hint_until_ms > 0)) return true;
  if (data.pi_bridge_live != _last.pi_bridge_live) return true;
  if (data.bump || data.stall) return true;
  if (data.link_state != _last.link_state) return true;
  if (data.ota_state != _last.ota_state) return true;
  if (data.bat_low != _last.bat_low || data.bat_critical != _last.bat_critical) return true;
  // Screen shows 1 decimal (0.1V). feq()'s 5mV tolerance is far finer than
  // that, so raw ADC noise was tripping this on almost every loop, forcing a
  // full SPI sprite redraw ~every 100ms and blocking the whole control loop
  // (including the sonar pan servo update) for tens of ms each time — that
  // was the real cause of the "jerky" pan sweep, not the PCA9685 or I2C.
  if (fabsf(data.bat_v - _last.bat_v) >= 0.05f) return true;
  if (!feq(data.cmd_lin, _last.cmd_lin) || !feq(data.cmd_ang, _last.cmd_ang)) return true;
  if (!feq(data.motor_l, _last.motor_l) || !feq(data.motor_r, _last.motor_r)) return true;
  if (data.ir_enabled != _last.ir_enabled || data.ir_ok != _last.ir_ok) return true;
  if (data.ir_in_ok != _last.ir_in_ok || data.ir_out_ok != _last.ir_out_ok) return true;
  if (data.ir_inputs != _last.ir_inputs) return true;
  if (data.ir_front_hit != _last.ir_front_hit) return true;
  if (data.ir_wheels_on != _last.ir_wheels_on) return true;
  if (data.ir_front_emit != _last.ir_front_emit) return true;
  if (data.ir_body_aux_emit != _last.ir_body_aux_emit) return true;
  if (data.ir_body_iodir != _last.ir_body_iodir || data.ir_body_olat != _last.ir_body_olat ||
      data.ir_body_gpio != _last.ir_body_gpio) {
    return true;
  }
  if (data.ir_front_iodir != _last.ir_front_iodir || data.ir_front_olat != _last.ir_front_olat ||
      data.ir_front_gpio != _last.ir_front_gpio) {
    return true;
  }
  if (data.ir_i2c_count != _last.ir_i2c_count) return true;
  for (uint8_t i = 0; i < 8; i++) {
    if (data.ir_i2c_addrs[i] != _last.ir_i2c_addrs[i]) return true;
  }
  if (data.ir_sda_pin != _last.ir_sda_pin || data.ir_scl_pin != _last.ir_scl_pin) return true;
  if (data.sonar_enabled != _last.sonar_enabled) return true;
  if (data.sonar_cal_active != _last.sonar_cal_active) return true;
  if (fabsf(data.sonar_pan_deg - _last.sonar_pan_deg) >= 2.0f) return true;
  if (fabsf(data.sonar_range_m - _last.sonar_range_m) >= 0.02f) return true;
#if defined(ROVER_VL53L)
  if (data.tof_left_ok != _last.tof_left_ok || data.tof_right_ok != _last.tof_right_ok) return true;
#else
  if (data.tof_enabled != _last.tof_enabled) return true;
#endif
  if (fabsf(data.tof_left_m - _last.tof_left_m) >= 0.01f) return true;
  if (fabsf(data.tof_right_m - _last.tof_right_m) >= 0.01f) return true;
  if (data.sonar_cal_snap_count != _last.sonar_cal_snap_count) return true;
  if (data.sonar_cal_summary_until_ms > 0 &&
      (data.sonar_cal_summary_until_ms > 0) != (_last.sonar_cal_summary_until_ms > 0)) {
    return true;
  }
  for (uint8_t i = 0; i < data.sonar_cal_snap_count && i < 4; i++) {
    if (fabsf(data.sonar_cal_snap_range[i] - _last.sonar_cal_snap_range[i]) >= 0.02f) {
      return true;
    }
  }
  if (data.boot_reason_until_ms > 0 &&
      ((data.boot_reason_until_ms > 0) != (_last.boot_reason_until_ms > 0))) {
    return true;
  }
  const char *mode = data.mode ? data.mode : "?";
  return strncmp(mode, _last_mode, sizeof(_last_mode)) != 0;
}

bool RoverDisplay::begin() {
  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);
  pinMode(PIN_LCD_BL, OUTPUT);
  digitalWrite(PIN_LCD_BL, HIGH);

  tft.init();
  tft.setRotation(1);  // landscape 320x170

  sprite.setColorDepth(16);
  _sprite_ok = sprite.createSprite(kScrW, kScrH);
  if (!_sprite_ok) {
    Serial.println("FAIL: display sprite alloc (need PSRAM)");
    return false;
  }

  RoverUiData boot{};
  boot.mode = "boot";
  render_frame(sprite, boot, 0);
  sprite.pushSprite(0, 0);

  _last = boot;
  _last_face = static_cast<uint8_t>(FaceExpr::kBoot);
  strncpy(_last_mode, "boot", sizeof(_last_mode) - 1);
  _ok = true;
  return true;
}

void RoverDisplay::draw(const RoverUiData &data, uint32_t now_ms) {
  if (!_ok || !_sprite_ok) return;

#ifndef ROVER_DISPLAY_MIN_MS
#define ROVER_DISPLAY_MIN_MS 1000
#endif
  if (now_ms - _last_draw < ROVER_DISPLAY_MIN_MS) return;
  _last_draw = now_ms;

  render_frame(sprite, data, now_ms);
  sprite.pushSprite(0, 0);

  _last = data;
  _last_face = static_cast<uint8_t>(face_for(data));
  strncpy(_last_mode, data.mode ? data.mode : "?", sizeof(_last_mode) - 1);
  _last_mode[sizeof(_last_mode) - 1] = '\0';
}

#else

bool RoverDisplay::begin() { return false; }

void RoverDisplay::draw(const RoverUiData &, uint32_t) {}

#endif
