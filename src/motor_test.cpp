#include <Arduino.h>
#include <ctype.h>
#include <string.h>
#include "drive.h"
#include "rover_board_power.h"

#ifdef ROVER_TDISPLAY_S3
#include "ir_tx.h"
#include "rover_pins_s3.h"
#endif

#ifndef DIR_L
#define DIR_L 0
#define PWM_L 1
#define DIR_R 3
#define PWM_R 2
#endif

SmoothDrive drv;

#ifdef ROVER_TDISPLAY_S3
static IrTx g_ir_tx;
static bool g_ir_dc = false;

static void ir_dc_mode(bool on) {
  g_ir_tx.set_enabled(false);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcDetach(static_cast<uint8_t>(IR_TX_PIN));
#else
  ledcDetachPin(static_cast<uint8_t>(IR_TX_PIN));
#endif
  pinMode(IR_TX_PIN, OUTPUT);
  digitalWrite(IR_TX_PIN, on ? HIGH : LOW);
  g_ir_dc = on;
}
#endif

static uint8_t g_duty_l = 0;
static uint8_t g_duty_r = 0;
static bool g_rev_l = false;
static bool g_rev_r = false;

static void apply_state() {
  drv.apply_duty_lr(g_duty_l, g_rev_l, g_duty_r, g_rev_r);
}

static void print_help() {
  Serial.println();
  Serial.println("Motor cal — Makerverse TC78H660 PWM+DIR");
  Serial.println("  A=left  B=right  DIR high + PWM = forward");
  Serial.printf("  Production: %u Hz, dead-zone remap L=%.0f%% R=%.0f%% (tune in platformio.ini)\n",
                (unsigned)PWM_FREQ_HZ, MIN_PWM_FLOOR_L * 100.0f, MIN_PWM_FLOOR_R * 100.0f);
  Serial.println();
  Serial.println("  L <pct> [rev]     left  0-100% duty  (also L60)");
  Serial.println("  R <pct> [rev]     right");
  Serial.println("  B <pct> [rev]     both");
  Serial.println("  raw L <0-255> [rev]   direct PWM count");
  Serial.println("  freq <hz>         change PWM (production 30000; try 20000–30000)");
  Serial.println("  stop / status / scan L 5");
#ifdef ROVER_TDISPLAY_S3
  Serial.printf("  I / i — GPIO%d IR LED DC on/off (phone camera)\n", IR_TX_PIN);
  Serial.printf("  T — toggle %u Hz PWM beacon on GPIO%d\n", (unsigned)IR_TX_HZ, IR_TX_PIN);
#endif
  Serial.println();
}

static void print_status() {
  Serial.printf("status @ PWM freq (see boot line):\n");
  Serial.printf("  left (A): duty=%u/255 dir=%s\n", g_duty_l, g_rev_l ? "rev" : "fwd");
  Serial.printf("  right (B): duty=%u/255 dir=%s\n", g_duty_r, g_rev_r ? "rev" : "fwd");
}

static void set_side(char side, int pct, bool rev) {
  pct = constrain(pct, 0, 100);
  const uint8_t duty = (uint8_t)((pct * 255 + 50) / 100);
  if (side == 'L') {
    g_duty_l = duty;
    g_rev_l = rev;
  } else if (side == 'R') {
    g_duty_r = duty;
    g_rev_r = rev;
  } else if (side == 'B') {
    g_duty_l = g_duty_r = duty;
    g_rev_l = g_rev_r = rev;
  }
  apply_state();
  print_status();
}

static void trim_inplace(char *s) {
  char *p = s;
  while (*p && isspace((unsigned char)*p)) p++;
  if (p != s) memmove(s, p, strlen(p) + 1);
  size_t n = strlen(s);
  while (n > 0 && isspace((unsigned char)s[n - 1])) s[--n] = '\0';
}

static bool parse_rev(const char *word) {
  return word && (strcasecmp(word, "rev") == 0 || strcasecmp(word, "r") == 0
                  || strcasecmp(word, "back") == 0);
}

static void handle_line(char *line) {
  trim_inplace(line);
  if (line[0] == '\0') return;

  char a0 = (char)toupper((unsigned char)line[0]);
  int val = 0;
  char word[16] = {};

  if (!strcasecmp(line, "help") || line[0] == '?') {
    print_help();
    return;
  }
#ifdef ROVER_TDISPLAY_S3
  if (strcasecmp(line, "I") == 0) {
    ir_dc_mode(true);
    Serial.printf("GPIO%d HIGH — IR LED should glow in phone camera\n", IR_TX_PIN);
    return;
  }
  if (strcasecmp(line, "i") == 0) {
    ir_dc_mode(false);
    Serial.printf("GPIO%d LOW\n", IR_TX_PIN);
    return;
  }
  if (strcasecmp(line, "T") == 0) {
    if (g_ir_dc || !g_ir_tx.ok()) {
      g_ir_dc = false;
      if (!g_ir_tx.begin(IR_TX_PIN, IR_TX_HZ, IR_TX_LEDC_CHANNEL)) {
        Serial.printf("IR PWM init failed GPIO%d\n", IR_TX_PIN);
        return;
      }
      g_ir_tx.set_enabled(true);
      Serial.printf("IR PWM GPIO%d: ON\n", IR_TX_PIN);
      return;
    }
    g_ir_tx.set_enabled(!g_ir_tx.enabled());
    Serial.printf("IR PWM GPIO%d: %s\n", IR_TX_PIN, g_ir_tx.enabled() ? "ON" : "off");
    return;
  }
#endif
  if (!strcasecmp(line, "stop") || !strcasecmp(line, "s") || !strcasecmp(line, "off")) {
    g_duty_l = g_duty_r = 0;
    apply_state();
    Serial.println("stop");
    return;
  }
  if (!strcasecmp(line, "status")) {
    print_status();
    return;
  }

  if (!strncasecmp(line, "freq ", 5)) {
    unsigned hz = 0;
    if (sscanf(line + 5, "%u", &hz) != 1) {
      Serial.println("usage: freq <hz>  (e.g. freq 200)");
      return;
    }
    if (drv.set_pwm_freq(hz)) {
      Serial.printf("PWM freq -> %u Hz\n", hz);
      apply_state();
    } else {
      Serial.println("freq set failed (8-400000)");
    }
    return;
  }

  if (!strncasecmp(line, "scan ", 5)) {
    char which = 0;
    int step = 0;
    if (sscanf(line + 5, " %c %d", &which, &step) < 2 || step < 1) {
      Serial.println("usage: scan <L|R|B> <step>");
      return;
    }
    which = (char)toupper((unsigned char)which);
    Serial.printf("scan %c step=%d%% (stop to abort)\n", which, step);
    for (int p = 0; p <= 100; p += step) {
      if (Serial.available()) {
        while (Serial.available()) Serial.read();
        Serial.println("scan aborted");
        return;
      }
      set_side(which, p, false);
      delay(1500);
    }
    Serial.println("scan done");
    return;
  }

  if (!strncasecmp(line, "raw ", 4)) {
    char which = 0;
    int duty = 0;
    if (sscanf(line + 4, " %c %d %15s", &which, &duty, word) < 2) {
      Serial.println("usage: raw L|R|B <0-255> [rev]");
      return;
    }
    which = (char)toupper((unsigned char)which);
    duty = constrain(duty, 0, 255);
    const bool rev = parse_rev(word);
    if (which == 'L') {
      g_duty_l = (uint8_t)duty;
      g_rev_l = rev;
    } else if (which == 'R') {
      g_duty_r = (uint8_t)duty;
      g_rev_r = rev;
    } else if (which == 'B') {
      g_duty_l = g_duty_r = (uint8_t)duty;
      g_rev_l = g_rev_r = rev;
    }
    apply_state();
    print_status();
    return;
  }

  // L60 / L 60 / R 30 rev
  if ((a0 == 'L' || a0 == 'R' || a0 == 'B') && isdigit((unsigned char)line[1])) {
    if (sscanf(line + 1, "%d %15s", &val, word) < 1) return;
    set_side(a0, val, parse_rev(word));
    return;
  }
  if ((a0 == 'L' || a0 == 'R' || a0 == 'B') && (line[1] == ' ' || line[1] == '\0')) {
    if (sscanf(line + 2, "%d %15s", &val, word) < 1) {
      Serial.println("usage: L|R|B <0-100> [rev]");
      return;
    }
    set_side(a0, val, parse_rev(word));
    return;
  }

  Serial.printf("unknown: %s (type help)\n", line);
}

void setup() {
  rover_board_power_on();
  SmoothDrive::gpio_safe_idle(DIR_L, PWM_L, DIR_R, PWM_R);

  Serial.begin(115200);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
#endif
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  delay(1500);  // USB CDC enumerate before first print
#else
  delay(300);
#endif
#ifdef ROVER_TDISPLAY_S3
  if (g_ir_tx.begin(IR_TX_PIN, IR_TX_HZ, IR_TX_LEDC_CHANNEL)) {
    g_ir_tx.set_enabled(false);
    Serial.printf("IR beacon ready on GPIO%d — press I for DC test, T for PWM\n", IR_TX_PIN);
  } else {
    Serial.printf("WARN: IR PWM init failed on GPIO%d\n", IR_TX_PIN);
  }
#endif
  if (!drv.begin(DIR_L, PWM_L, DIR_R, PWM_R)) {
    Serial.println("FAIL: LEDC attach");
  }
  apply_state();
  Serial.printf("motor_cal ready  DIR L=%d R=%d  PWM L=%d R=%d  @%uHz\n", DIR_L, DIR_R, PWM_L,
                PWM_R, (unsigned)PWM_FREQ_HZ);
  print_help();
}

void loop() {
#ifdef ROVER_TDISPLAY_S3
  if (!g_ir_dc) {
    g_ir_tx.tick();
  }
#endif
  static char buf[64];
  static size_t len = 0;

  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[len] = '\0';
      handle_line(buf);
      len = 0;
      continue;
    }
    if (len < sizeof(buf) - 1) buf[len++] = c;
  }
  delay(5);
}
