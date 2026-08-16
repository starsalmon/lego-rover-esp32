#include "rover_telem.h"

#include <Arduino.h>
#include <stdio.h>

namespace {

void (*g_sink)(const char*) = nullptr;
char g_buf[280];

}  // namespace

void rover_telem_set_sink(void (*fn)(const char*)) { g_sink = fn; }

void rover_telem_printf(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(g_buf, sizeof(g_buf), fmt, ap);
  va_end(ap);
  if (n <= 0) {
    return;
  }
  Serial.print(g_buf);
  if (g_sink) {
    g_sink(g_buf);
  }
}
