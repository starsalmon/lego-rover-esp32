#pragma once

#include <stdarg.h>

/** Optional second sink (e.g. WiFi UDP) for debug lines. Always mirrors to Serial. */
void rover_telem_set_sink(void (*fn)(const char* line));
void rover_telem_printf(const char* fmt, ...);
