#include "rover_board_power.h"

#ifdef ROVER_TDISPLAY_S3

#include <Arduino.h>
#include "rover_pins_s3.h"

void rover_board_power_on() {
  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);
  pinMode(PIN_LCD_BL, OUTPUT);
  digitalWrite(PIN_LCD_BL, HIGH);
}

#else

void rover_board_power_on() {}

#endif
