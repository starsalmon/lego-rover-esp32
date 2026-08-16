#include <Arduino.h>
#include "rover_board_power.h"

void setup() {
  rover_board_power_on();
  Serial.begin(115200);
  delay(1500);
  Serial.println("s3_min alive");
}

void loop() {
  Serial.println("tick");
  delay(1000);
}
