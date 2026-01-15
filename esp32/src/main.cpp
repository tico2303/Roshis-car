#include <Arduino.h>
#include "protocol.h"

static const uint32_t BAUD = 115200;

Protocol proto(Serial);

void setup() {
  Serial.begin(BAUD);
  delay(500);
  proto.sendBoot();
}

void loop() {
  proto.poll();
}