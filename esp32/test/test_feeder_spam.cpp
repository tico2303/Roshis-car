#include <Arduino.h>
#include "feeder.h"
#include "robot_config.h"
static Feeder feeder(RobotConfig::feederConfig);

void setup() {
  Serial.begin(115200);
  delay(500);

  feeder.begin();

  Serial.println();
  Serial.println("=== Feeder Spam Test ===");
  Serial.println("This simulates repeated open() calls.");
  Serial.println("Expected: opens, schedules close once, closes ~700ms later.");
  Serial.println("========================");
}

void loop() {
  static uint32_t t0 = millis();
  static bool didSpam = false;

  feeder.update();

  // Spam the open feeder button to test
  if (!didSpam && (millis() - t0) > 1000) {
    Serial.println("[TEST] Spamming open(700) for 1 second...");

    uint32_t start = millis();
    while ((millis() - start) < 1000) {
      feeder.open(400);
      delay(100); // 50Hz spam
      feeder.update();
    }

    Serial.println("[TEST] Done spamming. Now wait for auto-close...");
    didSpam = true;
  }

  // keep loop alive
  delay(5);
}