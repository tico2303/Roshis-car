#include <Wire.h>
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(21, 22); // set explicit pins
  Serial.println("Scanning...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("Found 0x%02X\n", addr);
      found++;
    }
  }
  Serial.printf("Done. Found: %d\n", found);
}

void loop() {}