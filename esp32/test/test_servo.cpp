#include <Arduino.h>
#include <ESP32Servo.h>

static Servo s;

void setup() {
  Serial.begin(115200);
  delay(500);

  s.setPeriodHertz(50);
  bool ok = s.attach(22, 500, 2500);

  Serial.print("attach ok? ");
  Serial.println(ok ? "YES" : "NO");

  Serial.println("Write 10 deg");
  s.write(10);
  delay(1200);

  Serial.println("Write 90 deg");
  s.write(90);
  delay(1200);

  Serial.println("Write 10 deg");
  s.write(10);
  delay(1200);

  Serial.println("Done");
}

void loop() {}