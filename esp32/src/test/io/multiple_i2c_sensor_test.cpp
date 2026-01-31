#include <Arduino.h>

#include "robot_config.h"
#include "i2c_bus.h"
#include "protocol.h"

#include "i2c_sensor.h"
#include "tof/tof_manager.h"


static I2CBus   i2c(RobotConfig::I2C);
static Protocol proto(Serial);

// -------------------- Sensors --------------------
// I2C sensors
static TofManager tof(RobotConfig::TOF_CFG, RobotConfig::TOF_CFG_COUNT);

// Sensory registry... add sensors here
static I2CSensor* sensors[] = {
  &tof,
  // &imu,
  // &ina226,
  // &oled,
};

static constexpr size_t SENSOR_COUNT = sizeof(sensors) / sizeof(sensors[0]);

// -------------------- Diagnostics timers --------------------
static uint32_t nextDiagMs = 0;
static uint32_t nextScanMs = 0;

static void printDiag() {
  Serial.println();
  Serial.println("=== SENSOR DIAG ===");

  for (size_t i = 0; i < SENSOR_COUNT; i++) {
    I2CSensor* s = sensors[i];
    Serial.print("  ");
    Serial.print(s->name());
    Serial.print(": ");
    Serial.println(s->ok() ? "OK" : "NOT OK");
  }
}

void setup() {
  Serial.begin(RobotConfig::SERIAL_BUAD_RATE);
  delay(100);

  Serial.println();
  Serial.println("=== sensors_test boot ===");

  // Bring up I2C bus first
  if (!i2c.begin()) {
    Serial.println("I2C begin FAILED");
  } else {
    Serial.println("I2C begin OK");
  }

  // Optional: scan once at boot
  i2c.scan(Serial);

  // Begin all sensors
  for (size_t i = 0; i < SENSOR_COUNT; i++) {
    I2CSensor* s = sensors[i];
    Serial.print("begin ");
    Serial.print(s->name());
    Serial.print(" ... ");

    bool ok = s->begin(i2c);
    Serial.println(ok ? "OK" : "FAIL");
  }

  // Start timers
  uint32_t now = millis();
  nextDiagMs = now + 2000;         // every 2s print status
  nextScanMs = now + 10000;        // every 10s scan bus (optional)
}

void loop() {
  proto.poll();

  // Poll + publish for each sensor
  for (size_t i = 0; i < SENSOR_COUNT; i++) {
    I2CSensor* s = sensors[i];

    if (s->poll()) {
      // publish() returns true if it emitted something
      s->publish(proto);
    }
  }

  // Diagnostics
  uint32_t now = millis();

  if ((int32_t)(now - nextDiagMs) >= 0) {
    nextDiagMs = now + 2000;
    printDiag();
  }

  // Optional: periodic scan if debugging is needed
  if ((int32_t)(now - nextScanMs) >= 0) {
    nextScanMs = now + 10000;
    Serial.println();
    Serial.println("=== I2C RESCAN ===");
    i2c.scan(Serial);
  }
}