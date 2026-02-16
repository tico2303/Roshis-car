#include <Arduino.h>
#include "robot_config.h"
#include "i2c_bus.h"
#include "protocol.h"
#include "bmi160_imu.h"

static I2CBus   i2c(RobotConfig::I2C);
static Protocol proto(Serial);
static Bmi160Imu imu;  // default addr 0x68 (SDO -> GND), use 0x69 if SDO -> VCC

static uint32_t nextDiagMs = 0;
static uint32_t nextScanMs = 0;

void setup() {
  //bring motors down
  pinMode(RobotConfig::RIGHT_PINS.in1, OUTPUT);
  digitalWrite(RobotConfig::RIGHT_PINS.in1, LOW);
  pinMode(RobotConfig::RIGHT_PINS.in2, OUTPUT);
  digitalWrite(RobotConfig::RIGHT_PINS.in2, LOW);
  //end
  Serial.begin(RobotConfig::SERIAL_BUAD_RATE);
  delay(200);

  Serial.println();
  Serial.println("=== BMI160 IMU Test ===");

  if (!i2c.begin()) {
    Serial.println("I2C begin FAILED");
  } else {
    Serial.println("I2C begin OK");
  }

  i2c.scan(Serial);

  Serial.print("BMI160 begin ... ");
  if (imu.begin(i2c)) {
    Serial.println("OK");
  } else {
    Serial.println("FAIL - check wiring (SDA=21, SCL=22, VCC=3.3V)");
  }

  imu.setPeriodMs(50);  // 20Hz for test output readability

  uint32_t now = millis();
  nextDiagMs = now + 5000;
  nextScanMs = now + 15000;
}

void loop() {
  proto.poll();

  if (imu.poll()) {
    // Publish via protocol (NDJSON over serial, ROS2-ready format)
    imu.publish(proto);

    // Also print human-readable for debugging
    const Bmi160Reading& r = imu.last();
    Serial.printf("[IMU] ax=%.3f ay=%.3f az=%.3f  gx=%.3f gy=%.3f gz=%.3f  t=%lu\n",
                  r.ax, r.ay, r.az, r.gx, r.gy, r.gz, r.ts_ms);
  }

  uint32_t now = millis();

  if ((int32_t)(now - nextDiagMs) >= 0) {
    nextDiagMs = now + 5000;
    Serial.println();
    Serial.print("=== IMU STATUS: ");
    Serial.println(imu.ok() ? "OK ===" : "NOT OK ===");
  }

  if ((int32_t)(now - nextScanMs) >= 0) {
    nextScanMs = now + 15000;
    Serial.println("=== I2C RESCAN ===");
    i2c.scan(Serial);
  }
}
