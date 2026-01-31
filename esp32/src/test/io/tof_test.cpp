#include <Arduino.h>

#include "robot_config.h"
#include "i2c_bus.h"
#include "tof/tof_manager.h"
#include "protocol.h"

// One bus, one ToF manager, one protocol.
static I2CBus     i2c(RobotConfig::I2C);
static TofManager tof(RobotConfig::TOF_CFG, RobotConfig::TOF_CFG_COUNT);
static Protocol   proto(Serial);

void setup() {
  Serial.begin(RobotConfig::SERIAL_BUAD_RATE);
  delay(50);

  // Bring up I2C and ToF
  i2c.begin();
  tof.begin(i2c);

  // Optional: quick bus sanity check
  // i2c.scan(Serial);
}

void loop() {
  proto.poll();

  if (tof.poll()) {
    tof.publish(proto);
  }
}