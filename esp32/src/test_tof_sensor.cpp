#include <Arduino.h>
#include "tof_manager.h"
#include "robot_config.h"

static TofManager tof(
    RobotConfig::TOF_CFG,
  RobotConfig::TOF_CFG_COUNT / sizeof(RobotConfig::TOF_CFG[0])
);


void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(RobotConfig::I2C_SDA, RobotConfig::I2C_SCL);
  Wire.setClock(RobotConfig::WIRE_CLOCK);

  tof.setI2CPins(RobotConfig::I2C_SDA, RobotConfig::I2C_SCL);
  tof.setI2CHz(RobotConfig::WIRE_CLOCK);

  tof.setAutoRecover(RobotConfig::TOF_AUTORECOVERY);
  tof.setRecoverThreshold(RobotConfig::TOF_RECOVERY_THRESHOLD);

  tof.begin(Wire);
}

void loop() {
  TofReading r;

  if (tof.poll(r)) {
    Serial.print("tof[");
    Serial.print(r.id);
    Serial.print("] ");

    if (r.status == 0) {
      Serial.print(r.mm);
      Serial.print(" mm");
    } else {
      Serial.print("ERROR status=");
      Serial.print(r.status);
    }

    Serial.print("  ts=");
    Serial.print(r.ts_ms);
    Serial.println();
  }
}