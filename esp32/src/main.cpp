#include <Arduino.h>
#include "protocol.h"
#include "drivetrain.h"
#include "robot_config.h"
#include <Bounce2.h>
#include "feeder.h"
#include "utils.h"
// ------------------------------------------------------------
// main.cpp
//
// Final (for now): ESP32 listens for newline-delimited JSON protocol messages
// on Serial and drives the robot using DriveTrain.
//
// Currently implemented:
//   - onDrive callback
//   - onFeed callback
//
//   - If no drive command is received for DRIVE_TIMEOUT_MS, motors stop.
// ------------------------------------------------------------

#include <Arduino.h>

#include "robot_config.h"
#include "protocol.h"
#include "drivetrain.h"
#include "feeder.h"
#include "utils.h"

#include "i2c_bus.h"
#include "i2c_sensor.h"
#include "tof/tof_manager.h"

#include <Bounce2.h>

// -------------------- Core subsystems --------------------

// Build drivetrain from centralized config
static DriveTrain drive(RobotConfig::driveTrainPins());

// Protocol runs on the hardware serial port
static Protocol proto(Serial);

// Controls feeder servo
static Feeder feeder(RobotConfig::feederConfig);

// Utilities
static Utils utils;

// -------------------- I2C + Sensors --------------------

static I2CBus i2c(RobotConfig::I2C);

// I2C sensors
static TofManager tof(RobotConfig::TOF_CFG, RobotConfig::TOF_CFG_COUNT);

// Registry for easy future expansion
static I2CSensor* i2cSensors[] = {
  &tof,
  // &imu,
  // &ina226,
  // &oled,
};

static constexpr size_t I2C_SENSOR_COUNT =
  sizeof(i2cSensors) / sizeof(i2cSensors[0]);

// Sensor polling cadence
static constexpr uint32_t SENSOR_POLL_PERIOD_MS = 10;
static uint32_t nextSensorMs = 0;

// -------------------- Drive failsafe --------------------

static constexpr uint32_t DRIVE_TIMEOUT_MS = 500;
static volatile uint32_t lastDriveMs = 0;

// -------------------- Protocol callback: onDrive --------------------

static void handleDrive(uint32_t seq, const Protocol::DriveCmd& cmd) {
  (void)seq;

  lastDriveMs = millis();

  const int throttle =
    RobotConfig::THR_SIGN * utils.map100To255(cmd.thr);
  const int steer =
    RobotConfig::STR_SIGN * utils.map100To255(cmd.str);

  drive.arcadeDrive(throttle, steer);
}

// -------------------- Bumper switch --------------------

static Bounce bumper;
static bool lastBumperPressed = false;
static constexpr bool STOP_ON_BUMPER = true;

static void setupBumper() {
  pinMode(RobotConfig::BUMPER_PIN, INPUT_PULLUP);
  bumper.attach((int)RobotConfig::BUMPER_PIN);
  bumper.interval(25);

  lastBumperPressed = (digitalRead(RobotConfig::BUMPER_PIN) == LOW);
  proto.sendSensorBool("bumper", lastBumperPressed, millis());
}

static void updateBumper() {
  bumper.update();
  const bool pressed = (bumper.read() == LOW);

  if (pressed == lastBumperPressed) return;

  lastBumperPressed = pressed;
  proto.sendSensorBool("bumper", pressed, millis());

  Serial.println("Hit bumper!");

  if (STOP_ON_BUMPER && pressed) {
    drive.stop();
  }
}

// -------------------- Protocol callback: onFeed --------------------

static void handleFeed(uint32_t seq, const Protocol::FeedCmd& cmd) {
  (void)seq;

  if (cmd.open) {
    Serial.print("[FEED] open ms=");
    Serial.println(cmd.ms);
    feeder.open(cmd.ms);
  } else {
    Serial.println("[FEED] close");
    feeder.close();
  }

  proto.sendSensorBool("feeder_open", feeder.isOpen(), millis());
}

// -------------------- setup() --------------------

void setup() {
  Serial.begin(RobotConfig::SERIAL_BUAD_RATE);
  delay(500);

  // ---- Drivetrain ----
  drive.setMaxSpeed(RobotConfig::MAX_SPEED);
  drive.setDeadband(RobotConfig::DEADBAND);
  drive.setPwmFrequency(RobotConfig::PWM_FREQ_HZ);
  drive.begin();

  // ---- Feeder ----
  //feeder.begin();

  // ---- Protocol callbacks ----
  Protocol::Callbacks cb;
  cb.onDrive = handleDrive;
  cb.onFeed  = handleFeed;
  proto.setCallbacks(cb);

  lastDriveMs = millis();

  // ---- I2C bring-up ----
  if (!i2c.begin()) {
    Serial.println("[I2C] begin FAILED");
  } else {
    Serial.println("[I2C] begin OK");
  }

  // Optional sanity scan
  // i2c.scan(Serial);

  // ---- Begin I2C sensors ----
  for (size_t i = 0; i < I2C_SENSOR_COUNT; i++) {
    I2CSensor* s = i2cSensors[i];
    Serial.print("[I2C] begin ");
    Serial.print(s->name());
    Serial.print(" ... ");
    //Serial.println(s->begin(i2c) ? "OK" : "FAIL");
  }

  // ---- Boot announce ----
  proto.sendBoot((uint32_t)(millis() / 1000));

  setupBumper();

}

// -------------------- loop() --------------------

void loop() {
  proto.poll();

  const uint32_t now = millis();

  // Drive failsafe
  if ((now - lastDriveMs) > DRIVE_TIMEOUT_MS) {
    drive.stop();
  }

  updateBumper();
  //feeder.update();

  // ---- I2C sensor polling ----
  if ((int32_t)(now - nextSensorMs) >= 0) {
    nextSensorMs = now + SENSOR_POLL_PERIOD_MS;

    for (size_t i = 0; i < I2C_SENSOR_COUNT; i++) {
      I2CSensor* s = i2cSensors[i];
      if (s->poll()) {
        s->publish(proto);
      }
    }
  }
}