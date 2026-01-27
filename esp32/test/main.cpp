#include <Arduino.h>
#include "protocol.h"
#include "drivetrain.h"
#include "robot_config.h"
#include <Bounce2.h>
#include "feeder.h"
// ------------------------------------------------------------
// main.cpp
//
// Final (for now): ESP32 listens for newline-delimited JSON protocol messages
// on Serial and drives the robot using DriveTrain.
//
// Currently implemented:
//   - onDrive callback only
//
//   - If no drive command is received for DRIVE_TIMEOUT_MS, motors stop.
// ------------------------------------------------------------

// Build drivetrain from centralized config
static DriveTrain drive(RobotConfig::driveTrainPins());

// Protocol runs on the hardware serial port
static Protocol proto(Serial);

// controls feeder servo
static Feeder feeder = Feeder(RobotConfig::feederConfig);
// Stop motors if commands stop arriving (failsafe)
static constexpr uint32_t DRIVE_TIMEOUT_MS = 500;

static volatile uint32_t lastDriveMs = 0;

static int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static int map100To255(int v) {
  v = clampInt(v, -100, 100);
  // Scale -100..100 to roughly -255..255
  return (v * 255) / 100;
}

// -------------------- Protocol callback: onDrive --------------------
static void handleDrive(uint32_t seq, const Protocol::DriveCmd& cmd) {
  (void)seq;

  // Record arrival time for failsafe
  lastDriveMs = millis();

  // Convert protocol units -> motor units
  const int throttle = RobotConfig::THR_SIGN * map100To255(cmd.thr);
  const int steer    = RobotConfig::STR_SIGN * map100To255(cmd.str);

  // DriveTrain uses arcade mixing internally
  drive.arcadeDrive(throttle, steer);
}

//-------------- Bumper switch logic
static Bounce bumper;
static bool lastBumperPressed = false;

// Optional: stop on bumper press
static constexpr bool STOP_ON_BUMPER = true;

static void setupBumper() {
  pinMode(RobotConfig::BUMPER_PIN, INPUT_PULLUP); // active-low wiring
  bumper.attach((int)RobotConfig::BUMPER_PIN);
  bumper.interval(25);

  lastBumperPressed = (digitalRead(RobotConfig::BUMPER_PIN) == LOW);
  proto.sendSensorBool("bumper", lastBumperPressed, millis());
}

static void updateBumper() {
  bumper.update();
  const bool pressed = (bumper.read() == LOW); // active-low

  if (pressed == lastBumperPressed) return;

  lastBumperPressed = pressed;
  proto.sendSensorBool("bumper", pressed, millis());
  Serial.println("Hit bumper!");

  if (STOP_ON_BUMPER && pressed) {
    drive.stop();
  }
}

static void handleFeed(uint32_t seq, const Protocol::FeedCmd& cmd) {
  (void)seq;

  if (cmd.open) {
    Serial.print("[FEED] open ms=");
    Serial.println(cmd.ms);
    feeder.open(cmd.ms);
    // Optional telemetry:
    proto.sendSensorBool("feeder_open", feeder.isOpen(), millis());
  } else {
    Serial.println("[FEED] close");
    feeder.close();
    // Optional telemetry:
    proto.sendSensorBool("feeder_open", feeder.isOpen(), millis());
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  drive.setMaxSpeed(RobotConfig::MAX_SPEED);
  drive.setDeadband(RobotConfig::DEADBAND);
  drive.setPwmFrequency(RobotConfig::PWM_FREQ_HZ);
  drive.begin();

  feeder.begin();

  // Wire up protocol callback(s)
  Protocol::Callbacks cb;
  cb.onDrive = handleDrive;
  cb.onFeed = handleFeed;

  proto.setCallbacks(cb);

  lastDriveMs = millis();

  // announce boot
  proto.sendBoot((uint32_t)(millis() / 1000));

  setupBumper();
  Serial.println("[ESP32] Ready. Waiting for protocol drive commands...");
}

void loop() {
  // Process any incoming protocol lines
  proto.poll();

  // Failsafe: if drive commands stop, stop motors
  const uint32_t now = millis();
  if ((now - lastDriveMs) > DRIVE_TIMEOUT_MS) {
    drive.stop();
  }
  updateBumper();
  feeder.update();
}
