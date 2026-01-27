#include <Arduino.h>
#include "drivetrain.h"

// ============================================================
// PIN CONFIG (must match your known-good all_wheel_test)
// ============================================================

// FRONT LEFT
static constexpr uint8_t FL_A = 33;
static constexpr uint8_t FL_B = 32;

// FRONT RIGHT
static constexpr uint8_t FR_A = 19;
static constexpr uint8_t FR_B = 18;

// REAR LEFT
static constexpr uint8_t RL_A = 27;
static constexpr uint8_t RL_B = 14;

// REAR RIGHT
static constexpr uint8_t RR_A = 25;
static constexpr uint8_t RR_B = 26;

// ============================================================
// ARCADE DRIVE DEBUG KNOBS (EDIT THESE)
// ============================================================

// Base command values
static int BASE_THROTTLE = 150;   // forward/backward magnitude
static int BASE_STEER    = 0;     // left/right magnitude

// Sign conventions (THIS is what we're debugging)
static int THROTTLE_SIGN = +1;    // flip to -1 if forward/back is reversed
static int STEER_SIGN    = +1;    // flip to -1 if left/right is reversed

// Timing
static constexpr uint32_t RUN_MS  = 2000;
static constexpr uint32_t STOP_MS = 1200;

// ============================================================
// DRIVE TRAIN SETUP
// ============================================================

// =========== The fix was seting the invert to TRUE for both front wheels
DriveTrain::DriveTrainPins pins(
  DriveTrain::MotorPins(FL_A, FL_B, true),
  DriveTrain::MotorPins(FR_A, FR_B, true),
  DriveTrain::MotorPins(RL_A, RL_B, false),
  DriveTrain::MotorPins(RR_A, RR_B, false)
);

DriveTrain drive(pins);

// ============================================================

void printConfig(int thr, int str, int left, int right) {
  Serial.println("--------------------------------");
  Serial.print("BASE_THROTTLE = "); Serial.println(BASE_THROTTLE);
  Serial.print("BASE_STEER    = "); Serial.println(BASE_STEER);
  Serial.print("THR_SIGN      = "); Serial.println(THROTTLE_SIGN);
  Serial.print("STR_SIGN      = "); Serial.println(STEER_SIGN);
  Serial.println();
  Serial.print("Final throttle = "); Serial.println(thr);
  Serial.print("Final steer    = "); Serial.println(str);
  Serial.print("Left output    = "); Serial.println(left);
  Serial.print("Right output   = "); Serial.println(right);
}

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println("=== arcadeDrive DEBUG TEST ===");
  Serial.println("Wheels OFF the ground.");

  drive.setMaxSpeed(255);
  drive.setDeadband(0);
  drive.setPwmFrequency(2000);
  drive.begin();

  Serial.println("Setup complete.");
}

void loop() {
    delay(5000);
  // Compute signed inputs
  int throttle = THROTTLE_SIGN * BASE_THROTTLE;
  int steer    = STEER_SIGN    * BASE_STEER;

  // What arcadeDrive *will* compute internally
  int left  = throttle + steer;
  int right = throttle - steer;

  printConfig(throttle, steer, left, right);

  Serial.println(">>> APPLY arcadeDrive()");
  drive.arcadeDrive(throttle, steer);
  delay(RUN_MS);

  Serial.println(">>> STOP");
  drive.stop();
  delay(STOP_MS);

  Serial.println("=== Loop complete. Adjust globals & reset ===");
  while (true) delay(1000);
}