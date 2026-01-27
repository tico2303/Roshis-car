//
// Tests:
//   1) Forward
//   2) Stop
//   3) Reverse
//   4) Stop
// Repeats forever.
//
// Pins requested:
//   Rear Left  = 27,14
//   Rear Right = 25,26
//
// IMPORTANT:
// This assumes your DriveTrain.cpp uses the proven L9110 logic:
//   - always analogWrite(A,0) + analogWrite(B,0) before switching direction
//   - Forward:  A=PWM, B=LOW
//   - Reverse:  A=LOW, B=PWM
//

#include <Arduino.h>
#include "drivetrain.h"

// -------------------- Rear motor pins --------------------
// Treat these as L9110 inputs A/B for each motor channel.
static constexpr uint8_t RL_A = 27;
static constexpr uint8_t RL_B = 14;

static constexpr uint8_t RR_A = 25;
static constexpr uint8_t RR_B = 26;

// -------------------- Test tuning knobs --------------------
static constexpr uint32_t BAUD = 115200;
static constexpr uint32_t PWM_FREQ_HZ = 2000;   // you said 2000 felt good
static constexpr int      SPEED = 120;          // 0..255
static constexpr uint32_t RUN_MS = 1500;
static constexpr uint32_t STOP_MS = 1000;

// -------------------- DriveTrain config --------------------
// Front motors unused -> MotorPins() uses sentinel pins (255,255)
DriveTrain::DriveTrainPins pins(
  DriveTrain::MotorPins(),                    // FL unused
  DriveTrain::MotorPins(),                    // FR unused
  DriveTrain::MotorPins(RL_A, RL_B, false),   // RL
  DriveTrain::MotorPins(RR_A, RR_B, false)    // RR
);

DriveTrain drive(pins);

void setup() {
  Serial.begin(BAUD);
  delay(800);

  Serial.println("=== DriveTrain rear-wheel test ===");

  drive.setMaxSpeed(255);
  drive.setDeadband(0);
  drive.setPwmFrequency(PWM_FREQ_HZ);

  // Initialize pins + PWM frequency and stop motors
  drive.begin();

  Serial.println("Setup complete.");
}

void loop() {
  Serial.println("FORWARD");
  drive.forward(SPEED);
  delay(RUN_MS);

  Serial.println("STOP");
  drive.stop();
  delay(STOP_MS);

  Serial.println("REVERSE");
  drive.reverse(SPEED);
  delay(RUN_MS);

  drive.turnRight(SPEED);;
  delay(RUN_MS);

  Serial.println("STOP");
  drive.stop();
  delay(2000);
}