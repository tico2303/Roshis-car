#include <Arduino.h>
#include "drivetrain.h"

// -------------------- Pin assignment (L9110 inputs A/B) --------------------

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

// -------------------- Test tuning knobs --------------------
static constexpr uint32_t BAUD = 115200;
static constexpr uint32_t PWM_FREQ_HZ = 2000;   
static constexpr int      SPEED = 120;          // 0..255
static constexpr uint32_t RUN_MS = 1500;
static constexpr uint32_t STOP_MS = 800;

// -------------------- DriveTrain pin config --------------------
// Start with all invert=false.
// Flip individual wheels if spinning wrong way
DriveTrain::DriveTrainPins pins(
  DriveTrain::MotorPins(FL_A, FL_B, true), // FL
  DriveTrain::MotorPins(FR_A, FR_B, true), // FR
  DriveTrain::MotorPins(RL_A, RL_B, false), // RL
  DriveTrain::MotorPins(RR_A, RR_B, false)  // RR
);

DriveTrain drive(pins);

void setup() {
  Serial.begin(BAUD);
  delay(800);

  Serial.println("=== DriveTrain 4-wheel test ===");
  Serial.println("Keep wheels OFF the ground for initial test.");

  drive.setMaxSpeed(255);
  drive.setDeadband(0);
  drive.setPwmFrequency(PWM_FREQ_HZ);
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

  Serial.println("STOP");
  drive.stop();
  delay(STOP_MS);

  Serial.println("TURN LEFT (pivot)");
  drive.turnLeft(SPEED);
  delay(RUN_MS);

  Serial.println("STOP");
  drive.stop();
  delay(STOP_MS);

  Serial.println("TURN RIGHT (pivot)");
  drive.turnRight(SPEED);
  delay(RUN_MS);

  Serial.println("STOP");
  drive.stop();
  delay(2000);
}
