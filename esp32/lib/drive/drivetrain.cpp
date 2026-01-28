
#include <drivetrain.h>
#include <Arduino.h>

// ------------------------------------------------------------
// DriveTrain.cpp (L9110 two-input control)
//
// L9110 per-motor inputs (two pins per motor):
//   - Forward:  A = PWM, B = LOW
//   - Reverse:  A = LOW, B = PWM
//   - Stop:     A = LOW, B = LOW
//
// IMPORTANT:
// Your MotorPins names are dirPin/pwmPin, but for L9110 they’re really just:
//   dirPin -> input A
//   pwmPin -> input B
// We keep the names unchanged to preserve your header/UI.
// ------------------------------------------------------------

static inline void initMotorPins(uint8_t pinA, uint8_t pinB) {
  if (pinA == 255 || pinB == 255) return;   // "unset" sentinel
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  digitalWrite(pinA, LOW);
  digitalWrite(pinB, LOW);
}

static inline void stopMotor(uint8_t pinA, uint8_t pinB) {
  if (pinA == 255 || pinB == 255) return;

  // IMPORTANT on ESP32: ensure PWM is zeroed on both pins
  analogWrite(pinA, 0);
  analogWrite(pinB, 0);

  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  digitalWrite(pinA, LOW);
  digitalWrite(pinB, LOW);
}

// Core: apply signed speed to one L9110 channel

static inline void setMotorL9110(uint8_t pinA, uint8_t pinB, bool invert, int speed) {
  if (pinA == 255 || pinB == 255) return;

  if (invert) speed = -speed;
  speed = constrain(speed, -255, 255);

  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);

  // Always release PWM control from both pins first (matches your proven POC behavior).
  // This prevents the "previous PWM pin still attached" weirdness on ESP32.
  analogWrite(pinA, 0);
  analogWrite(pinB, 0);

  if (speed == 0) {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
    return;
  }

  const int pwm = abs(speed);

  if (speed > 0) {
    // Forward: A = PWM, B = LOW
    digitalWrite(pinB, LOW);
    analogWrite(pinA, pwm);
  } else {
    // Reverse: A = LOW, B = PWM
    digitalWrite(pinA, LOW);
    analogWrite(pinB, pwm);
  }
}

// -------------------- Construction / init --------------------

DriveTrain::DriveTrain(const DriveTrainPins& pins)
  : pins_(pins) {
  // Not active until begin() is called.
}

bool DriveTrain::begin(const DriveTrainPins& pins) {
  pins_ = pins;

  // Push PWM above audible range (works on many ESP32 Arduino cores)
  #if defined(ARDUINO_ARCH_ESP32)
    analogWriteFrequency(pwmFreqHz_);
  #endif

  initMotorPins(pins_.fl.dirPin, pins_.fl.pwmPin);
  initMotorPins(pins_.fr.dirPin, pins_.fr.pwmPin);
  initMotorPins(pins_.rl.dirPin, pins_.rl.pwmPin);
  initMotorPins(pins_.rr.dirPin, pins_.rr.pwmPin);

  initialized_ = true;
  stop();
  return true;
}

bool DriveTrain::begin() {
  #if defined(ARDUINO_ARCH_ESP32)
    analogWriteFrequency(pwmFreqHz_);
  #endif

  initMotorPins(pins_.fl.dirPin, pins_.fl.pwmPin);
  initMotorPins(pins_.fr.dirPin, pins_.fr.pwmPin);
  initMotorPins(pins_.rl.dirPin, pins_.rl.pwmPin);
  initMotorPins(pins_.rr.dirPin, pins_.rr.pwmPin);

  initialized_ = true;
  stop();
  return true;
}

// -------------------- Public API --------------------

void DriveTrain::forward(int speed) {
  if (!initialized_) return;

  speed = applyDeadband(clampSpeed(speed, maxSpeed_));
  setLeft(speed);
  setRight(speed);
}

void DriveTrain::reverse(int speed) {
  if (!initialized_) return;

  speed = applyDeadband(clampSpeed(speed, maxSpeed_));
  setLeft(-speed);
  setRight(-speed);
}

void DriveTrain::stop() {
  if (!initialized_) return;

  stopMotor(pins_.fl.dirPin, pins_.fl.pwmPin);
  stopMotor(pins_.fr.dirPin, pins_.fr.pwmPin);
  stopMotor(pins_.rl.dirPin, pins_.rl.pwmPin);
  stopMotor(pins_.rr.dirPin, pins_.rr.pwmPin);
}

void DriveTrain::turnLeft(int speed) {
  if (!initialized_) return;

  speed = applyDeadband(clampSpeed(speed, maxSpeed_));
  setLeft(-speed);
  setRight(speed);
}

void DriveTrain::turnRight(int speed) {
  if (!initialized_) return;

  speed = applyDeadband(clampSpeed(speed, maxSpeed_));
  setLeft(speed);
  setRight(-speed);
}

void DriveTrain::arcadeDrive(int throttle, int steer) {
  if (!initialized_) return;

  throttle = applyDeadband(clampSpeed(throttle, maxSpeed_));
  steer    = applyDeadband(clampSpeed(steer, maxSpeed_));

  int left  = throttle + steer;
  int right = throttle - steer;

  left  = applyDeadband(clampSpeed(left,  maxSpeed_));
  right = applyDeadband(clampSpeed(right, maxSpeed_));

  setLeft(left);
  setRight(right);
}
/*
//Alternative arcadeDrive implemenation
//Normalize + ramp (best driving feel)
//Normalization keeps turns sane; ramping prevents current spikes.
void DriveTrain::arcadeDrive(int throttle, int steer) {
  if (!initialized_) return;

  throttle = applyDeadband(clampSpeed(throttle, maxSpeed_));
  steer    = applyDeadband(clampSpeed(steer,    maxSpeed_));

  int left  = throttle + steer;
  int right = throttle - steer;

  // --- Normalize instead of clamp ---
  // If either side exceeds maxSpeed_, scale BOTH sides down proportionally.
  int maxMag = max(abs(left), abs(right));
  if (maxMag > maxSpeed_ && maxMag > 0) {
    left  = (left  * maxSpeed_) / maxMag;
    right = (right * maxSpeed_) / maxMag;
  }

  left  = applyDeadband(left);
  right = applyDeadband(right);

  setLeft(left);
  setRight(right);
}
*/
// -------------------- Tuning / UX knobs --------------------

void DriveTrain::setMaxSpeed(int maxSpeed) {
  maxSpeed_ = constrain(maxSpeed, 0, 255);
}

void DriveTrain::setDeadband(int deadband) {
  deadband_ = constrain(deadband, 0, 255);
}

void DriveTrain::setInvert(Wheel w, bool invert) {
  switch (w) {
    case Wheel::FL: pins_.fl.invert = invert; break;
    case Wheel::FR: pins_.fr.invert = invert; break;
    case Wheel::RL: pins_.rl.invert = invert; break;
    case Wheel::RR: pins_.rr.invert = invert; break;
  }
}

DriveTrain::DriveTrainPins DriveTrain::getPins() const {
  return pins_;
}

void DriveTrain::setPwmFrequency(uint32_t hz) {
  pwmFreqHz_ = constrain(hz, 2000UL, 40000UL);

  // Apply immediately if possible
  #if defined(ARDUINO_ARCH_ESP32)
    analogWriteFrequency(pwmFreqHz_);
  #endif
}

// -------------------- Helpers --------------------

int DriveTrain::clampSpeed(int speed, int maxSpeed) {
  if (speed >  maxSpeed) speed =  maxSpeed;
  if (speed < -maxSpeed) speed = -maxSpeed;
  return speed;
}

int DriveTrain::applyDeadband(int speed) const {
  return (abs(speed) < deadband_) ? 0 : speed;
}

// -------------------- Internal left/right application --------------------

void DriveTrain::setLeft(int speed) {
  speed = applyDeadband(clampSpeed(speed, maxSpeed_));
  setMotorL9110(pins_.fl.dirPin, pins_.fl.pwmPin, pins_.fl.invert, speed);
  setMotorL9110(pins_.rl.dirPin, pins_.rl.pwmPin, pins_.rl.invert, speed);
}

void DriveTrain::setRight(int speed) {
  speed = applyDeadband(clampSpeed(speed, maxSpeed_));
  setMotorL9110(pins_.fr.dirPin, pins_.fr.pwmPin, pins_.fr.invert, speed);
  setMotorL9110(pins_.rr.dirPin, pins_.rr.pwmPin, pins_.rr.invert, speed);
}