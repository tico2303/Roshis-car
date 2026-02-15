#include <Arduino.h>
#include "drivebase.h"

// ---------- LEFT MOTOR ----------
Motor leftMotor;
MotorPins LEFT_PINS{
  .in1 = 25,
  .in2 = 26,
  .encA = 32,
  .encB = 33
};

MotorPwmConfig LEFT_PWM{
  .ch_in1 = 0,
  .ch_in2 = 1,
  .freq_hz = 20000,
  .res_bits = 8
};

// ---------- RIGHT MOTOR ----------
Motor rightMotor;
MotorPins RIGHT_PINS{
  .in1 = 27,
  .in2 = 14,
  .encA = 19,
  .encB = 18
};

MotorPwmConfig RIGHT_PWM{
  .ch_in1 = 2,
  .ch_in2 = 3,
  .freq_hz = 20000,
  .res_bits = 8
};

// ---------- DRIVE BASE ----------
DriveBase drive(leftMotor, rightMotor);

// ---------- TIMING ----------
static uint32_t lastUpdate = 0;
static const uint32_t LOOP_MS = 20;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nDriveBase Test Starting...");

  leftMotor.begin(LEFT_PINS, LEFT_PWM, false, false);
  rightMotor.begin(RIGHT_PINS, RIGHT_PWM, true, true);

  // Update these once you know your real pulses-per-wheel-rev
  leftMotor.setPulsesPerWheelRev(660);
  rightMotor.setPulsesPerWheelRev(660);

  lastUpdate = millis();
}

static void sendCommand(float l, float r) {
  DriveCommand cmd;
  cmd.left_u = l;
  cmd.right_u = r;
  drive.setCommand(cmd, millis());
}

void loop() {
  const uint32_t now = millis();
  const float dt = (now - lastUpdate) / 1000.0f;

  if (now - lastUpdate >= LOOP_MS) {
    lastUpdate = now;
    drive.update(now, dt);

    const auto tel = drive.telemetry();
    Serial.printf("L dt=%ld tot=%lld  R dt=%ld tot=%lld\n",
      (long)tel.left.dticks, (long long)tel.left.ticks_total,
      (long)tel.right.dticks, (long long)tel.right.ticks_total
    );
  }

  // --- Command phases ---
  static uint32_t phaseStart = millis();
  const uint32_t phaseTime = now - phaseStart;

  if (phaseTime < 6000) {
    sendCommand(0.8f, 0.0f);   // LEFT ONLY
  }
  else if (phaseTime < 12000) {
    sendCommand(0.0f, 0.4f);   // RIGHT ONLY
  }
  else if (phaseTime < 18000) {
    sendCommand(0.4f, 0.4f);   // BOTH
  }
  else {
    phaseStart = now;
  }
}