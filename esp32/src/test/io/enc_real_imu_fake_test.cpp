// enc_real_imu_fake_test.cpp
//
// Sends REAL encoder telemetry via Protocol (NDJSON, no COBS, no CRC)
// plus FAKE IMU data as plain Serial.println at matching size/rate.
//
// Purpose: isolate whether the encoder hardware + protocol path
//          introduces corruption vs the baseline.
//
// Build: pio run -e test_enc_real -t upload
// Read:  python3 pi/test_baseline_rx.py /dev/ttyUSB0 115200

#include <Arduino.h>
#include "robot_config.h"
#include "motor.h"
#include "drivebase.h"

// ---- Motors + DriveBase (real hardware) ----
static Motor leftMotor;
static Motor rightMotor;
static DriveBase drive(leftMotor, rightMotor);

// ---- Timing ----
static constexpr uint32_t ENC_PERIOD_MS = 100;   // 10Hz — matches real
static constexpr uint32_t IMU_PERIOD_MS = 100;    // 10Hz
static constexpr uint32_t CONTROL_PERIOD_MS = 20;  // 50Hz motor update

static uint32_t lastEncMs = 0;
static uint32_t lastImuMs = 0;
static uint32_t lastCtrlMs = 0;

static uint32_t seqEnc = 0;
static uint32_t seqImu = 0;

void setup() {
  Serial.setTxBufferSize(1024);
  Serial.begin(RobotConfig::SERIAL_BUAD_RATE);
  delay(500);
  while (Serial.available()) Serial.read();

  // ---- Init real motors + encoders ----
  leftMotor.begin(
    RobotConfig::LEFT_PINS, RobotConfig::LEFT_PWM,
    RobotConfig::INVERT_LEFT_MOTOR, RobotConfig::INVERT_LEFT_ENCODER);
  rightMotor.begin(
    RobotConfig::RIGHT_PINS, RobotConfig::RIGHT_PWM,
    RobotConfig::INVERT_RIGHT_MOTOR, RobotConfig::INVERT_RIGHT_ENCODER);

  leftMotor.setPulsesPerWheelRev(RobotConfig::PULSES_PER_WHEEL_REV);
  rightMotor.setPulsesPerWheelRev(RobotConfig::PULSES_PER_WHEEL_REV);

  // Motors at zero — we just want encoder reads, not movement
  leftMotor.setU(0.0f);
  rightMotor.setU(0.0f);

  Serial.println("{\"type\":\"diag\",\"msg\":\"enc_real_imu_fake_test_start\"}");

  uint32_t now = millis();
  lastEncMs = now;
  lastImuMs = now + 50;  // stagger
  lastCtrlMs = now;
}

void loop() {
  uint32_t now = millis();

  // ---- Motor update at 50Hz (reads encoders) ----
  if (now - lastCtrlMs >= CONTROL_PERIOD_MS) {
    float dt = (now - lastCtrlMs) / 1000.0f;
    lastCtrlMs = now;
    drive.update(now, dt);
  }

  // ---- REAL encoder telemetry via plain NDJSON (no COBS/CRC) ----
  if (now - lastEncMs >= ENC_PERIOD_MS) {
    lastEncMs = now;
    seqEnc++;

    DriveTelemetry tel = drive.telemetry();

    // Build the same JSON structure as Protocol::sendDriveTelemetry
    // but using raw Serial.print (no protocol layer)
    Serial.print("{\"type\":\"enc\",\"seq\":");
    Serial.print(seqEnc);
    Serial.print(",\"t\":");
    Serial.print(now);
    Serial.print(",\"left\":{\"dt\":");
    Serial.print(tel.left.dticks);
    Serial.print(",\"tot\":");
    Serial.print((long)tel.left.ticks_total);
    Serial.print(",\"rad_s\":");
    Serial.print(tel.left.wheel_rad_s, 4);
    Serial.print("},\"right\":{\"dt\":");
    Serial.print(tel.right.dticks);
    Serial.print(",\"tot\":");
    Serial.print((long)tel.right.ticks_total);
    Serial.print(",\"rad_s\":");
    Serial.print(tel.right.wheel_rad_s, 4);
    Serial.println("}}");
  }

  // ---- FAKE IMU as plain Serial.println (~190 bytes) ----
  if (now - lastImuMs >= IMU_PERIOD_MS) {
    lastImuMs = now;
    seqImu++;

    Serial.print("{\"type\":\"imu\",\"seq\":");
    Serial.print(seqImu);
    Serial.print(",\"t\":");
    Serial.print(now);
    Serial.print(",\"ax\":-9.6175,\"ay\":0.4663,\"az\":2.1997");
    Serial.print(",\"gx\":-0.0023,\"gy\":0.0014,\"gz\":0.0089");
    Serial.print(",\"pad\":\"AAAAAAAAAAAAAAAAAAAAAAAAAAA\"");
    Serial.println("}");
  }
}
