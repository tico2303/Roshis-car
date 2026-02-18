// enc_real_imu_fake_test.cpp
//
// Layered encoder test — progressively enable hardware to find what
// corrupts serial. Each layer is a #define you toggle below.
//
// Layers (enable one at a time, bottom-up):
//   LAYER 0: fake enc + fake imu (baseline — should be 0% corrupt)
//   LAYER 1: raw ESP32Encoder reads only (no motor PWM, no DriveBase)
//   LAYER 2: motor.begin() + encoder reads (adds LEDC PWM setup)
//   LAYER 3: full DriveBase (adds motor.update loop)
//
// Build: pio run -e test_enc_real -t upload
// Read:  python3 pi/test_baseline_rx.py /dev/ttyUSB0 115200

#include <Arduino.h>
#include "robot_config.h"

// ============================================================
// >>> SET EXACTLY ONE OF THESE TO 1, REST TO 0 <<<
// ============================================================
#define LAYER 1
// 0 = all fake (baseline)
// 1 = raw ESP32Encoder only
// 2 = Motor.begin + encoder (adds PWM)
// 3 = full DriveBase
// ============================================================

#if LAYER >= 3
  #include "motor.h"
  #include "drivebase.h"
  static Motor leftMotor;
  static Motor rightMotor;
  static DriveBase drive(leftMotor, rightMotor);
#elif LAYER >= 2
  #include "motor.h"
  static Motor leftMotor;
  static Motor rightMotor;
#elif LAYER >= 1
  #include <ESP32Encoder.h>
  static ESP32Encoder encLeft;
  static ESP32Encoder encRight;
#endif

// ---- Timing ----
static constexpr uint32_t ENC_PERIOD_MS     = 100;  // 10Hz
static constexpr uint32_t IMU_PERIOD_MS     = 100;  // 10Hz
static constexpr uint32_t CONTROL_PERIOD_MS = 20;   // 50Hz (layer 3 only)

static uint32_t lastEncMs  = 0;
static uint32_t lastImuMs  = 0;
static uint32_t lastCtrlMs = 0;

static uint32_t seqEnc = 0;
static uint32_t seqImu = 0;

void setup() {
  Serial.setTxBufferSize(1024);
  Serial.begin(RobotConfig::SERIAL_BUAD_RATE);
  delay(500);
  while (Serial.available()) Serial.read();

#if LAYER >= 3
  // Full DriveBase init (motors + encoders + PWM)
  leftMotor.begin(
    RobotConfig::LEFT_PINS, RobotConfig::LEFT_PWM,
    RobotConfig::INVERT_LEFT_MOTOR, RobotConfig::INVERT_LEFT_ENCODER);
  rightMotor.begin(
    RobotConfig::RIGHT_PINS, RobotConfig::RIGHT_PWM,
    RobotConfig::INVERT_RIGHT_MOTOR, RobotConfig::INVERT_RIGHT_ENCODER);
  leftMotor.setPulsesPerWheelRev(RobotConfig::PULSES_PER_WHEEL_REV);
  rightMotor.setPulsesPerWheelRev(RobotConfig::PULSES_PER_WHEEL_REV);
  leftMotor.setU(0.0f);
  rightMotor.setU(0.0f);

#elif LAYER >= 2
  // Motor init (sets up LEDC PWM + encoder) but no DriveBase
  leftMotor.begin(
    RobotConfig::LEFT_PINS, RobotConfig::LEFT_PWM,
    RobotConfig::INVERT_LEFT_MOTOR, RobotConfig::INVERT_LEFT_ENCODER);
  rightMotor.begin(
    RobotConfig::RIGHT_PINS, RobotConfig::RIGHT_PWM,
    RobotConfig::INVERT_RIGHT_MOTOR, RobotConfig::INVERT_RIGHT_ENCODER);
  leftMotor.setU(0.0f);
  rightMotor.setU(0.0f);

#elif LAYER >= 1
  // Raw encoder only — no motor PWM at all
  ESP32Encoder::useInternalWeakPullResistors = puType::none;
  pinMode(RobotConfig::LEFT_PINS.encA, INPUT);
  pinMode(RobotConfig::LEFT_PINS.encB, INPUT);
  pinMode(RobotConfig::RIGHT_PINS.encA, INPUT);
  pinMode(RobotConfig::RIGHT_PINS.encB, INPUT);
  encLeft.attachHalfQuad(RobotConfig::LEFT_PINS.encA, RobotConfig::LEFT_PINS.encB);
  encRight.attachHalfQuad(RobotConfig::RIGHT_PINS.encA, RobotConfig::RIGHT_PINS.encB);
  encLeft.clearCount();
  encRight.clearCount();
#endif

  // Announce which layer we're running
  Serial.print("{\"type\":\"diag\",\"msg\":\"enc_test_layer_");
  Serial.print(LAYER);
  Serial.println("_start\"}");

  uint32_t now = millis();
  lastEncMs  = now;
  lastImuMs  = now + 50;  // stagger
  lastCtrlMs = now;
}

void loop() {
  uint32_t now = millis();

#if LAYER >= 3
  // DriveBase control loop at 50Hz
  if (now - lastCtrlMs >= CONTROL_PERIOD_MS) {
    float dt = (now - lastCtrlMs) / 1000.0f;
    lastCtrlMs = now;
    drive.update(now, dt);
  }
#endif

  // ---- Encoder telemetry at 10Hz ----
  if (now - lastEncMs >= ENC_PERIOD_MS) {
    lastEncMs = now;
    seqEnc++;

    int32_t l_dt = 0, r_dt = 0;
    long l_tot = 0, r_tot = 0;
    float l_rs = 0.0f, r_rs = 0.0f;

#if LAYER >= 3
    DriveTelemetry tel = drive.telemetry();
    l_dt  = tel.left.dticks;
    l_tot = (long)tel.left.ticks_total;
    l_rs  = tel.left.wheel_rad_s;
    r_dt  = tel.right.dticks;
    r_tot = (long)tel.right.ticks_total;
    r_rs  = tel.right.wheel_rad_s;

#elif LAYER >= 2
    // Read from Motor objects directly
    l_tot = (long)leftMotor.count();
    r_tot = (long)rightMotor.count();

#elif LAYER >= 1
    // Read raw PCNT counts
    l_tot = (long)encLeft.getCount();
    r_tot = (long)encRight.getCount();

#else
    // Layer 0: all fake
    l_dt = 5; l_tot = seqEnc * 5; l_rs = 1.2345f;
    r_dt = -3; r_tot = -(long)(seqEnc * 3); r_rs = -0.9876f;
#endif

    Serial.print("{\"type\":\"enc\",\"seq\":");
    Serial.print(seqEnc);
    Serial.print(",\"t\":");
    Serial.print(now);
    Serial.print(",\"layer\":");
    Serial.print(LAYER);
    Serial.print(",\"l_dt\":");
    Serial.print(l_dt);
    Serial.print(",\"l_tot\":");
    Serial.print(l_tot);
    Serial.print(",\"l_rs\":");
    Serial.print(l_rs, 4);
    Serial.print(",\"r_dt\":");
    Serial.print(r_dt);
    Serial.print(",\"r_tot\":");
    Serial.print(r_tot);
    Serial.print(",\"r_rs\":");
    Serial.print(r_rs, 4);
    Serial.println("}");
  }

  // ---- Fake IMU at 10Hz (~190 bytes) ----
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
