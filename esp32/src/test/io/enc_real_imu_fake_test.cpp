// enc_real_imu_fake_test.cpp
//
// Layered hardware test — progressively enable subsystems to find what
// corrupts serial. Change #define LAYER and reflash.
//
// Layers:
//   0: all fake (baseline — should be ~0% corrupt)
//   1: raw ESP32Encoder only (no motor PWM)
//   2: Motor.begin + encoder (adds LEDC PWM setup)
//   3: full DriveBase (adds motor.update at 50Hz)
//   4: I2C bus + real BMI160 IMU reads + real encoders (Layer 2 + IMU)
//   5: full main.cpp equivalent (DriveBase + I2C + IMU + bumper, via Protocol)
//
// Build: pio run -e test_enc_real -t upload
// Read:  python3 pi/test_baseline_rx.py /dev/ttyUSB0 115200
//        (layers 0-4 use plain Serial.println, layer 5 uses Protocol+COBS)

#include <Arduino.h>
#include "robot_config.h"

// ============================================================
// >>> SET THIS TO THE LAYER YOU WANT TO TEST <<<
// ============================================================
#define LAYER 5
// ============================================================

// ---- Includes per layer ----
// bmi160_imu.h includes a publish(Protocol&) method that uses ArduinoJson
// and Protocol, so we must include those headers whenever bmi160_imu.h is used.
#if LAYER >= 4
  #include <ArduinoJson.h>
  #include "protocol.h"
  #include "motor.h"
  #include "i2c_bus.h"
  #include "i2c_sensor.h"
  #include "bmi160_imu.h"
  #if LAYER >= 5
    #include "drivebase.h"
    #include <Bounce2.h>
  #endif
#elif LAYER >= 3
  #include "motor.h"
  #include "drivebase.h"
#elif LAYER >= 2
  #include "motor.h"
#elif LAYER >= 1
  #include <ESP32Encoder.h>
#endif

// ---- Objects per layer ----
#if LAYER >= 5
  static Motor leftMotor;
  static Motor rightMotor;
  static DriveBase drive(leftMotor, rightMotor);
  static Protocol proto(Serial);
  static I2CBus i2c(RobotConfig::I2C);
  static Bmi160Imu imu;
  static Bounce bumper;
  static bool lastBumperPressed = false;
#elif LAYER >= 4
  static Motor leftMotor;
  static Motor rightMotor;
  static I2CBus i2c(RobotConfig::I2C);
  static Bmi160Imu imu;
#elif LAYER >= 3
  static Motor leftMotor;
  static Motor rightMotor;
  static DriveBase drive(leftMotor, rightMotor);
#elif LAYER >= 2
  static Motor leftMotor;
  static Motor rightMotor;
#elif LAYER >= 1
  static ESP32Encoder encLeft;
  static ESP32Encoder encRight;
#endif

// ---- Timing ----
static constexpr uint32_t ENC_PERIOD_MS     = 100;  // 10Hz
static constexpr uint32_t IMU_PERIOD_MS     = 100;  // 10Hz
static constexpr uint32_t CONTROL_PERIOD_MS = 20;   // 50Hz (layer 3+ only)
static constexpr uint32_t SENSOR_POLL_MS    = 10;   // 100Hz I2C poll (layer 4+)

static uint32_t lastEncMs    = 0;
static uint32_t lastImuMs    = 0;
static uint32_t lastCtrlMs   = 0;
static uint32_t nextSensorMs = 0;

static uint32_t seqEnc = 0;
static uint32_t seqImu = 0;

void setup() {
  Serial.setTxBufferSize(1024);
  Serial.begin(RobotConfig::SERIAL_BUAD_RATE);
  delay(500);
  while (Serial.available()) Serial.read();

  // ---- Layer-specific init ----

#if LAYER >= 5
  // Full main.cpp equivalent
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

  if (!i2c.begin()) {
    proto.sendErr("debug", 0, "[I2C] begin FAILED");
  }
  if (!imu.begin(i2c)) {
    proto.sendErr("debug", 0, "[IMU] begin FAILED");
  }

  pinMode(RobotConfig::BUMPER_PIN, INPUT_PULLUP);
  bumper.attach((int)RobotConfig::BUMPER_PIN);
  bumper.interval(25);
  lastBumperPressed = (digitalRead(RobotConfig::BUMPER_PIN) == LOW);

  proto.sendBoot((uint32_t)(millis() / 1000));

#elif LAYER >= 4
  // Motors (for encoders) + I2C + real IMU
  leftMotor.begin(
    RobotConfig::LEFT_PINS, RobotConfig::LEFT_PWM,
    RobotConfig::INVERT_LEFT_MOTOR, RobotConfig::INVERT_LEFT_ENCODER);
  rightMotor.begin(
    RobotConfig::RIGHT_PINS, RobotConfig::RIGHT_PWM,
    RobotConfig::INVERT_RIGHT_MOTOR, RobotConfig::INVERT_RIGHT_ENCODER);
  leftMotor.setU(0.0f);
  rightMotor.setU(0.0f);

  bool i2c_ok = i2c.begin();
  bool imu_ok = imu.begin(i2c);

  Serial.print("{\"type\":\"diag\",\"msg\":\"layer4_init\",\"i2c\":");
  Serial.print(i2c_ok ? "true" : "false");
  Serial.print(",\"imu\":");
  Serial.print(imu_ok ? "true" : "false");
  Serial.println("}");

#elif LAYER >= 3
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
  leftMotor.begin(
    RobotConfig::LEFT_PINS, RobotConfig::LEFT_PWM,
    RobotConfig::INVERT_LEFT_MOTOR, RobotConfig::INVERT_LEFT_ENCODER);
  rightMotor.begin(
    RobotConfig::RIGHT_PINS, RobotConfig::RIGHT_PWM,
    RobotConfig::INVERT_RIGHT_MOTOR, RobotConfig::INVERT_RIGHT_ENCODER);
  leftMotor.setU(0.0f);
  rightMotor.setU(0.0f);

#elif LAYER >= 1
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

#if LAYER < 5
  Serial.print("{\"type\":\"diag\",\"msg\":\"layer_");
  Serial.print(LAYER);
  Serial.println("_start\"}");
#endif

  uint32_t now = millis();
  lastEncMs    = now;
  lastImuMs    = now + 50;  // stagger
  lastCtrlMs   = now;
  nextSensorMs = now;
}

void loop() {
  uint32_t now = millis();

#if LAYER >= 5
  proto.poll();
#endif

#if LAYER == 3 || LAYER >= 5
  if (now - lastCtrlMs >= CONTROL_PERIOD_MS) {
    float dt = (now - lastCtrlMs) / 1000.0f;
    lastCtrlMs = now;
    drive.update(now, dt);
  }
#endif

#if LAYER >= 4
  if ((int32_t)(now - nextSensorMs) >= 0) {
    nextSensorMs = now + SENSOR_POLL_MS;
    imu.poll();
  }
#endif

#if LAYER >= 5
  bumper.update();
  bool pressed = (bumper.read() == LOW);
  if (pressed != lastBumperPressed) {
    lastBumperPressed = pressed;
    proto.sendSensorBool("bumper", pressed, millis());
  }
#endif

  // ---- Encoder telemetry at 10Hz ----
  if (now - lastEncMs >= ENC_PERIOD_MS) {
    lastEncMs = now;
    seqEnc++;

    int32_t l_dt = 0, r_dt = 0;
    long l_tot = 0, r_tot = 0;
    float l_rs = 0.0f, r_rs = 0.0f;

#if LAYER == 3 || LAYER >= 5
    DriveTelemetry tel = drive.telemetry();
    l_dt  = tel.left.dticks;
    l_tot = (long)tel.left.ticks_total;
    l_rs  = tel.left.wheel_rad_s;
    r_dt  = tel.right.dticks;
    r_tot = (long)tel.right.ticks_total;
    r_rs  = tel.right.wheel_rad_s;
#elif LAYER == 2 || LAYER == 4
    // Motor objects have encoders but no DriveBase
    l_tot = (long)leftMotor.count();
    r_tot = (long)rightMotor.count();
#elif LAYER >= 1
    l_tot = (long)encLeft.getCount();
    r_tot = (long)encRight.getCount();
#else
    l_dt = 5; l_tot = seqEnc * 5; l_rs = 1.2345f;
    r_dt = -3; r_tot = -(long)(seqEnc * 3); r_rs = -0.9876f;
#endif

#if LAYER >= 5
    DriveTelemetry tel5;
    tel5.t_ms = now;
    tel5.left.dticks = l_dt;
    tel5.left.ticks_total = l_tot;
    tel5.left.wheel_rad_s = l_rs;
    tel5.right.dticks = r_dt;
    tel5.right.ticks_total = r_tot;
    tel5.right.wheel_rad_s = r_rs;
    proto.sendDriveTelemetry(tel5);
#else
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
#endif
  }

  // ---- IMU at 10Hz ----
  if (now - lastImuMs >= IMU_PERIOD_MS) {
    lastImuMs = now;
    seqImu++;

#if LAYER >= 5
    if (imu.hasNew()) {
      imu.publish(proto);
    }

#elif LAYER >= 4
    if (imu.hasNew()) {
      const Bmi160Reading& r = imu.last();
      Serial.print("{\"type\":\"imu\",\"seq\":");
      Serial.print(seqImu);
      Serial.print(",\"t\":");
      Serial.print(now);
      Serial.print(",\"ax\":");
      Serial.print(r.ax, 4);
      Serial.print(",\"ay\":");
      Serial.print(r.ay, 4);
      Serial.print(",\"az\":");
      Serial.print(r.az, 4);
      Serial.print(",\"gx\":");
      Serial.print(r.gx, 4);
      Serial.print(",\"gy\":");
      Serial.print(r.gy, 4);
      Serial.print(",\"gz\":");
      Serial.print(r.gz, 4);
      Serial.println("}");
    } else {
      Serial.print("{\"type\":\"imu\",\"seq\":");
      Serial.print(seqImu);
      Serial.print(",\"t\":");
      Serial.print(now);
      Serial.print(",\"fake\":true");
      Serial.println("}");
    }

#else
    // Fake IMU (~190 bytes)
    Serial.print("{\"type\":\"imu\",\"seq\":");
    Serial.print(seqImu);
    Serial.print(",\"t\":");
    Serial.print(now);
    Serial.print(",\"ax\":-9.6175,\"ay\":0.4663,\"az\":2.1997");
    Serial.print(",\"gx\":-0.0023,\"gy\":0.0014,\"gz\":0.0089");
    Serial.print(",\"pad\":\"AAAAAAAAAAAAAAAAAAAAAAAAAAA\"");
    Serial.println("}");
#endif
  }
}
