// ------------------------------------------------------------
// main.cpp
//
// ESP32 firmware for DARO robot.
// Listens for drv2 commands (differential drive) from Pi via NDJSON serial.
// Publishes encoder telemetry back to Pi for ROS2 odometry.
//
// Protocol messages:
//   Inbound:  {"type":"drv2","left":-1.0..1.0,"right":-1.0..1.0}
//   Outbound: {"type":"enc","t_ms":...,"left":{...},"right":{...}}
// ------------------------------------------------------------

#include <Arduino.h>

#include "robot_config.h"
#include "protocol.h"
#include "motor.h"
#include "drivebase.h"
#include "feeder.h"

#include "i2c_bus.h"
#include "i2c_sensor.h"
#include "tof/tof_manager.h"
#include "bmi160_imu.h"

#include <Bounce2.h>

// -------------------- Core subsystems --------------------

static Motor leftMotor;
static Motor rightMotor;
static DriveBase drive(leftMotor, rightMotor);

static Protocol proto(Serial);

static Feeder feeder(RobotConfig::feederConfig);

// -------------------- I2C + Sensors --------------------

static I2CBus i2c(RobotConfig::I2C);

static TofManager tof(RobotConfig::TOF_CFG, RobotConfig::TOF_CFG_COUNT);
static Bmi160Imu imu;

static I2CSensor* i2cSensors[] = {
  &tof,
  &imu,
  // &ina226,
};

static constexpr size_t I2C_SENSOR_COUNT =
  sizeof(i2cSensors) / sizeof(i2cSensors[0]);

static constexpr uint32_t SENSOR_POLL_PERIOD_MS = 10;
static uint32_t nextSensorMs = 0;

// -------------------- Control loop timing --------------------

static uint32_t lastUpdateMs = 0;

// Encoder telemetry publishes at a slower rate than the control loop
// to stay within 115200 baud bandwidth (~11,520 bytes/sec).
static constexpr uint32_t ENC_PUBLISH_PERIOD_MS = 100;  // 10Hz
static uint32_t lastEncPublishMs = 0;

// -------------------- Protocol callback: drv2 --------------------

static void handleDrive2(uint32_t seq, const Protocol::Drive2Cmd& cmd) {
  (void)seq;
  DriveCommand dcmd;
  dcmd.left_u  = cmd.left_u;
  dcmd.right_u = cmd.right_u;
  drive.setCommand(dcmd, millis());
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

  if (STOP_ON_BUMPER && pressed) {
    DriveCommand stop{};
    drive.setCommand(stop, millis());
  }
}

// -------------------- Protocol callback: feed --------------------

static void handleFeed(uint32_t seq, const Protocol::FeedCmd& cmd) {
  (void)seq;

  if (cmd.open) {
    feeder.open(cmd.ms);
  } else {
    feeder.close();
  }

  proto.sendSensorBool("feeder_open", feeder.isOpen(), millis());
}

// -------------------- setup() --------------------

void setup() {
  // Enlarge TX buffer so enc+imu telemetry doesn't overflow the
  // default 128-byte UART FIFO when multiple messages queue up.
  Serial.setTxBufferSize(1024);
  Serial.begin(RobotConfig::SERIAL_BUAD_RATE);
  delay(500);

  // ---- DriveBase ----
  leftMotor.begin(
    RobotConfig::LEFT_PINS, RobotConfig::LEFT_PWM,
    RobotConfig::INVERT_LEFT_MOTOR, RobotConfig::INVERT_LEFT_ENCODER);
  rightMotor.begin(
    RobotConfig::RIGHT_PINS, RobotConfig::RIGHT_PWM,
    RobotConfig::INVERT_RIGHT_MOTOR, RobotConfig::INVERT_RIGHT_ENCODER);

  leftMotor.setPulsesPerWheelRev(RobotConfig::PULSES_PER_WHEEL_REV);
  rightMotor.setPulsesPerWheelRev(RobotConfig::PULSES_PER_WHEEL_REV);

  // ---- Feeder ----
  //feeder.begin();

  // ---- Protocol callbacks ----
  Protocol::Callbacks cb;
  cb.onDrive2 = handleDrive2;
  cb.onFeed   = handleFeed;
  proto.setCallbacks(cb);

  // ---- I2C bring-up ----
  if (!i2c.begin()) {
    Serial.println("[I2C] begin FAILED");
  } else {
    Serial.println("[I2C] begin OK");
  }

  for (size_t i = 0; i < I2C_SENSOR_COUNT; i++) {
    I2CSensor* s = i2cSensors[i];
    Serial.print("[I2C] begin ");
    Serial.print(s->name());
    Serial.print(" ... ");
    Serial.println(s->begin(i2c) ? "OK" : "FAIL");
  }

  // ---- Boot announce ----
  proto.sendBoot((uint32_t)(millis() / 1000));

  setupBumper();

  lastUpdateMs = millis();
}

// -------------------- loop() --------------------

void loop() {
  proto.poll();

  const uint32_t now = millis();

  // ---- DriveBase control loop (20ms / 50Hz) ----
  if (now - lastUpdateMs >= RobotConfig::CONTROL_PERIOD_MS) {
    const float dt = (now - lastUpdateMs) / 1000.0f;
    lastUpdateMs = now;

    drive.update(now, dt);
  }

  // ---- Encoder telemetry (50ms / 20Hz) ----
  if (now - lastEncPublishMs >= ENC_PUBLISH_PERIOD_MS) {
    lastEncPublishMs = now;
    proto.sendDriveTelemetry(drive.telemetry());
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
