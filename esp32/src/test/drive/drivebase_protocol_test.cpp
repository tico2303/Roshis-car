#include <Arduino.h>
#include "protocol.h"
#include "drivebase.h"
#include "robot_config.h"
// -------------------- Motor Setup --------------------
Motor leftMotor;

Motor rightMotor;

DriveBase drive(leftMotor, rightMotor);

// -------------------- ScriptedStream --------------------
class ScriptedStream : public Stream {
public:
  explicit ScriptedStream(Stream& out) : _out(out) {}

  void injectLine(const char* line) {
    while (*line) _rx += *line++;
    _rx += '\n';
  }

  int available() override { return (int)_rx.length(); }

  int read() override {
    if (_rx.length() == 0) return -1;
    char c = _rx[0];
    _rx.remove(0, 1);
    return (uint8_t)c;
  }

  int peek() override {
    if (_rx.length() == 0) return -1;
    return (uint8_t)_rx[0];
  }

  void flush() override { _out.flush(); }

  size_t write(uint8_t b) override {
    return _out.write(b);
  }

private:
  Stream& _out;
  String _rx;
};

ScriptedStream scriptedIO(Serial);
Protocol proto(scriptedIO);

// -------------------- Timing --------------------
static constexpr uint32_t LOOP_MS = 20;
static constexpr uint32_t INJECT_MS = 100;  // re-send command every 100ms (within 250ms watchdog)
static uint32_t lastUpdate = 0;
static uint32_t lastInject = 0;
static uint32_t phaseStart = 0;
static uint32_t seq = 1;
static bool stopped = false;

// -------------------- drv2 callback --------------------
void onDrive2(uint32_t /*seq*/, const Protocol::Drive2Cmd& cmd) {
  Serial.println("onDrive2 called...");
  DriveCommand dcmd;
  dcmd.left_u  = cmd.left_u;
  dcmd.right_u = cmd.right_u;
  drive.setCommand(dcmd, millis());
}

// -------------------- Phase injection --------------------
struct Phase {
  const char* label;
  float left;
  float right;
  uint32_t duration_ms;
};

static const Phase PHASES[] = {
  { "FORWARD",    0.5f,  0.5f,  4000 },
  { "REVERSE",   -0.5f, -0.5f,  4000 },
  { "SPIN LEFT", -0.4f,  0.4f,  4000 },
  { "SPIN RIGHT", 0.4f, -0.4f,  4000 },
  { "STOP",       0.0f,  0.0f,  2000 },
};
static constexpr size_t PHASE_COUNT = sizeof(PHASES) / sizeof(PHASES[0]);
static size_t phaseIdx = 0;

static void injectPhaseCommand() {
  const Phase& p = PHASES[phaseIdx];
  char buf[128];
  snprintf(buf, sizeof(buf),
    R"({"type":"drv2","seq":%u,"left":%.2f,"right":%.2f})",
    (unsigned)seq++, p.left, p.right);
  scriptedIO.injectLine(buf);
}

// -------------------- Arduino --------------------
void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println("\n=== DriveBase + Protocol (drv2) Auto-Run Test ===");

  leftMotor.begin(RobotConfig::LEFT_PINS, RobotConfig::LEFT_PWM, RobotConfig::INVERT_LEFT_MOTOR, RobotConfig::INVERT_LEFT_ENCODER);
  rightMotor.begin(RobotConfig::RIGHT_PINS, RobotConfig::RIGHT_PWM, RobotConfig::INVERT_RIGHT_MOTOR, RobotConfig::INVERT_RIGHT_ENCODER);
  leftMotor.setPulsesPerWheelRev(660);
  rightMotor.setPulsesPerWheelRev(660);

  Protocol::Callbacks cb;
  cb.onDrive2 = onDrive2;
  proto.setCallbacks(cb);

  uint32_t now = millis();
  lastUpdate = now;
  lastInject = now;
  phaseStart = now;

  Serial.println("Type 's' + ENTER for emergency stop.");
  Serial.println("Setup complete.\n");
}

void loop() {
  proto.poll();

  const uint32_t now = millis();

  // Emergency stop via serial
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 's' || c == 'S') {
      stopped = !stopped;
      if (stopped) {
        Serial.println("!!! STOPPED !!! (press 's' to resume)");
        DriveCommand stop;
        drive.setCommand(stop, now);
      } else {
        Serial.println("--- RESUMED ---");
        phaseStart = now;
      }
      while (Serial.available()) Serial.read();
    }
  }

  // Drive update at 20ms cadence
  if (now - lastUpdate >= LOOP_MS) {
    const float dt = (now - lastUpdate) / 1000.0f;
    lastUpdate = now;
    drive.update(now, dt);
    //sendEncoderTelemetry(proto, drive.telemetry());
    proto.sendDriveTelemetry(drive.telemetry());
  }

  if (stopped) return;

  // Phase transition
  const uint32_t phaseTime = now - phaseStart;
  if (phaseTime >= PHASES[phaseIdx].duration_ms) {
    phaseIdx = (phaseIdx + 1) % PHASE_COUNT;
    phaseStart = now;
    Serial.printf("\n--- Phase: %s (%.1f, %.1f) for %ums ---\n",
      PHASES[phaseIdx].label,
      PHASES[phaseIdx].left,
      PHASES[phaseIdx].right,
      (unsigned)PHASES[phaseIdx].duration_ms);
  }

  // Inject drv2 command every 100ms to keep watchdog happy
  if (now - lastInject >= INJECT_MS) {
    lastInject = now;
    injectPhaseCommand();
  }
}
