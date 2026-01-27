#include <Arduino.h>
#include "protocol.h"
#include "drivetrain.h"
#include "robot_config.h"


DriveTrain drive(RobotConfig::driveTrainPins());

// -------------------- A Stream that reads from a scripted RX buffer,
// and writes outgoing protocol responses to Serial --------------------
class ScriptedStream : public Stream {
public:
  explicit ScriptedStream(Stream& out) : _out(out) {}

  // Push a line of JSON that Protocol::poll() will read as if it arrived over serial.
  void injectLine(const char* line) {
    while (*line) _rx += *line++;
    _rx += '\n'; // protocol is newline-delimited
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
    return _out.write(b); // outbound messages go to Serial monitor
  }

private:
  Stream& _out;
  String _rx; // simple + fine for a test harness
};

ScriptedStream scriptedIO(Serial);
Protocol proto(scriptedIO);

static const char* SCRIPT[] = {
  R"({"type":"drv","seq":1,"thr":  0,"str":  0})",

  // Forward
  R"({"type":"drv","seq":2,"thr": 60,"str":  0})",
  R"({"type":"drv","seq":3,"thr": 60,"str":  0})",

  // Stop
  R"({"type":"drv","seq":4,"thr":  0,"str":  0})",

  // Reverse
  R"({"type":"drv","seq":5,"thr":-60,"str":  0})",
  R"({"type":"drv","seq":6,"thr":-60,"str":  0})",

  // Stop
  R"({"type":"drv","seq":7,"thr":  0,"str":  0})",

  // AGRESSIVE left turn while moving forward
  R"({"type":"drv","seq":8,"thr": 50,"str":-40})",
  R"({"type":"drv","seq":9,"thr": 50,"str":-40})",

  // AGRESSIVE right turn while moving forward
  R"({"type":"drv","seq":10,"thr":50,"str": 40})",
  R"({"type":"drv","seq":11,"thr":50,"str": 40})",

  // Gentle left turn while moving forward
  R"({"type":"drv","seq":8,"thr": 50,"str":-15})",
  R"({"type":"drv","seq":9,"thr": 50,"str":-15})",

  // Gentle right turn while moving forward
  R"({"type":"drv","seq":10,"thr":50,"str": 15})",
  R"({"type":"drv","seq":11,"thr":50,"str": 15})",

  // Stop
  R"({"type":"drv","seq":12,"thr": 0,"str":  0})",
};

static constexpr size_t SCRIPT_LEN = sizeof(SCRIPT) / sizeof(SCRIPT[0]);

// How often to inject the next command (ms)
static constexpr uint32_t STEP_MS = 800;

// -------------------- Drive callback: protocol -> drivetrain --------------------
static int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static int map100To255(int v) {
  v = clampInt(v, -100, 100);
  // Scale -100..100 to -255..255
  // (avoid Arduino map() long math surprises)
  return (v * 255) / 100;
}

void onDrive(uint32_t seq, const Protocol::DriveCmd& cmd) {
  int throttle = map100To255(cmd.thr);
  int steer    = map100To255(cmd.str);

  Serial.print("[onDrive] seq=");
  Serial.print(seq);
  Serial.print(" thr=");
  Serial.print(cmd.thr);
  Serial.print(" str=");
  Serial.print(cmd.str);
  Serial.print(" -> arcadeDrive(thr=");
  Serial.print(throttle);
  Serial.print(", steer=");
  Serial.print(steer);
  Serial.println(")");

  drive.arcadeDrive(throttle, steer);
}
static void waitForEnter() {
  Serial.println("Press ENTER to send next command...");
  // Clear any pending input
  while (Serial.available()) Serial.read();

  // Wait for newline
  while (true) {
    if (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n' || c == '\r') break;
    }
    delay(5);
  }
}

static void emergencyStop() {
  Serial.println("!!! EMERGENCY STOP !!!");
  drive.stop();
}
// -------------------- Arduino setup/loop --------------------
void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println("=== Protocol-driven DriveTrain script test ===");

  drive.setMaxSpeed(255);
  drive.setDeadband(0);
  drive.setPwmFrequency(2000); 
  drive.begin();

  // Wire protocol callbacks
  Protocol::Callbacks cb;
  cb.onDrive = onDrive;
  proto.setCallbacks(cb);

  Serial.println("Setup complete.");
}

void loop() {
  // Always keep parsing anything pending
  proto.poll();

  static size_t idx = 0;
  static bool first = true;

  if (first) {
    Serial.println("=== Step mode: one command per ENTER ===");
    Serial.println("Type 's' + ENTER for emergency stop anytime.");
    first = false;
  }

  // If user typed 's' or 'S', stop immediately
  if (Serial.available()) {
    char c = (char)Serial.peek();
    if (c == 's' || c == 'S') {
      // consume the line
      while (Serial.available()) {
        char x = (char)Serial.read();
        if (x == '\n' || x == '\r') break;
      }
      emergencyStop();
      return;
    }
  }

  // Show next command
  Serial.print("\n\n[next ");
  Serial.print(idx);
  Serial.print("] ");
  Serial.println(SCRIPT[idx]);

  // Wait for Enter keypress to send it
  waitForEnter();

  // Inject next scripted JSON line into Protocol as if it arrived over serial
  Serial.print("[inject] ");
  Serial.println(SCRIPT[idx]);
  scriptedIO.injectLine(SCRIPT[idx]);

  // Give protocol a moment to parse and apply the command
  for (int i = 0; i < 50; i++) {  // ~250ms total
    proto.poll();
    delay(5);
  }

  // Advance
  idx = (idx + 1) % SCRIPT_LEN;
}