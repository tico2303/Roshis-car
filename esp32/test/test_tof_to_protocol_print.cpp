#include <Arduino.h>
#include <Wire.h>

#include "tof_manager.h"
#include "robot_config.h"   // if you keep I2C pins here; otherwise remove
#include "configs.h"        // where your TofConfig array lives

#include <ArduinoJson.h>

static TofManager tof(RobotConfig::TOF_CFG, RobotConfig::TOF_CFG_COUNT);

static uint32_t g_seq = 1;

static void printTofJson(const TofReading& r) {
  JsonDocument doc;
  doc["type"]   = "tof";
  doc["seq"]    = g_seq++;
  doc["id"]     = r.id;
  doc["mm"]     = r.mm;
  doc["status"] = r.status;
  doc["ts_ms"]  = r.ts_ms;

  serializeJson(doc, Serial);
  Serial.print('\n'); // NDJSON framing
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n--- TOF Protocol Print Test ---");

  tof.setPeriodMs(50);
  tof.setAutoRecover(true);
  tof.setRecoverThreshold(5);

  Wire.begin();
  Wire.setClock(100000);

  bool ok = tof.begin(Wire);
  Serial.print("[test] tof.begin() = ");
  Serial.println(ok ? "OK" : "FAIL");

  if (!ok) {
    Serial.println("[test] If FAIL: check power, SDA/SCL pins, XSHUT wiring, addresses.");
  }
}

void loop() {
  TofReading r{};
  if (tof.poll(r)) {
    printTofJson(r);
  }

  delay(1);
}