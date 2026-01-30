#include <Arduino.h>
#include "protocol.h"
#include "feeder.h"

static Protocol proto(Serial);
static Feeder feeder(RobotConfig::feederConfig);

static void handleFeed(uint32_t seq, const Protocol::FeedCmd& cmd) {
  (void)seq;

  if (cmd.open) {
    Serial.print("[FEED] open ms=");
    Serial.println(cmd.ms);
    feeder.open(cmd.ms);
  } else {
    Serial.println("[FEED] close");
    feeder.close();
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  feeder.begin();

  Protocol::Callbacks cb;
  cb.onFeed = handleFeed;
  proto.setCallbacks(cb);

  Serial.println();
  Serial.println("=== Feeder Protocol Test ===");
  Serial.println("Send (newline delimited):");
  Serial.println(R"({"v":1,"type":"feed","seq":1,"open":true,"ms":700})");
  Serial.println(R"({"v":1,"type":"feed","seq":2,"open":false})");
  Serial.println(R"({"v":1,"type":"feed","seq":3,"action":"open","ms":1500})");
  Serial.println(R"({"v":1,"type":"feed","seq":4,"action":"close"})");
  Serial.println("============================");

  Serial.println("[SELFTEST] open 1s");
  feeder.open(1000);
}

void loop() {
  proto.poll();     // reads serial + triggers callbacks
  feeder.update();  // handles auto-close timing
}