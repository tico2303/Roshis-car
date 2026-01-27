#include <Arduino.h>
#include "protocol.h"

Protocol proto(Serial);

static void handleDrive(uint32_t seq, const Protocol::DriveCmd& cmd) {
  // Apply motor control here.
  // cmd.thr, cmd.str are available.
  (void)seq;
}

static void handleMode(uint32_t seq, const char* mode) {
  // Switch state machine here.
  (void)seq;
  (void)mode;
}

void setup() {
  Serial.begin(115200);

  Protocol::Callbacks cb;
  
  cb.onDrive = handleDrive;
  cb.onMode  = handleMode;
  proto.setCallbacks(cb);

  proto.sendBoot(0);
}

void loop() {
  proto.poll();

  // Other robot logic...
}