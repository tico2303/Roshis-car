#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include "tof/tof_manager.h"
/**
 * Robot Serial Protocol (newline-delimited JSON).
 *
 * Goals:
 *  - Small, readable, and easy to extend with new message types.
 *  - Minimal dynamic allocation (uses StaticJsonDocument).
 *  - Friendly interface via callbacks for robot behavior.
 */
class Protocol {
public:

  struct DriveCmd {
    int thr;   // throttle, suggested -100..100
    int str;   // steering, suggested -100..100
  };
  struct FeedCmd {
      bool open;       // true=open, false=close
      uint32_t ms;     // optional duration (for open)
    };

  // Extend Commands here

  

  struct Callbacks {
    // Called when Pi says hello. (You can record peer name, etc.)
    void (*onHello)(uint32_t seq, const char* name, const char* sw) = nullptr;

    // Called on ping; default behavior will still send pong automatically.
    void (*onPing)(uint32_t seq, uint32_t t) = nullptr;

    // Called when drive command is received.
    void (*onDrive)(uint32_t seq, const DriveCmd& cmd) = nullptr;

    // Called when mode is requested.
    void (*onMode)(uint32_t seq, const char* modeName) = nullptr;
    // Called when feeder command is requested
    void (*onFeed)(uint32_t seq, const FeedCmd& cmd) = nullptr;
  };

  explicit Protocol(Stream& serial);

  // Set optional callbacks (you can set only what you need).
  void setCallbacks(const Callbacks& cb) { _cb = cb; }

  // Call frequently in loop(); processes any available bytes.
  void poll();

  // Stats / convenience
  uint32_t msgCount() const { return _msgCount; }

  // Outgoing messages (ESP32 -> Pi)
  void sendBoot(uint32_t uptimeSeconds = 0);
  void sendHelloAck(uint32_t seq);
  void sendPong(uint32_t seq, uint32_t t);
  void sendAck(uint32_t seq, const char* ackType);
  void sendErr(const char* msg, uint32_t seq = 0, const char* detail = nullptr, const char* got = nullptr);
  //Time of Flight sensor
  void sendTof(const TofReading& r);

  // Send sensor data (ESP32 -> Pi)
  void sendSensorBool(const char* key, bool value, uint32_t t_ms = 0);
  void sendSensorInt(const char* key, int32_t value, uint32_t t_ms = 0);
  void sendSensorFloat(const char* key, float value, uint32_t t_ms = 0);

  // Optional: begin/end a packet for batching multiple fields
  void sendSensorPacketBegin(JsonDocument& doc, uint32_t t_ms = 0);
  void sendSensorPacketEnd(const JsonDocument& doc);

private:
  // --- Parsing/framing ---
  static constexpr size_t LINE_BUF = 256;   // adjust as needed
  char   _lineBuf[LINE_BUF];
  size_t _lineLen = 0;

  Stream& _serial;
  Callbacks _cb{};
  uint32_t _msgCount = 0;

  // --- Helpers ---
  void sendJson(const JsonDocument& doc);
  void handleLine(const char* line);

  // Dispatch based on "type"
  void dispatch(JsonDocument& doc);

  // Individual handlers
  void onHello(JsonDocument& doc);
  void onPing(JsonDocument& doc);
  void onDrv(JsonDocument& doc);
  void onMode(JsonDocument& doc);
  void onFeed(JsonDocument& doc);

  uint32_t _txSeq = 1;
};
