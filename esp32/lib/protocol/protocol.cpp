#include "protocol.h"
#include "drivebase.h"

namespace {
  // Small helper: safe string extraction
  const char* jstr(JsonVariant v, const char* fallback = "") {
    const char* s = v | fallback;
    return s ? s : fallback;
  }
}

Protocol::Protocol(Stream& serial)
: _serial(serial) {
  _lineBuf[0] = '\0';
}

void Protocol::sendJson(const JsonDocument& doc) {
  serializeJson(doc, _serial);
  _serial.print('\n');
}

void Protocol::sendHelloAck(uint32_t seq) {
  JsonDocument doc;
  doc["v"] = 1;
  doc["type"] = "hello_ack";
  doc["name"] = "esp32-body";
  doc["fw"] = "0.1.0";
  doc["seq"] = seq;
  sendJson(doc);
}

void Protocol::sendPong(uint32_t seq, uint32_t t) {
  JsonDocument doc;
  doc["v"] = 1;
  doc["type"] = "pong";
  doc["seq"] = seq;
  doc["t"] = t;
  sendJson(doc);
}

void Protocol::sendAck(uint32_t seq, const char* ackType) {
  JsonDocument doc;
  doc["v"] = 1;
  doc["type"] = "ack";
  doc["seq"] = seq;
  doc["ack_type"] = ackType;
  sendJson(doc);
}

void Protocol::sendErr(const char* msg, uint32_t seq, const char* detail, const char* got) {
  JsonDocument e;
  e["v"] = 1;
  e["type"] = "err";
  e["msg"] = msg;
  if (seq != 0) e["seq"] = seq;
  if (detail)  e["detail"] = detail;
  if (got)     e["got"] = got;
  sendJson(e);
}

void Protocol::sendBoot(uint32_t uptimeSeconds) {
  JsonDocument boot;
  boot["v"] = 1;
  boot["type"] = "boot";
  boot["name"] = "esp32-body";
  boot["fw"] = "0.1.0";
  boot["uptime"] = uptimeSeconds;
  sendJson(boot);
}

void Protocol::poll() {
  // Read bytes and assemble newline-delimited messages.
  while (_serial.available()) {
    char c = (char)_serial.read();

    if (c == '\r') {
      // Ignore CR so Windows line endings don't break us.
      continue;
    }

    if (c == '\n') {
      // End of message. Null-terminate and handle if non-empty.
      _lineBuf[_lineLen] = '\0';
      if (_lineLen > 0) {
        handleLine(_lineBuf);
      }
      _lineLen = 0;
      continue;
    }

    // Regular character: append if there is room.
    if (_lineLen < (LINE_BUF - 1)) {
      _lineBuf[_lineLen++] = c;
    } else {
      // Line overflow. Reset and report error.
      _lineLen = 0;
      sendErr("line_too_long");
    }
  }
}

void Protocol::handleLine(const char* line) {
  // Parse JSON line into a document.
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, line);

  if (err) {
    // Keep detail short; ArduinoJson returns a compact string.
    sendErr("json_parse_failed", 0, err.c_str());
    return;
  }

  _msgCount++;
  dispatch(doc);
}

void Protocol::dispatch(JsonDocument& doc) {
  // Pull common envelope fields.
  const char* type = jstr(doc["type"], "");
  // seq might be missing for events; default to 0.
  uint32_t seq = doc["seq"] | 0;

  // Small dispatch table 
  // Each entry maps a type string to a member function handler.
  struct Entry { const char* type; void (Protocol::*fn)(JsonDocument&); };

  static const Entry kTable[] = {
    {"hello", &Protocol::onHello},
    {"ping",  &Protocol::onPing},
    {"drv",   &Protocol::onDrv},
    {"mode",  &Protocol::onMode},
    {"feed", &Protocol::onFeed},
    {"drv2", &Protocol::onDrv2}
  };

  for (const auto& e : kTable) {
    if (strcmp(type, e.type) == 0) {
      (this->*e.fn)(doc);
      return;
    }
  }

  // Unknown message type: report what we received.
  sendErr("unknown_type", seq, nullptr, type);
}

void Protocol::onHello(JsonDocument& doc) {
  uint32_t seq = doc["seq"] | 0;
  const char* name = jstr(doc["name"], "unknown");
  const char* sw   = jstr(doc["sw"],   "unknown");

  // Protocol behavior: always ACK hello.
  sendHelloAck(seq);

  // Optional callback for app logic.
  if (_cb.onHello) _cb.onHello(seq, name, sw);
}

void Protocol::onPing(JsonDocument& doc) {
  uint32_t seq = doc["seq"] | 0;
  uint32_t t   = doc["t"]   | 0;

  // Protocol behavior: always pong.
  sendPong(seq, t);

  if (_cb.onPing) _cb.onPing(seq, t);
}

void Protocol::onDrv(JsonDocument& doc) {
  uint32_t seq = doc["seq"] | 0;

  DriveCmd cmd;
  cmd.thr = doc["thr"] | 0;
  cmd.str = doc["str"] | 0;

  // Let the robot code apply the command.
  if (_cb.onDrive) _cb.onDrive(seq, cmd);

  // Keep protocol simple: ack regardless (or make this conditional later).
  sendAck(seq, "drv");
}

void Protocol::onMode(JsonDocument& doc) {
  uint32_t seq = doc["seq"] | 0;
  const char* name = jstr(doc["name"], "unknown");

  if (_cb.onMode) _cb.onMode(seq, name);

  sendAck(seq, "mode");
}

void Protocol::onFeed(JsonDocument& doc) {
  uint32_t seq = doc["seq"] | 0;

  const char* cmdStr = jstr(doc["cmd"], "");
  uint32_t ms = doc["ms"] | 700;   // default open duration

  FeedCmd cmd{};
  if (strcmp(cmdStr, "open") == 0) {
    cmd.open = true;
  } else if (strcmp(cmdStr, "close") == 0) {
    cmd.open = false;
  } else {
    sendErr("bad_feed_cmd", seq, nullptr, cmdStr);
    return;
  }
  cmd.ms = ms;

  if (_cb.onFeed) _cb.onFeed(seq, cmd);

  // Always ACK if command was valid
  sendAck(seq, "feed");
}
void Protocol::onDrv2(JsonDocument& doc) {
  uint32_t seq = doc["seq"] | 0;

  Drive2Cmd cmd;
  cmd.left_u  = doc["left"]  | 0.0f;
  cmd.right_u = doc["right"] | 0.0f;

  if (_cb.onDrive2) _cb.onDrive2(seq, cmd);

  sendAck(seq, "drv2");
}

void Protocol::sendDriveTelemetry(const DriveTelemetry& tel) {
  JsonDocument doc;
  doc["type"]  = "enc";
  doc["seq"]   = _txSeq++;
  doc["t_ms"]  = tel.t_ms;

  JsonObject left  = doc["left"].to<JsonObject>();
  left["dt"]    = tel.left.dticks;
  left["tot"]   = (long long)tel.left.ticks_total;
  left["rad_s"] = tel.left.wheel_rad_s;

  JsonObject right = doc["right"].to<JsonObject>();
  right["dt"]    = tel.right.dticks;
  right["tot"]   = (long long)tel.right.ticks_total;
  right["rad_s"] = tel.right.wheel_rad_s;

  sendJson(doc);
}

void Protocol::sendTof(const TofReading& r) {
  JsonDocument doc;             
  doc["type"]   = "tof";
  doc["seq"]    = _txSeq++;         // optional but consistent with other outgoing messages
  doc["id"]     = r.id;
  doc["mm"]     = r.mm;
  doc["status"] = r.status;
  doc["ts_ms"]  = r.ts_ms;

  sendJson(doc);
}

// --------------protocol.cpp (additions)
static uint32_t msOrNow(uint32_t t_ms) { return t_ms ? t_ms : millis(); }

void Protocol::sendSensorPacketBegin(JsonDocument& doc, uint32_t t_ms) {
  doc.clear();
  doc["v"] = 1;
  doc["type"] = "sens";
  doc["seq"] = _txSeq++;
  doc["t_ms"] = msOrNow(t_ms);
  //doc.createNestedObject("data"); // caller populates
  doc["data"].to<JsonObject>();//caller popluates
}

void Protocol::sendSensorPacketEnd(const JsonDocument& doc) {
  sendJson(doc);
}

void Protocol::sendSensorBool(const char* key, bool value, uint32_t t_ms) {
  JsonDocument doc;
  sendSensorPacketBegin(doc, t_ms);
  doc["data"][key] = value ? 1 : 0;
  sendSensorPacketEnd(doc);
}

void Protocol::sendSensorInt(const char* key, int32_t value, uint32_t t_ms) {
  JsonDocument doc;
  sendSensorPacketBegin(doc, t_ms);
  doc["data"][key] = value;
  sendSensorPacketEnd(doc);
}

void Protocol::sendSensorFloat(const char* key, float value, uint32_t t_ms) {
  JsonDocument doc;
  sendSensorPacketBegin(doc, t_ms);
  doc["data"][key] = value;
  sendSensorPacketEnd(doc);
}
