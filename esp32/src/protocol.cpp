#include "protocol.h"
#include <cstring>

Protocol::Protocol(Stream& serial)
    : _serial(serial), _lineLen(0), _msgCount(0) {
}

void Protocol::sendJson(const JsonDocument& doc) {
  serializeJson(doc, _serial);
  _serial.print('\n');
}

void Protocol::sendHelloAck(uint32_t seq) {
  StaticJsonDocument<256> doc;
  doc["v"] = 1;
  doc["type"] = "hello_ack";
  doc["name"] = "esp32-body";
  doc["fw"] = "0.1.0";
  doc["seq"] = seq;
  sendJson(doc);
}

void Protocol::sendPong(uint32_t seq, uint32_t t) {
  StaticJsonDocument<256> doc;
  doc["v"] = 1;
  doc["type"] = "pong";
  doc["t"] = t;
  doc["seq"] = seq;
  sendJson(doc);
}

void Protocol::sendAck(uint32_t seq, const char* ackType) {
  StaticJsonDocument<256> doc;
  doc["v"] = 1;
  doc["type"] = "ack";
  doc["ack_type"] = ackType;
  doc["seq"] = seq;
  sendJson(doc);
}

void Protocol::handleMessage(JsonDocument& doc) {
  const char* type = doc["type"] | "";
  uint32_t seq = doc["seq"] | 0;
  _msgCount++;

  if (strcmp(type, "hello") == 0) {
    sendHelloAck(seq);
    return;
  }

  if (strcmp(type, "ping") == 0) {
    uint32_t t = doc["t"] | 0;
    sendPong(seq, t);
    return;
  }

  if (strcmp(type, "drv") == 0) {
    int thr = doc["thr"] | 0;
    int str = doc["str"] | 0;
    (void)thr;
    (void)str;
    sendAck(seq, "drv");
    return;
  }

  if (strcmp(type, "mode") == 0) {
    const char* name = doc["name"] | "unknown";
    (void)name;
    sendAck(seq, "mode");
    return;
  }

  StaticJsonDocument<256> err;
  err["v"] = 1;
  err["type"] = "err";
  err["msg"] = "unknown_type";
  err["got"] = type;
  err["seq"] = seq;
  sendJson(err);
}

void Protocol::poll() {
  while (_serial.available()) {
    char c = (char)_serial.read();

    if (c == '\n') {
      _lineBuf[_lineLen] = '\0';

      if (_lineLen > 0) {
        StaticJsonDocument<512> doc;
        DeserializationError err = deserializeJson(doc, _lineBuf);

        if (!err) {
          handleMessage(doc);
        } else {
          StaticJsonDocument<256> e;
          e["v"] = 1;
          e["type"] = "err";
          e["msg"] = "json_parse_failed";
          e["detail"] = err.c_str();
          sendJson(e);
        }
      }

      _lineLen = 0;
      continue;
    }

    if (c == '\r') continue;

    if (_lineLen < LINE_BUF - 1) {
      _lineBuf[_lineLen++] = c;
    } else {
      _lineLen = 0;
      StaticJsonDocument<256> e;
      e["v"] = 1;
      e["type"] = "err";
      e["msg"] = "line_too_long";
      sendJson(e);
    }
  }
}

uint32_t Protocol::msgCount() const {
  return _msgCount;
}

void Protocol::sendBoot() {
  StaticJsonDocument<256> boot;
  boot["v"] = 1;
  boot["type"] = "boot";
  boot["name"] = "esp32-body";
  boot["fw"] = "0.1.0";
  boot["uptime"] = 0;
  sendJson(boot);
}
