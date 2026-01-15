#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>
#include <ArduinoJson.h>

class Protocol {
public:
  explicit Protocol(Stream& serial);
  void sendBoot();
  void sendJson(const JsonDocument& doc);
  void poll();
  uint32_t msgCount() const;

private:
  Stream& _serial;
  static const size_t LINE_BUF = 256;
  char _lineBuf[LINE_BUF];
  size_t _lineLen;
  uint32_t _msgCount;

  void sendHelloAck(uint32_t seq);
  void sendPong(uint32_t seq, uint32_t t);
  void sendAck(uint32_t seq, const char* ackType);
  void handleMessage(JsonDocument& doc);
};

#endif // PROTOCOL_H
