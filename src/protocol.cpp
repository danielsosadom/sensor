#include <Arduino.h>
#include "protocol.h"

bool sendFrame(byte opcode, byte len, const byte *body) {
  if (len > MAX_BODY) return false;
  Serial1.write(opcode);
  Serial1.write(len);
  if (len > 0) Serial1.write(body, len);
  return true;
}

// Espera un byte por Serial1 como mucho timeoutMs (igual que echo_example)
static bool readByte(byte &data, uint32_t timeoutMs) {
  uint32_t last_ms = millis();
  while (millis() - last_ms < timeoutMs) {
    if (Serial1.available() > 0) {
      data = Serial1.read();
      return true;
    }
  }
  return false;
}

bool readFrame(Frame &out, uint32_t timeoutMs) {
  if (!readByte(out.opcode, timeoutMs)) return false;
  if (!readByte(out.len, timeoutMs)) return false;
  if (out.len > MAX_BODY) return false;
  for (byte i = 0; i < out.len; i++) {
    if (!readByte(out.body[i], timeoutMs)) return false;
  }
  return true;
}
