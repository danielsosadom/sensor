/* ----------------------------------------------------------------------
 *  Dispositivo sensor
 *    Atiende por Serial1 los comandos del supervisor y controla por I2C
 *    dos sensores de ultrasonidos SRF02.
 *
 *  Necesita protocol.h y protocol.cpp (los mismos que el supervisor).
 *
 *  Arduino MKR WAN 1310:
 *    Pin 11 -> SDA, Pin 12 -> SCL (I2C)
 *    Pin 13 -> RX, Pin 14 -> TX (Serial1, cruzados con el supervisor)
 * ---------------------------------------------------------------------- */

#include <Arduino.h>
#include <Wire.h>
#include "protocol.h"

#define SRF02_I2C_INIT_DELAY 100  // ms
#define SRF02_RANGING_DELAY 70    // ms
#define SRF02_MIN_DELAY 70        // retardo minimo permitido entre disparos (ms)

// Registros del SRF02
#define COMMAND_REGISTER byte(0x00)
#define SOFTWARE_REVISION byte(0x00)
#define RANGE_HIGH_BYTE byte(2)
#define RANGE_LOW_BYTE byte(3)

// Comandos del SRF02
#define REAL_RANGING_MODE_INCHES    byte(80)
#define REAL_RANGING_MODE_CMS       byte(81)
#define REAL_RANGING_MODE_USECS     byte(82)

struct Srf02 {
  byte address;         // direccion en formato de 8 bits (0xE0), la que usa el protocolo
  byte unit;            // UNIT_CMS, UNIT_INCS o UNIT_US
  uint16_t minDelay;    // retardo minimo entre disparos (ms)
  bool periodic;        // disparo periodico activado
  uint16_t period;      // periodo del disparo periodico (ms)
  uint32_t lastShotMs;  // millis() del ultimo disparo
};

Srf02 sensors[] = {
  {0xE0, UNIT_CMS, SRF02_MIN_DELAY, false, 0, 0},
  {0xE6, UNIT_CMS, SRF02_MIN_DELAY, false, 0, 0}
};
constexpr byte NUM_SENSORS = sizeof(sensors) / sizeof(sensors[0]);

uint8_t led_state = LOW;

void toggleLed() {
  digitalWrite(LED_BUILTIN, led_state); led_state = (led_state + 1) & 0x1;
}

/* ------------------------------- I2C -------------------------------- */
// Igual que en srf02_example, pero devolviendo false si el sensor no
// responde en lugar de quedarse colgado en while(!Wire.available()).

// La libreria Wire usa direcciones de 7 bits: 0xE0 -> 0x70
byte i2cAddress(byte address) { return address >> 1; }

bool write_command(byte address, byte command) {
  Wire.beginTransmission(i2cAddress(address));
  Wire.write(COMMAND_REGISTER);
  Wire.write(command);
  return Wire.endTransmission() == 0;
}

bool read_register(byte address, byte the_register, byte &value) {
  Wire.beginTransmission(i2cAddress(address));
  Wire.write(the_register);
  if (Wire.endTransmission() != 0) return false;

  if (Wire.requestFrom(i2cAddress(address), byte(1)) != 1) return false;
  value = Wire.read();
  return true;
}

byte rangingCommand(byte unit) {
  switch (unit) {
    case UNIT_INCS: return REAL_RANGING_MODE_INCHES;
    case UNIT_US:   return REAL_RANGING_MODE_USECS;
    default:        return REAL_RANGING_MODE_CMS;
  }
}

// Dispara el sensor y lee la medida en su unidad configurada
bool measure(Srf02 &s, uint16_t &range) {
  s.lastShotMs = millis();
  if (!write_command(s.address, rangingCommand(s.unit))) return false;
  delay(SRF02_RANGING_DELAY);

  byte high_byte, low_byte;
  if (!read_register(s.address, RANGE_HIGH_BYTE, high_byte)) return false;
  if (!read_register(s.address, RANGE_LOW_BYTE, low_byte)) return false;
  range = ((uint16_t) high_byte << 8) | low_byte;
  return true;
}

/* ---------------------------- protocolo ----------------------------- */

Srf02* findSensor(byte address) {
  for (Srf02 &s : sensors) {
    if (s.address == address) return &s;
  }
  return nullptr;
}

void sendNack(byte errorCode) {
  sendFrame(OP_NACK, 1, &errorCode);
  Serial.print("--> NACK, error "); Serial.println(errorCode);
}

void sendAck(byte len, const byte *body) {
  sendFrame(OP_ACK, len, body);
  Serial.println("--> ACK");
}

void printAddress(byte address) {
  Serial.print("0x"); Serial.print(address, HEX);
}

void handleFrame(const Frame &f) {
  Serial.print("<-- opcode "); printAddress(f.opcode);

  // us: lista de sensores, no lleva direccion
  if (f.opcode == OP_US) {
    Serial.println(" (us)");
    byte body[1 + NUM_SENSORS];
    body[0] = NUM_SENSORS;
    for (byte i = 0; i < NUM_SENSORS; i++) body[1 + i] = sensors[i].address;
    sendAck(sizeof(body), body);
    return;
  }

  if (f.opcode < OP_ONE_SHOT || f.opcode > OP_STATUS) {
    Serial.println(" desconocido");
    sendNack(ERR_UNKNOWN_OP);
    return;
  }

  // El resto de comandos llevan la direccion del sensor en body[0]
  if (f.len < 1) { Serial.println(); sendNack(ERR_BAD_ARGS); return; }
  Serial.print(" sensor "); printAddress(f.body[0]); Serial.println();

  Srf02 *s = findSensor(f.body[0]);
  if (s == nullptr) { sendNack(ERR_UNKNOWN_SENSOR); return; }

  switch (f.opcode) {
    case OP_ONE_SHOT: {
      if (millis() - s->lastShotMs < s->minDelay) { sendNack(ERR_TOO_SOON); return; }
      uint16_t range;
      if (!measure(*s, range)) { sendNack(ERR_I2C); return; }
      byte body[] = { s->address, (byte) (range >> 8), (byte) (range & 0xFF), s->unit };
      sendAck(sizeof(body), body);
      return;
    }

    case OP_ON: {
      if (f.len < 3) { sendNack(ERR_BAD_ARGS); return; }
      uint16_t period = ((uint16_t) f.body[1] << 8) | f.body[2];
      if (period == 0) { sendNack(ERR_BAD_ARGS); return; }
      s->period = period;
      s->periodic = true;
      sendAck(1, &s->address);
      return;
    }

    case OP_OFF:
      s->periodic = false;
      sendAck(1, &s->address);
      return;

    case OP_UNIT:
      if (f.len < 2 || f.body[1] > UNIT_US) { sendNack(ERR_BAD_ARGS); return; }
      s->unit = f.body[1];
      sendAck(1, &s->address);
      return;

    case OP_DELAY: {
      if (f.len < 3) { sendNack(ERR_BAD_ARGS); return; }
      uint16_t minDelay = ((uint16_t) f.body[1] << 8) | f.body[2];
      if (minDelay < SRF02_MIN_DELAY) { sendNack(ERR_BAD_ARGS); return; }
      s->minDelay = minDelay;
      sendAck(1, &s->address);
      return;
    }

    case OP_STATUS: {
      byte body[] = {
        s->address,
        (byte) (s->minDelay >> 8), (byte) (s->minDelay & 0xFF),
        s->unit,
        (byte) (s->periodic ? 1 : 0),
        (byte) (s->period >> 8), (byte) (s->period & 0xFF)
      };
      sendAck(sizeof(body), body);
      return;
    }
  }
}

/* --------------------------- setup y loop --------------------------- */

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  toggleLed();

  // Monitor serie solo para depurar: si no hay PC conectado no se queda esperando
  Serial.begin(SERIAL_MONITOR_BAUDS);
  while (!Serial) {}

  Serial1.begin(SERIAL_BAUDS);

  Wire.begin();
  delay(SRF02_I2C_INIT_DELAY);

  Serial.println("******************** sensor SRF02 ********************");
  for (Srf02 &s : sensors) {
    byte revision;
    Serial.print("SRF02 "); printAddress(s.address);
    if (read_register(s.address, SOFTWARE_REVISION, revision)) {
      Serial.print(" OK (revision "); printAddress(revision); Serial.println(")");
    } else {
      Serial.println(" NO RESPONDE");
    }
  }
}

void loop() {
  // Comandos del supervisor
  if (Serial1.available() > 0) {
    Frame frame;
    if (readFrame(frame)) {
      toggleLed();
      handleFrame(frame);
    }
  }

  // Disparos periodicos: nunca mas rapido que el retardo minimo
  for (Srf02 &s : sensors) {
    if (!s.periodic) continue;

    uint16_t wait = max(s.period, s.minDelay);
    if (millis() - s.lastShotMs < wait) continue;

    uint16_t range;
    if (measure(s, range)) {
      byte body[] = { s.address, (byte) (range >> 8), (byte) (range & 0xFF), s.unit };
      sendFrame(OP_MEASSURE, sizeof(body), body);
    } else {
      Serial.print("Error I2C en disparo periodico de "); printAddress(s.address); Serial.println();
    }
  }
}
