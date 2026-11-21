#pragma once
#include <Arduino.h>

/* ----------------------------------------------------------------------
 *  Protocolo binario supervisor <-> sensor (Serial1)
 *
 *  Trama:  | opcode (1 byte) | len (1 byte) | body (len bytes) |
 *
 *  Peticiones (supervisor -> sensor) y respuestas (sensor -> supervisor):
 *
 *  OP_US        req: -                     ACK: [n, addr_1, ..., addr_n]
 *  OP_ONE_SHOT  req: [addr]                ACK: [addr, med_hi, med_lo, unit]
 *  OP_ON        req: [addr, per_hi, per_lo]  ACK: [addr]
 *  OP_OFF       req: [addr]                ACK: [addr]
 *  OP_UNIT      req: [addr, unit]          ACK: [addr]
 *  OP_DELAY     req: [addr, del_hi, del_lo]  ACK: [addr]
 *  OP_STATUS    req: [addr]                ACK: [addr, del_hi, del_lo, unit,
 *                                                periodic(0/1), per_hi, per_lo]
 *  Error en cualquier comando:             NACK: [err_code]
 *
 *  Asincrono (sensor -> supervisor, disparo periodico):
 *  OP_MEASSURE  [addr, med_hi, med_lo, unit]
 *
 *  Todos los valores de 16 bits van en big-endian.
 * ---------------------------------------------------------------------- */

#define MAX_BODY 32
#define RESPONSE_TIMEOUT_MS 1000UL
#define SERIAL_MONITOR_BAUDS 115200UL
#define SERIAL_BAUDS 9600UL

#define OP_ACK 0X00
#define OP_ONE_SHOT 0X01
#define OP_ON 0x02
#define OP_OFF 0x03
#define OP_UNIT 0X04
#define OP_DELAY 0x05
#define OP_STATUS 0x06
#define OP_US 0x07
#define OP_MEASSURE 0x20
#define OP_NACK 0xFF

#define UNIT_CMS 0X00
#define UNIT_INCS 0X01
#define UNIT_US 0x02

#define ERR_UNKNOWN_SENSOR 0x01
#define ERR_I2C 0x02
#define ERR_BAD_ARGS 0x03
#define ERR_UNKNOWN_OP 0x04
#define ERR_TOO_SOON 0x05  // no ha pasado el retardo minimo desde el ultimo disparo

struct Meassure {
  byte repr;
  const char *name;    // nombre aceptado en el comando "unit"
  const char *symbol;  // simbolo mostrado junto a la medida
};

constexpr Meassure meassures[] = {
  {UNIT_CMS, "cm", "cm"},
  {UNIT_INCS, "inc", "in"},
  {UNIT_US, "ms", "us"}  // el SRF02 devuelve el tiempo de vuelo en microsegundos
};

struct Frame {
  byte opcode;
  byte len;
  byte body[MAX_BODY];
};

bool sendFrame(byte opcode, byte len, const byte *body);

// Lee una trama completa; false si pasa timeoutMs sin recibir algun byte
bool readFrame(Frame &out, uint32_t timeoutMs = RESPONSE_TIMEOUT_MS);
