# Sensor SRF02

Firmware del dispositivo sensor para Arduino MKR WAN 1310. Recibe comandos de un
supervisor por `Serial1` y controla dos sensores de ultrasonidos SRF02 por I2C.

## Hardware

- Arduino MKR WAN 1310
- 2 sensores SRF02 en las direcciones `0xE0` y `0xE6`
- Una segunda placa que hace de supervisor

### Conexiones

| Pin MKR | Conexión                            |
| ------- | ----------------------------------- |
| 11      | SDA de los SRF02                    |
| 12      | SCL de los SRF02                    |
| 13 (RX) | TX del supervisor                   |
| 14 (TX) | RX del supervisor                   |
| GND     | GND común con sensores y supervisor |

Los SRF02 se alimentan a 5 V.

## Requisitos

- [VS Code](https://code.visualstudio.com/) con la extensión **PlatformIO IDE**, o
  [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html) (CLI)

PlatformIO descarga solo la plataforma `atmelsam` y el framework Arduino la
primera vez que se compila.

## Puerto serie

En `platformio.ini` el puerto de subida es `COM12`. Si tu placa aparece en otro
puerto, cámbialo:

```ini
upload_port = COM12
```

Para ver los puertos disponibles:

```
pio device list
```

## Compilar, subir y monitorizar

Desde la carpeta del proyecto:

```
pio run                          # compilar
pio run -t upload                # compilar y subir
pio device monitor -b 115200     # abrir el monitor serie
```

O todo de una vez:

```
pio run -t upload -t monitor
```

En VS Code se puede hacer lo mismo con los botones de la barra inferior de
PlatformIO (✓ compilar, → subir, 🔌 monitor).

Para salir del monitor: `Ctrl+C` (o `Ctrl+]` si no responde).
