# ESP-ModbusRTUServer

A small Modbus RTU server for Arduino and ESP-IDF, on ESP8266 and ESP32. It has coils, holding, discrete and
input registers, a TX enable pin for RS-485, an echo filter for transceivers that hear themselves, and optional
framing of your own for buses that don't keep Modbus' pauses between frames.

It's what [esphome-flexit-modbus-server](https://github.com/MSkjel/esphome-flexit-modbus-server) is built on.
That pulls it in by itself, nothing to install.

## Use

```cpp
#include <ModbusRTUServer.h>

ModbusRTUServer mb;

void setup() {
  Serial.begin(115200);
  // server ID 1, TX enable on pin 4 (active high), 16 coils, 32 holding registers
  mb.begin(&Serial, 115200, 1, 4, true, 16, 32);
  mb.setHoldingRegister(0, 42);
}

void loop() { mb.update(); }
```

Off Arduino, implement `ModbusRTUServer::Port` (read, write, flush) for your UART and pass that to `begin()`.
The callbacks, the echo filter, custom framing and leaving function codes out with `-DMODBUS_DISABLE_<name>` are
described in [ModbusRTUServer.h](ModbusRTUServer.h).

## Tests

`make -C tests` runs the host tests with the sanitizers.
