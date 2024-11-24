---
tags:
    - zenoh
    - pico
---

Zenoh-pico is the Eclipse zenoh implementation that targets constrained devices

## Simple demo
Try X64 implementation

version 1.02 [pico download](https://download.eclipse.org/zenoh/zenoh-pico/1.0.2/)

- Download x64-debian
- download x64-examples
- pull zenohd router as docker , select the correct version


```
docker pull eclipse/zenoh:1.0.2
```

```
docker run --init --net host eclipse/zenoh:1.0.2
```

```bash
./z_sub
#
>> [Subscriber] Received ('demo/example/zenoh-pico-pub': '[  30] Pub from Pico!')
>> [Subscriber] Received ('demo/example/zenoh-pico-pub': '[  31] Pub from Pico!')

```

```bash
./z_pub
#
Putting Data ('demo/example/zenoh-pico-pub': '[  32] Pub from Pico!')...
Putting Data ('demo/example/zenoh-pico-pub': '[  33] Pub from Pico!')...

```

---

## Demo: Pub data from arduino

- Run router on pc
- Run Subscriber `z_sub` on pc
- Run Publisher on Arduino framework (ESP32)

[code example](https://github.com/eclipse-zenoh/zenoh-pico/blob/main/examples/arduino/z_pub.ino)


```ini title=platformio.ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
; framework = espidf
upload_port = /dev/ttyUSB1
monitor_port= /dev/ttyUSB1
monitor_speed = 115200
build_flags =
    -DZENOH_COMPILER_GCC
    -DZENOH_ARDUINO_ESP32
lib_deps = https://github.com/eclipse-zenoh/zenoh-pico
```

---

## Reference
- [github](https://github.com/eclipse-zenoh/zenoh-pico/tree/main)
- [The Power of Zenoh Pico - Lightweight IoT Connectivity for Microcontrollers](https://www.youtube.com/watch?v=OfvVS0oaT6s)