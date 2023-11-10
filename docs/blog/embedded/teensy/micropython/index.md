---
tags:
    - teensy
    - micro python
    - embedded
    - vscode
    - pymakr
---

First step config teensy 4.1 with MicroPython
Set VSCode with pymaker extension


## Build teensy loader cli
build cli to upload micropython to teensy board

- Download source from [git](https://github.com/PaulStoffregen/teensy_loader_cli)
- Install `sudo apt-get install libusb-dev`
- Run `make`


## load MicroPython firmware
```bash
teensy_loader_cli --mcu=imxrt1062 -v -w TEENSY41-<date_version_tag>.hex
#
teensy_loader_cli --mcu=imxrt1062 -v -w ~/Downloads/TEENSY41-20231005-v1.21.0.hex
```

---

## usage

```bash
sudo apt install picocom
```

```bash
picocom -b 115200 /dev/ttyACM0
#
# press enter
# get python prompt
>>>
# exit
ctrl-a ctrl-q
```

---

## VSCode
- install [Pymaker]()
- Create new project : `PyMakr: Create new project`


---

## Blink Example

```python
import machine
from machine import Pin
led = Pin('LED', Pin.OUT)
led.on()
led.off()
import time
for i in range(10):
    time.sleep_ms(250)
    led.on()
    time.sleep_ms(250)
    led.off()
```

---
## Reference
- [Teensy Loader, Command Line](https://www.pjrc.com/teensy/loader_cli.html)
- [Micropython Teensy 4.1](https://micropython.org/download/TEENSY41/)
- [MicroPython teensy](https://forum.micropython.org/viewtopic.php?t=6783&start=280)