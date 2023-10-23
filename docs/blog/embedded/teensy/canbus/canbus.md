---
tags:
    - teensy
    - can
    - canbus
---

# Teensy 4.1 canbus

![](images/teensy41_pinout.png)


![](images/inno_pinput.png)

---

## Software
[CAN Library for Teensy 4.0 / 4.1](https://github.com/pierremolinaro/acan-t4)


### vscode and platformio

```ini
[env:teensy41]
platform = teensy
board = teensy41
framework = arduino

lib_deps = 
    pierremolinaro/ACAN_T4@^1.1.5

```

---

## Reference
- [inno can-to-usb](https://docs.rs-online.com/5b40/A700000008257389.pdf)