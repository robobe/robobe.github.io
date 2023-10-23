---
tags:
    - can
    - canbus
    - 
---

![](images/inno_usb_to_can.png)

```
sudo ip link set can0 type can bitrate 125000 
sudo ifconfig can0 up

sudo ip link set can1 type can bitrate 125000 sudo ifconfig can1 up
```

```bash
sudo apt install can-utils
```

```bash title="terminal1 - send"
cansend can0 5A1#11.22.33.44.55.66.77.88

```

```bash title="terminal2 - dump"
candump can1
  can1  5A1   [8]  11 22 33 44 55 66 77 88
```