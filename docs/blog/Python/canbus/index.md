---
tags:
    - can
    - canbus
    - python
---

```
pip install python-can
```

## read demo

```python
import can
import time

bus = can.interface.Bus(interface="socketcan", channel="can0")

while True:
    print_listener = can.Printer()

    can.Notifier(bus, [print_listener])

    time.sleep(1.0)

can_bus.stop_listening()%  
```