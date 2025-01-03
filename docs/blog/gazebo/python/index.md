---
tags:
    - gazebo
    - gz
    - python
    - bindings
---

# GZ Python bindings

```bash
# install transport binding ()
sudo apt install python3-gz-transport13
```


```bash title="check bindings installed"
dpkg -l | grep python3-gz
ii  python3-gz-math7                                  7.5.1-1~jammy                           amd64        Gazebo Math Library - Python3 bindings
ii  python3-gz-msgs10                                 10.3.1-1~jammy                          amd64        Set of message definitions used by robotics apps - Dev files
ii  python3-gz-sim8                                   8.7.0-1~jammy                           amd64        Gazebo Sim classes and functions for robot apps - Development files
ii  python3-gz-transport13                            13.4.0-2~jammy                          amd64        Gazebo transport Library - Python3 bindings
```

!!! tip "msgs"
    protobuf msgs location: `/usr/lib/python3/dist-packages/gz/msgs10/` 


```python
from gz.msgs10.clock_pb2 import Clock

from gz.transport13 import Node
import signal

def handler(msg):
    print(msg)

def main():
    node = Node()
    topic = "clock"

    sub = node.subscribe(Clock, topic, handler)

    signal.pause()

if __name__ == "__main__":
    main()
```
     