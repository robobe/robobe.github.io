---
title: Ignition programing
tags:
    - ignition
    - ign-transport
---

## Objective
- Subscriber to ign topic

## demo
### project
```
├── build
├── main.cpp
└── CMakeLists.txt
```
### code
```cpp title="main.cpp"
#include <iostream>
#include "ignition/msgs/clock.pb.h"
#include "ignition/transport/Node.hh"

void cb(const ignition::msgs::Clock &msg)
{
    std::cout << msg.DebugString() << std::endl;
}

int main(int, char**) {
    std::cout << "Hello, ignition !\n";
    std::string topic = "/clock";
    ignition::transport::Node node;
    node.Subscribe(topic, cb);
    ignition::transport::waitForShutdown();
}
```

```c title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.10.0)
project(ign_demos VERSION 0.1.0)

find_package(ignition-transport11 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

add_executable(main main.cpp)
target_link_libraries(main ignition-transport${IGN_TRANSPORT_VER}::core)
```

### Run demo

```bash title="terminal1"
ign gazebo -r
```

```bash title="terminal2"
./main

system {
  sec: 1666468493
  nsec: 778510659
}
real {
  sec: 15
  nsec: 459216049
}
sim {
  sec: 14
  nsec: 393000000
}
```

---

### Clock topic

```bash
ign topic --info -t /clock
#
Publishers [Address, Message Type]:
  tcp://172.18.0.1:34283, ignition.msgs.Clock
```

```bash
ign msg --info ignition.msgs.Clock
#
Name: ignition.msgs.Clock
File: ignition/msgs/clock.proto

message Clock {
  .ignition.msgs.Header header = 1;
  .ignition.msgs.Time system = 2;
  .ignition.msgs.Time real = 3;
  .ignition.msgs.Time sim = 4;
}

```