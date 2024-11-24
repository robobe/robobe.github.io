---
tags:
    - zenoh
---



- Resource: A named data, key value
  - /home/kitchen/sensor/temp, 22.5
  - /home/kitchen/sensor/hum, 0.67
- Key expression: Identifying a set of keys
  - /home/kitchen/sensor/*
- Selector: Identifying a set of resources
  - /home/*/sensor/air?co2>12[humidity]
- Publisher: Source of values
- Subscribers: Sink of value for a key expression
- Queryable: return values for key expression


Scouting
Discover zenoh runtime and infrastructure (route) on network