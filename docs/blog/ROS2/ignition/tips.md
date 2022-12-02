---
title: Ignition gazebo tips
tags:
    - gazebo
    - tips
---

## dashboard
![[gazebo dashboard](https://app.gazebosim.org/dashboard)](image/dashboard.png)
[gazebo dashboard](https://app.gazebosim.org/dashboard)

---

## include same model multiple times

```xml
<include>
    <uri>model://coke_can</uri>
    <name>coke1</name>
    <pose>0 2 0 0 0 0</pose>
</include>

<include>
    <uri>model://coke_can</uri>
    <name>coke2</name>
    <pose>0 -5 1 0 0 0</pose>
</include>
```

---

## include from fuel

```xml
<include>
    <name>j1</name>
    <pose frame=''>0 0 0 0 0 0</pose>
    <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Jersey Barrier</uri>
</include>
```