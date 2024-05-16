---
tags:
    - gazebo
    - classic
    - sdf
    - urdf
    - tips
---

## Convert from urdf

```
gz sdf -p /my_urdf.urdf > my_sdf.sdf
```

```
ign sdf -p my_urdf.urdf > my_urdf.sdf
```

## Send message to subscriber

```bash
# gz topic -p topic msg_type -m message_data
gz topic -p "/gazebo/default/iris_demo/gimbal_tilt_cmd"  "gazebo.msgs.GzString" -m 'data: "1.0"'
```