---
title: ROS2 action cli
tags:
    - action
    - cli
---

## Demo

```
ros2 run turtlesim turtlesim_node
```

### Check for actions
```bash title="cli"
ros2 action list
/turtle1/rotate_absolute

# -t show action type
ros2 action list -t
/turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
```

### Show interface

```bash
ros2 interface show turtlesim/action/RotateAbsolute 
# Request: The desired heading in radians
float32 theta
---
# Result: The angular displacement in radians to the starting position
float32 delta
---
# Feedback: The remaining rotation in radians
float32 remaining
```

### send goal

```bash
# ros2 action send_goal action_name action_type value
# -f show feedback
ros2 action send_goal -f /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.575}


Waiting for an action server to become available...
Sending goal:
     theta: 1.575

Feedback:
    remaining: 1.5750000476837158

Goal accepted with ID: 8340924a5f1c4e75b36ae3d95f399429

Feedback:
    remaining: 1.559000015258789

...

Feedback:
    remaining: 0.007000088691711426

Result:
    delta: -1.5679999589920044

Goal finished with status: SUCCEEDED


```

