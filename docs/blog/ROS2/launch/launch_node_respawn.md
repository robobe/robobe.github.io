---
title: Launch respawn node
tags:
    - ros2
    - launch
---
# LAB
- Ros restart process and file or exit

# Demo
```python
--8<-- "/home/user/projects/blog/docs/blog/ROS2/launch/files/launch_node_respawn.launch.py"
```

!!! tip ""
     You can run launch file without the package relation

     ```
     ros2 launch <launch file>
     ```

## usage
```bash
ros2 launch launch_node_respawn.launch.py
```

- Close the window
- After 4 sec ROS2 launch the node
- To Stop: exit from terminal