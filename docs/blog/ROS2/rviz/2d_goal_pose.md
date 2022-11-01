---
title: 2D goal pose
tags:
    - rviz
    - pose
---

```bash
ros2 topic list
#
/goal_pose

ros2 topic info /goal_pose
#
Type: geometry_msgs/msg/PoseStamped
Publisher count: 1
```

```
ros2 interface show geometry_msgs/msg/PoseStamped

# A Pose with reference coordinate frame and timestamp

std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
Pose pose
	Point position
		float64 x
		float64 y
		float64 z
	Quaternion orientation
		float64 x 0
		float64 y 0
		float64 z 0
		float64 w 1

```