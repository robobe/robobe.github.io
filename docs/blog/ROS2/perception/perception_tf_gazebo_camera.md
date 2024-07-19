---
tags:
    - ros2
    - gazebo
    - perception
    - tf
    - vision
    - camera
---


### right hand rule

![alt text](images/right_hand_rule.png)

### Camera coordinate system

![alt text](images/camera_coordinate_system.png)

![alt text](images/tf_optical_frame.png)

![alt text](images/ros_and_camera_coordinate.png)
[copy from Articulated Robotics](https://articulatedrobotics.xyz/tutorials/mobile-robot/hardware/camera/)


## Gazebo Add optical frame

```xml title="camera link urdf"
<link name="camera_link">
    <visual>
        <geometry>
            <box size="0.010 0.03 0.03"/>
        </geometry>
        <material name="red"/>
    </visual>
</link>
```

```xml title="optical link and joint urdf"
<joint name="camera_optical_joint" type="fixed">
    <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}" />
    <parent link="camera_link" />
    <child link="camera_link_optical" />
</joint>

<link name="camera_link_optical"></link>
```