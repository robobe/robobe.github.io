---
title: Odom frame
tags:
    - gazebo classic
    - tutorial
    - odom
---


## diff drive
using odom tf from diff drive plugin

- change `publish_odom_tf` from false to true
  
```xml
<publish_odom_tf>true</publish_odom_tf>
```

![](images/gazebo_rviz_odom_frame.png)

---

## Gazebo garden
[diff drive ](https://gazebosim.org/api/sim/7/classgz_1_1sim_1_1systems_1_1DiffDrive.html#details)

```xml
<gazebo>
    <plugin
        filename="gz-sim-diff-drive-system"
        name="gz::sim::systems::DiffDrive">
        <left_joint>drivewhl_l_joint</left_joint>
        <right_joint>drivewhl_r_joint</right_joint>
        <wheel_separation>0.4</wheel_separation>
        <wheel_radius>0.1</wheel_radius>
        <odom_publish_frequency>1</odom_publish_frequency>
        <topic>cmd_vel</topic>
        <tf_topic>tf</tf_topic>
        <frame_id>odom</frame_id>
        <child_frame_id>base_link</child_frame_id>
    </plugin>
</gazebo>
```

```python title="bridge"
bridge2 = Node(
        name="bridge2",
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/cart/model/robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            "/model/robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
        ],
        remappings=[
            ('/world/cart/model/robot/joint_state', 'joint_states'),
            ('/model/robot/odometry', '/odom')
        ],
        output='screen'
    )
```