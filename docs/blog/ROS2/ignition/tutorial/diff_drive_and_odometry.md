---
title: Diff drive and odometry
tags:
    - diff_drive
    - odom
    - odometry
    - ros2
    - ignition
---

# LAB
- Add diff drive plugin
- publish `odometry`, `odometry tf` and `cmd_vel` from ignition to ros2 using bridge
  
!!! note 
    check [joint state post](joint_state_sdf_model.md) for project structure and files


!!! note 
    Install rqt_robotic_steering for GUI cmd_vel control

    ```bash
    sudo apt install ros-humble-rqt-robot-steering
    ```


![](images/diff_drive_ign_rviz_rqt_control.png)


# bridge
```bash title="" linenums="1" hl_lines="4-6"
ign topic --list
#
/clock
/gazebo/resource_paths
/gui/camera/pose
/model/basic_mobile_bot/cmd_vel
/model/basic_mobile_bot/odometry
/model/basic_mobile_bot/tf
/stats
/world/demo/clock
/world/demo/dynamic_pose/info
/world/demo/model/basic_mobile_bot/joint_state
/world/demo/pose/info
/world/demo/scene/deletion
/world/demo/scene/info
/world/demo/state
/world/demo/stats

```
- bridge cmd_vel (/model/basic_mobile_bot/cmd_vel)
- bridge odometry (/model/basic_mobile_bot/odometry)
- bridge tf (/model/basic_mobile_bot/tf)


```python
# cmd_vel bridge 
    cmd_vel_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
            namespace = namespace,
            name = 'cmd_vel_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }],
            arguments = [
                ign_model_prefix + '/cmd_vel' + '@geometry_msgs/msg/Twist' + ']ignition.msgs.Twist'
            ],
            remappings = [
                (ign_model_prefix + '/cmd_vel', '/cmd_vel')
            ])

    # odometry bridge 
    odometry_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
            namespace = namespace,
            name = 'odometry_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }],
            arguments = [
                    ign_model_prefix + '/odometry' + '@nav_msgs/msg/Odometry' + '[ignition.msgs.Odometry'
            ],
            remappings = [
                (ign_model_prefix + '/odometry', '/odom')
            ])

    # odom to base_link transform bridge
    odom_base_tf_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
            namespace = namespace,
            name = 'odom_base_tf_bridge',
            output = 'screen',
            parameters=[{
            'use_sim_time': use_sim_time
            }],
            arguments = [
                ign_model_prefix + '/tf' + '@tf2_msgs/msg/TFMessage' + '[ignition.msgs.Pose_V'
            ],
            remappings = [
                (ign_model_prefix + '/tf', '/tf')
            ])
```

[get bridge source](files/diff_drive_with_odometry/ign_bridge.launch.py)
---

# tf

![](images/diff_drive_and_odometry_tf_tree.png)