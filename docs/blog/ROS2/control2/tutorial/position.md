---
tags:
    - ros2
    - ros2_control
---
# ROS2 control, simple tutorial base on one continues joint

# Objective
- Create simple `urdf` with two links and one continues joint
- Define controllers
    - Position
    - Velocity
    - Effort

```xml title="robot.urdf"
<?xml version="1.0"?>
<robot name="simple_example">
    <link name="world"/>

    <material name="blue">
        <color rgba="0 0 0.8 1"/>
    </material>

    <material name="black">
        <color rgba="0 0 0 1"/>
    </material>

    <joint name="word2base" type="fixed">
        <parent link="world"/>
        <child link="base_link"/>
    </joint>
    <link name="base_link">
        <inertial>
            <mass value="10" />
            <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
        </inertial>
        <collision>
            <geometry>
                <cylinder radius="0.05" length="0.24" />
            </geometry>
        </collision>
        <visual>
            <geometry>
                <cylinder radius="0.05" length="0.24" />
            </geometry>
            <material name="black"/>
        </visual>
    </link>

    <link name="second_link">
        <inertial>
            <mass value="0.18" />
            <inertia ixx="0.0002835" ixy="0.0" ixz="0.0" iyy="0.0002835" iyz="0.0" izz="0.000324" />
        </inertial>
        <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0" />
        <collision>
            <geometry>
                <box size="0.05 0.05 0.15" />
            </geometry>
        </collision>
        <visual>
            <geometry>
                <box size="0.05 0.05 0.15" />
            </geometry>
            <material name="blue"/>
        </visual>
    </link>

    <joint name="base_to_second_joint" type="continuous">
        <parent link="base_link"/>
        <child link="second_link"/>
        <axis xyz="0 0 1"/>
        <origin xyz="0.0 0.0 0.2" rpy="0.0 0.0 0.0"/>
    </joint>
</robot>
```

## Position controller
- Add ros2_control definition to urdf
- Add gazebo plugin definition to urdf
- Add gazebo plugin yaml file
- Load controller
- Pub command


```xml title="ros2_control"
<ros2_control name="GazeboSystem" type="system">
    <hardware>
        <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>    
    <joint name="base_to_second_joint">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
    </joint>
</ros2_control>
```

```xml title="ros2_Control plugin"
<gazebo>
<plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find simple_joint)/config/position.yaml</parameters>
</plugin>
</gazebo>
```

```yaml title="position.yaml"
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    position_controller:
      type: position_controllers/JointGroupPositionController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

position_controller:
  ros__parameters:
    joints:
      - camera2base
    interface_name: position
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

joint_state_broadcaster:
  ros__parameters:
      joints:
        - camera2base
```

### usage

```bash title="load controllers"
# joint_state
ros2 run controller_manager spawner joint_state_broadcaster

# position controller
ros2 run controller_manager spawner position_controller
```

```bash title="topics" linenums="1" hl_lines="5 10"
# topics
ros2 topic list

/clock
/dynamic_joint_states
/joint_state_broadcaster/transition_event
/joint_states
/parameter_events
/performance_metrics
/position_controller/commands
/position_controller/transition_event
/robot_description
/rosout
/tf
/tf_static

```

#### position command
```bash title="pub position command"
ros2 topic pub --once \
/position_controller/commands \
std_msgs/msg/Float64MultiArray "{data: [1.575]}"
```

#### state
```bash
ros2 topic echo /dynamic_joint_states
# 
---
header:
  stamp:
    sec: 108
    nanosec: 430000000
  frame_id: ''
joint_names:
- camera2base
interface_values:
- interface_names:
  - position
  - velocity
  - effort
  values:
  - 1.5750000000000002
  - -2.7191856406436335e-23
  - 0.0
---

```