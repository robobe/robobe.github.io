---
tags:
    - gazebo
    - joint
    - sdf
    - force
    - friction
    - damping
---

# Apply joint force


```xml
<joint name="j1" type="revolute">
    <parent>base_link</parent>
    <child>rotor</child>
    <axis>
        <xyz>0 0 1</xyz>
        <dynamics>
            <damping>0.1</damping>
            <friction>0.2</friction> <!-- Friction coefficient -->
        </dynamics>
    </axis>
</joint>
```

## damping (joints)
joint damping refers to a resistive force or torque that opposes the motion of the joint, proportional to its velocity. It is used to simulate energy dissipation in the joint, such as internal friction, air resistance, or viscous damping.

## friction (joints)
Friction in a joint models resistance to motion and can be rotational (torque) or linear (force).

```
```

## apply force

Total Force (or Torque)=Friction+Other Resistive Forces (e.g., damping, inertia)

example:
- friction: 0.2
- damping: 0.1
- angular velocity: 2

```
τdamping​ = −c⋅ω
τrequired​ = friction + damping
τrequired​=0.2+0.1⋅2=0.4Nm
```


```
gz topic -t /model/joint_force_test/joint/j1/cmd_force -m gz.msgs.Double -p "data: 0.0"
```

---

## Demo

```xml
<?xml version="1.0"?>

<!-- 

gz topic -t /model/joint_force_test/joint/j1/cmd_force -m gz.msgs.Double -p "data: 0.0"

 -->

<sdf version="1.7">
  <world name="empty">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>
    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    <plugin
      filename="gz-sim-contact-system"
      name="gz::sim::systems::Contact">
    </plugin>

    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <model name="joint_force_test">
      <pose>0 0 0.005 0 0 0</pose>
      <link name="base_link">
        <pose>0.0 0.0 0.0 0 0 0</pose>
        <inertial>
          <inertia>
            <ixx>2.501</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>2.501</iyy>
            <iyz>0</iyz>
            <izz>5</izz>
          </inertia>
          <mass>120.0</mass>
        </inertial>
        <visual name="base_visual">
          <pose>0.0 0.0 0.0 0 0 0</pose>
          <geometry>
            <box>
              <size>0.5 0.5 0.01</size>
            </box>
          </geometry>
        </visual>
        <collision name="base_collision">
          <pose>0.0 0.0 0.0 0 0 0</pose>
          <geometry>
            <box>
              <size>0.5 0.5 0.01</size>
            </box>
          </geometry>
        </collision>
      </link>
      <link name="rotor">
        <pose>0.0 0.0 1.0 0.0 0 0</pose>
        <inertial>
          <pose>0.0 0.0 0.0 0 0 0</pose>
          <inertia>
            <ixx>0.032</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.032</iyy>
            <iyz>0</iyz>
            <izz>0.00012</izz>
          </inertia>
          <mass>0.6</mass>
        </inertial>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.25 0.25 0.05</size>
            </box>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.25 0.25 0.05</size>
            </box>
          </geometry>
        </collision>
      </link>

      <joint name="j1" type="revolute">
        <pose>0 0 -0.5 0 0 0</pose>
        <parent>base_link</parent>
        <child>rotor</child>
        <axis>
          <xyz>0 0 1</xyz>
          <dynamics>
            <damping>0.1</damping>
            <friction>0.2</friction> <!-- Friction coefficient -->
          </dynamics>
        </axis>
      </joint>
      <plugin
        filename="gz-sim-apply-joint-force-system"
        name="gz::sim::systems::ApplyJointForce">
        <joint_name>j1</joint_name>
      </plugin>
    </model>
  </world>
</sdf>
```