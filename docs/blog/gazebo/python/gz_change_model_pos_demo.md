---
tags:
    - gazebo
    - harmonic
    - python
    - service
    - pose
---

## Change model pose in work using service

```xml
<plugin filename="gz-sim-user-commands-system"
    name="gz::sim::systems::UserCommands">
</plugin>
```

```bash
gz service --list | grep pose
#
/gui/move_to/pose
/world/default/set_pose
```

## Change model position from cli

```bash
gz service -s /world/default/set_pose \
--reqtype gz.msgs.Pose \
--reptype gz.msgs.Boolean \
--timeout 300 \
--req \
'name: "simple_box", position: {x:5.0, y:1.0 z: 0.0}'
```

## Change model position from python

```python
from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean

# <plugin filename="gz-sim-user-commands-system"
#     name="gz::sim::systems::UserCommands">
# </plugin>

def main():
    node = Node()
    request = Pose()
    request.name = "simple_box"
    request.position.x = 0
    request.position.y = 5
    request.position.z = 1
    req = node.request("/world/default/set_pose",
                       request ,
                    Pose,
                    Boolean,
                    timeout=300)
    
    print(req)

if __name__ == "__main__":
    main()
```

```xml
<?xml version="1.0"?>
<sdf version="1.7">
    <world name="default">
        <scene>
            <ambient>0.5 0.5 0.5 1</ambient>
        </scene>

        <physics name="1ms" type="ignored">
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1.0</real_time_factor>
        </physics>
        <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands">
        </plugin>
        <plugin
            filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics">
        </plugin>

        <plugin
            filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster">
        </plugin>

        <plugin
            filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
            <render_engine>ogre2</render_engine>
        </plugin>


        <model name="ground_plane">
            <static>true</static>
            <link name="link">
                <collision name="collision">
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>100 100</size>
                        </plane>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>100 100</size>
                        </plane>
                    </geometry>
                    <material>
                        <ambient>0.8 0.8 0.8 1</ambient>
                        <diffuse>0.8 0.8 0.8 1</diffuse>
                        <specular>0.8 0.8 0.8 1</specular>
                    </material>
                </visual>
            </link>
        </model>


        <model name="simple_box">
            <pose>2 0 0.5 0 0 0</pose> <!-- Position and orientation (X Y Z roll pitch yaw) -->

            <link name="box_link">
                <pose>0 0 0 0 0 0</pose>

                <!-- Box geometry -->
                <collision name="box_collision">
                    <geometry>
                        <box>
                            <size>1 1 1</size> <!-- Size of the box (X Y Z) -->
                        </box>
                    </geometry>
                </collision>

                <visual name="box_visual">
                    <geometry>
                        <box>
                            <size>1 1 1</size> <!-- Size of the box (X Y Z) -->
                        </box>
                    </geometry>
                    <material>
                        <ambient>0.6 0.2 0.6 1.0</ambient> <!-- Color of the box -->
                        <diffuse>0.6 0.2 0.6 1.0</diffuse>
                    </material>
                </visual>
            </link>
        </model>

    </world>
</sdf>
```