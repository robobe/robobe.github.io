---
title: gazebo ignition hello world
tags:
    - gazebo
---

## First try
Run gazebo with empty world from install worlds

!!! note "install worlds"   
     `/usr/share/ignition/ignition-gazebo6`

```bash
ign gazebo empty.sdf -v 4
# -v 4 => verbose
```

---

## ign cli
- ign cli hello
- run custom world
  - control gui options
- spawn and delete entities from cli

### ign command
```bash
ign --help
The 'ign' command provides a command line interface to the ignition tools.

  ign <command> [options]

List of available commands :

  help:          Print this help text.
  ...
  gazebo:        Run and manage Gazebo.
  msg:           Print information about messages.
  ...
  topic:         Print information about topics.
  service:       Print information about services.
  ...
```

### ign gazebo command
```bash
ign gazebo --help

  -r                           Run simulation on start.                         

  -v [ --verbose ] [arg]       Adjust the level of console output (0~4).        

  --gui-config [arg]           Ignition GUI configuration file to load.         
                               
```

### Environment variables

| Variable  | Description  |
|---|---|
| IGN_GAZEBO_RESOURCE_PATH  | Colon separated paths used to locate resources such as worlds and models.  |
| IGN_GAZEBO_SYSTEM_PLUGIN_PATH | Colon separated paths used to locate system plugins.                |
| for more check | ign gazebo --help |

---

## Custom world.sdf 
- load minimal world
- control gui layout
- spawn / delete model from cli


!!! note "Set ignition resource path"
    ```bash
    export IGN_GAZEBO_RESOURCE_PATH=/home/user/projects/ign_tutorial/
    ```
     
```xml title="my_empty.sdf"
<sdf version="1.6">
    <world name="empty">
      <physics name="1ms" type="ignored">
        <max_step_size>0.001</max_step_size>
        <real_time_factor>1.0</real_time_factor>
      </physics>
      <gui>
        <plugin filename="GzScene3D" name="3D View">
            <ignition-gui>
            <title>3D View</title>
            <property type="bool" key="showTitleBar">false</property>
            <property type="string" key="state">docked</property>
            </ignition-gui>

            <engine>ogre2</engine>
            <scene>scene</scene>
            <ambient_light>0.4 0.4 0.4</ambient_light>
            <background_color>0.8 0.8 0.8</background_color>
        </plugin>
        
      </gui>
      <plugin
        filename="ignition-gazebo-physics-system"
        name="ignition::gazebo::systems::Physics">
      </plugin>
      <plugin
        filename="ignition-gazebo-user-commands-system"
        name="ignition::gazebo::systems::UserCommands">
      </plugin>
      <plugin
        filename="ignition-gazebo-scene-broadcaster-system"
        name="ignition::gazebo::systems::SceneBroadcaster">
      </plugin>
      <plugin
        filename="ignition-gazebo-contact-system"
        name="ignition::gazebo::systems::Contact">
      </plugin>
  
      <light type="directional" name="sun">
        <cast_shadows>true</cast_shadows>
        <pose>0 0 10 0 0 0</pose>
        <diffuse>0.8 0.8 0.8 1</diffuse>
        <specular>0.2 0.2 0.2 1</specular>
        <attenuation>
          <range>1000</range>
          <constant>0.9</constant>
          <linear>0.01</linear>
          <quadratic>0.001</quadratic>
        </attenuation>
        <direction>-0.5 0.1 -0.9</direction>
      </light>
  
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
  
    </world>
  </sdf>
```

- Everything control by plugin

---

# Spawn and remove models

### user-commands-system
Allow user control from cli

```xml
<plugin
    filename="ignition-gazebo-user-commands-system"
    name="ignition::gazebo::systems::UserCommands">
</plugin>
```

### ign service / msg command
- 
```bash
# list services
ign service --list
...
/world/empty/create
...
/world/empty/remove
...


# more info about the service
ign service --info --service /world/empty/remove
# result
Service providers [Address, Request Message Type, Response Message Type]:
  tcp://172.18.0.1:45365, ignition.msgs.Entity, ignition.msgs.Boolean

# Service has two type of message
# request: ignition.msgs.Entity
# response: ignition.msgs.Boolean

# message fields
## serach for boolean msg
ign msg --list | grep Boolean
ign_msgs.Boolean

# show msg fields with --info argument
user@lap2:~/projects/blog$ ign msg --info ign_msgs.Boolean
Name: ignition.msgs.Boolean
File: ignition/msgs/boolean.proto

message Boolean {
  .ignition.msgs.Header header = 1;
  bool data = 2;
}
```

### Spawn SDF
using ign service to Spawn and remove entities

```title="from SDF file"
ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 300 --req 'sdf_filename: "models/my_ball/model.sdf" pose: {position: {z: 5}} name: "new_name"'
```

```bash title="from SDF string"

```

### Remove

```
ign service -s /world/empty/remove \
--reqtype ignition.msgs.Entity \
--reptype ignition.msgs.Boolean \
--timeout 300 \
--req 'name: "new_name" type: MODEL'
```

```xml title="models/my_ball/model.sdf"
<sdf version="1.6">
    <model name="spawned_model">
        <link name="link">
            <visual name="visual">
                <geometry>
                    <sphere>
                        <radius>2.0</radius>
                    </sphere>
                </geometry>
            </visual>
            <collision name="visual">
                <geometry>
                    <sphere>
                        <radius>2.0</radius>
                    </sphere>
                </geometry>
            </collision>
        </link>
    </model>
</sdf>
```

---