---
title: Gazebo resources path
tags:
    - gazebo
    - sdf
---

## uri

```xml
<visual name='base_link_visual'>
    <pose>0 0 0 0 0 0</pose>
    <geometry>
      <mesh>
        <scale>1 1 1</scale>
        <uri>file://path_to_dae/textured.dae</uri>
      </mesh>
    </geometry>
  </visual>
```

```xml
<visual name='base_link_visual'>
    <pose>0 0 0 0 0 0</pose>
    <geometry>
      <mesh>
        <scale>1 1 1</scale>
        <uri>model://model_name/meshes/textured.dae</uri>
      </mesh>
    </geometry>
  </visual>
```

## package.xml

```xml
<export>
    <build_type>ament_cmake</build_type>
    <gazebo_ros gazebo_model_path="${prefix}/models"/>
    <gazebo_ros gazebo_media_path="${prefix}"/>
  </export>
```
!!! note "prefix"
    `${prefix}` is replaced by package's share directory in install

!!! warning "depend"
    Using the above tags depend in `gazebo_ros`
    add `exec_depend` to package.xml
     