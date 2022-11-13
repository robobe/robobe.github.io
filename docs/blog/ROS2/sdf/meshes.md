---
title: Load meshes
tags:
    - sdf
---

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

!!! warning 
    SDF uses file instead of package

!!! warning 
    RVIZ not support `model` uri
     
