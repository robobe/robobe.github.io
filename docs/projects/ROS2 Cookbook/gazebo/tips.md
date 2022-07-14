---
title: Gazebo ROS tips
tags:
    - tips
---

# Gazebo environment

- GAZEBO_MODEL_PATH: Gazebo will search for models
- GAZEBO_RESOURCE_PATH: Gazebo search for other resources such as **world** and **media** files.
- GAZEBO_PLUGIN_PATH: Gazebo search for the plugin shared libraries at runtime


```xml
<exec_depend>gazebo_ros</exec_depend>
  
<export>                            
    <gazebo_ros
    gazebo_media_path="${prefix}:/another/path"
    gazebo_plugin_path="${prefix}/lib"    
    gazebo_model_path="${prefix}/../your_models"           
    gazebo_resource_path="${prefix}:/path/to/your/resources"/>                    
</export>
```

!!! note "prefix"
    ```bash
    # prefix       Output the prefix path of a package
    ros2 pkg prefix diffbot_description
    /home/user/dev_ws/install/diffbot_description
    # --share Show share directory for the package
    ros2 pkg prefix --share diffbot_description
    /home/user/dev_ws/install/diffbot_description/share/diffbot_description
    ```
     

## Example

- Load mesh from sdf model


```
├── models
│   └── warehouse
│       ├── meshes
│       │   └── warehouse.stl
│       ├── model.config
│       └── model.sdf
```

```xml title="sdf visual"
<visual name="visual">
    <geometry>
        <mesh>
        <uri>model:///warehouse/meshes/warehouse.stl</uri>
        <scale>0.001 0.001 0.001</scale>
        </mesh>
    </geometry>
</visual>
```

```xml title="package.xml"
<export>
    <build_type>ament_cmake</build_type>
    <gazebo_ros gazebo_model_path="${prefix}/models"/>
    <gazebo_ros gazebo_media_path="${prefix}/worlds"/>
</export>
```

!!! warning
    The only combination that success loading mesh under module folder
    is to set 
    - uri: <model path>/meshes/mesh file
    - package.xml <gazebo_ros gazebo_model_path="${prefix}/models"/>

     