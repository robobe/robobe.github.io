---
title: ROS2 gazebo camera sensor and plugin
git_revision_date_localized_raw_iso_date: 06-01-23
tags:
    - ros2
    - gazebo
    - camera
---
Gazebo camera sensor with ROS2 plugin Tutorial
Using SDF, 

- Spawn camera SDF model into gazebo
- View Camera with correct TF in rviz
  - set plugin camera coordinate system (x:right, y:down, z:into the plan)



![](images/camera_gazebo_rviz.png)

---
### Image Coordinate Frame
![](images/imae_coordinate_frame.png)
### Camera Coordinate Frame

![](images/camera_coordinate_frame.png)

### Robot Coordinate Frmae
![](images/robot_coordinate_frame.png)

---

### Demo 

- [camera.world](#world): Gazebo world sdf file 
- [camera2.sdf](#model): camera model, SDF file with ros2 plugin
- [camera.launch.py](#launch): ROS2 launch file 
    - Launch gazebo
    - Run Rviz
    - spawn ROBOT (camera)
    - Set static TF's

#### world
Basic gazebo world 
- Add simple object viewed by the camera

```xml title="worlds/camera.world"
--8<-- "/home/user/ros2_ws/src/ros2_gazebo_tutorial/worlds/camera.world"
```

#### model
```xml title="models/camera2/model.sdf" linenums="1" hl_lines="5 30 46"
--8<-- "/home/user/ros2_ws/src/ros2_gazebo_tutorial/models/camera2/model.sdf"
```

#### launch
```python title="launch/camera.launch.py" linenums="1" hl_lines="70 78"
--8<-- "/home/user/ros2_ws/src/ros2_gazebo_tutorial/launch/camera.launch.py"
```


---

# Reference
- [Projec@on](https://www.cs.cornell.edu/courses/cs4670/2015sp/lectures/lec15_projection_web.pdf)
- [image copywrite](https://docs.nvidia.com/isaac/archive/2020.1nx/packages/perception/doc/coord_frame.html)