---
title: Ignition ROS2
tags:
    - ros_ign
---

## ros ign bridge

ros_ign contains packages that provide integration between ROS2 and Ignition:

- ros_ign: Metapackage that provides all other software packages;
- ros_ign_image: Use image_transport to transfer the image from Ignition to the one-way transmission bridge of ROS;
- ros_ign_bridge: Two-way transmission bridge between Ignition and ROS;
- ros_ign_gazebo: It is convenient to use the startup files and executable files of Ignition Gazebo and ROS;
- ros_ign_gazebo_demos: Demos using ROS-Ignition integration;
- ros_ign_point_cloud: A plug-in used to simulate publishing point clouds to ROS from Ignition Gazebo

### install
```
sudo apt install ros-humble-ros-gz
```

---

### usage

- launch file that run node for each mapping
- The launch file included by parent launch file that run ignition and spawn the robot

```bash title="project" linenums="1" hl_lines="5-7"
├── CMakeLists.txt
├── config
│   ├── ekf.yaml
│   └── nav2_params.yaml
├── launch
│   ├── display.launch.py
│   └── sam_bridge.launch.py
├── package.xml
├── README.md
├── rviz
│   └── urdf_config.rviz
├── src
│   └── description
│       └── sam_bot_description.urdf
└── world
    ├── ign_world.sdf
    └── my_world.sdf

```

```python title="gazebo, spawn, bridge"
# Gazebo Sim
gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={'gz_args': f'-r {world_path}'}.items(),
)

#spawn
spawn_entity = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-name', 'sam_bot',
        '-topic', 'robot_description',
        '-z', '0.5'
    ],
    output='screen',
)

#bridge
ign_bridge = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_share, 'launch', BRIDGE_FILE_NAME),
    ),
    launch_arguments={
        'use_sim_time': "True"}.items()
)
```

---

# Reference
- [ROS + Gazebo Sim demos](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim_demos)