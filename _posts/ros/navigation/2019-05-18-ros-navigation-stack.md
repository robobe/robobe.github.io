---
layout: post
title: ROS Navigation stack , Hello
categories: ros
tags: [navigation stack, gmapping]
description: Post base on Construct live class "navigation stack", SLAM Tutorial show gmapping and AMCL algorithm for mapping and localization
image: route.png
public: true
---
# Content
- [Navigation stack](#navigation-stack)
- [Setup environment](#setup-environment)
- [gmapping](#gmapping)
- [Localization](#localization)
- [Robot autonomous](#robot-autonomous)
  
  
  
&nbsp;  
&nbsp;  
&nbsp;  
# Navigation stack
The goal of the navigation stack is to move a robot from one position to another position safely (without crashing or getting lost)  

It takes in information from the odometryand sensors, and a goal pose and outputs safe velocity commands that are sent to the robo

![](/images/navigation_stack.png)

&nbsp;  
&nbsp;  
&nbsp;  
# Setup environment
- Compile gmapping from source for ROS melodic check [post](../ros-melodic-gmapping)
- Install AMCL
```
sudo apt install ros-melodic-amcl
```
- Install dwa_local_planner
```
sudo apt install ros-melodic-dwa-local-planne
```
- Using `jackal_race.world` from jackal_gazebo package
- 
## launch world and spawn robot

- nav_world.launch
  - Launch gazebo with jackal race world
  - Set `robot_description`
  - Run joint_State and robot_State from `robot_state_publisher` package
  - Spawn the robot


```xml
<launch>
  <arg name="paused" default="false"/>
  <arg name="world_name" default="$(find jackal_gazebo)/worlds/jackal_race.world"/>
  
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(arg world_name)" />
    <arg name="verbose" value="true"/>
    <arg name="paused" value="$(arg paused)"/>
  </include>

  <!-- urdf xml robot description loaded on the Parameter Server-->
  <param name="robot_description" 
  command="$(find xacro)/xacro '$(find kbot_description)/urdf/diff_wheeled_robot.xacro'" /> 

  	<param name="use_gui" value="false"/>
	<!-- Starting Joint state publisher node which will publish the joint values -->
	<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />

	<!-- Starting robot state publish which will publish tf -->
<node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />
  

  <!-- Run a python script to the send a service call to gazebo_ros to spawn a URDF robot -->
  <node name="urdf_spawner" 
  pkg="gazebo_ros" 
  type="spawn_model" 
  respawn="false" output="screen"
  args="-urdf -model diff_wheeled_robot -param robot_description"/> 
</launch>

```

&nbsp;  
&nbsp;  
&nbsp;  
# gmapping
- Set scan_topic (LaserScan topic)
- odom_frame: Odom topic name (publish by the robot gazebo ros plugin)
- base_frame: Robot base frame
- map_frame: publish map topic
- maxRange: Sensor maximum range
- maxURange: Usage range to build the map from sensor reading

{% gist 980b85ec4fd9c83da76eadf29134aa24 %}

## Running
- Terminal 1
  - gazebo world and robot
- Terminal 2
  - [teleop](#teleop)
- Termianl 3
  - gmapping
- Termianl 4
  - run rviz
- Terminal 5 (save map)
  - from map directory
  - `rosrun map_server map_saver -f my_map`

## Rviz
- Add `LaserScan`
  - topic: /diff/scan
  - size (laser marker): 0.1 (display laser more clearly)
- Add `Map`
  - topic: `/map` (published by `/slam_gmapping` node)

![gmapping Gazebo and rviz](/images/2019-05-18-23-54-53.png)


&nbsp;  
&nbsp;  
&nbsp;  


# Localization
- Provide a map (created by gmapping)
  - `rosrun map_server map_server my_map.yaml`

{% gist 5603c818b12321c43c32cc71480b7978 %}

## running
- Terminal1 (gazebo)
```bash
roslaunch kbot_navigation nav_world.launch
```

- Terminal2 (map provider)
  > From map location
```bash
rosrun map_server map_server my_map.yaml
```

- Terminal3 (amcl)
```bash
roslaunch kbot_navigation amcl.launch
```

- Terminal4 (rviz)
```
rosrun rviz rviz
```

- Terminal5 (teleop)
```
roslaunch kbot_navigation teleop.launch
```

## rviz config and usage

- Set fixed frame to `map`
- Add `map` set topic to `/map` (provide by amcl)

![rviz display map](/images/2019-05-19-10-36-11.png)

- Add `PoseArray` subscribe to a topic `/particlecloud`
```bash
rostopic info /particlecloud 
Type: geometry_msgs/PoseArray

Publishers: 
 * /amcl (http://dev:37263/)

Subscribers: 
 * /rviz_1558251268155732663 (http://dev:39477/)
```
- Init the robot location using `2D  pose estimation`
![rviz using 2D pose estimation](/images/2019-05-19-10-51-43.png)

## teleop
- When the robot move the algorithm estimate better the robot position
- The pose is good when the laser scan align with the map
  
![rviz particle cloud](/images/2019-05-19-11-00-03.png)


# Robot autonomous
## move_base
The move_base package use to move a robot to desired positions using the navigation stack  
The robot will move through the map using two types of navigation
- **Global navigation**: Create path for a goal target in the map
- **Local navigation**: Create path in the nearby distance and avoid obstacles

## costmap
A data structure that represents places that are safe for the robot to be in a grid of cells (map)
- **Global costmap**
- **Local costmap**

Each cell in the costmaphas an integer value in the range [0 (FREE_SPACE), 255 (UNKNOWN)]

Configuration of the costmaps consists of three files: 
- [costmap_common_params.yaml](#costmapcommonparamsyaml)
  - Set `robot_radius` param
  - Check `laser_scan_sensor` for correct `topic` attribute
  - Check `map_topic` param
  - Check `global_frame` param
  - Check `robot_base_frame` param
- [global_costmap_params.yaml](#global-costmapparamsyaml)
  - Check `global_frame` param
  - Check `robot_base_frame` param
- [local_costmap_params.yaml](#localcostmapparamsyaml)
  - Check `global_frame` param
  - Check `robot_base_frame` param

The costmap performs map update cycles, in each cycle:
- Read sensor data
- marking and clearing operations are performed in the underlying occupancy structure of the costmap

### rviz 
#### costmap
- Add `map` with Topic `/move_base_node/global_costmap/costmap`
- Add `map` with Topic `/move_base_node/local_costmap/costmap`

#### Navigation Plans
- NavFn topic `/move_base/NavfnROS/plan`
- Global Plan topic `/move_base/DWAPlannerROS/global_plan`
- Local Plan topic `/move_base/DWAPlannerROS/local_plan`

## Running
- Terminal1 (gazebo)
```bash
roslaunch kbot_navigation nav_world.launch
```

- Terminal2 (map provider)
  > From map location
```bash
rosrun map_server map_server my_map.yaml
```

- Terminal3 (amcl)
```bash
roslaunch kbot_navigation amcl.launch
```

- Terminal4 (move_base)
```
roslaunch kbot_navigation move_base.launch
```

- Terminal5 (rviz)
```
rosrun rviz rviz
```

## Move base launch and config files
#### move_base launch file
{% gist 4c43bbbb3fea70313c98c0e961b1f972 %}

#### move_base_params.yaml
{% gist e3e1240dd682e63a154e49dcc9f4e9b2 %}

### costmap_common_params.yaml
{% gist a24f917fcb6049bb0f53dc5fe457bc17 %}

### global costmap_params.yaml
{% gist 79f051ec68c7afb09e36594ee183bd80 %}

### local_costmap_params.yaml
{% gist b3dc029e7a15efe586d687a6cd94584d %}

### dwa_local_planner_params.yaml
{% gist 9ac349a1753f88dd291ff45bf80b72c9 %}

### base_local_planner_params.yaml
{% gist af48d25280e1d0a05e4025f85c90df16 %}

&nbsp;  
&nbsp;  
&nbsp;  


## teleop
```xml
<?xml version="1.0"?>
<launch>
  <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="teleop_twist_keyboard"  output="screen">
    <param name="scale_linear" value="1.0" type="double"/>
    <param name="scale_angular" value="1.5" type="double"/>
  </node>
</launch>
```

# Reference
- [ROS Developers LIVE-Class #13: ROS Navigation Stack How To](https://www.youtube.com/watch?v=fTizQneURWo&t=3675s)
- [Bar-Ilan ROS Lesson 7](https://u.cs.biu.ac.il/~yehoshr1/89-685/Fall2015/ROS_Lesson7.pdf)
- [
A ROS move_base local planner plugin for Car-Like robots with Ackermann or 4-Wheel-Steering.
](https://github.com/gkouros/rsband_local_planner)