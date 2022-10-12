---
title: JointStatePublisher
tags:
    - ignition
    - ros2
    - bridge
---

## JointStatePublisher
The JointStatePub system publishes state information for a model. The published message type is ignition::msgs::Model, and the publication topic is `/world/<world_name>/model/<model_name>/state`.

```xml
<plugin
    filename="ignition-gazebo-joint-state-publisher-system"
    name="ignition::gazebo::systems::JointStatePublisher">
</plugin>
```

# Reference
- [double_pendulum_model](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim_demos/models/double_pendulum_model.sdf)
- [launch demo](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim_demos/launch/tf_bridge.launch.py)
- [ros_gz_sim_demos](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim_demos)
- [JointStatePublisher Class Reference](https://gazebosim.org/api/gazebo/4.3/classignition_1_1gazebo_1_1systems_1_1JointStatePublisher.html)
- [joint state publisher and robot state publisher](https://answers.ros.org/question/275079/joint-state-publisher-and-robot-state-publisher/)
- [RVIZ2 Tutorials Episode1: Learn TF](https://www.theconstructsim.com/rviz2-tutorials-episode1-learn-tf/)
