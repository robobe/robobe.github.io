---
tags:
    - rrbot
    - JointTrajectory
    - gazebo_ros_joint_pose_trajectory
---

# Send joint commands using JointTrajectory

Using RRBot to demonstration how to send **JointTrajectory** message from cli or code

!!! 
    From it's name gazebo_ros_joint_**pose**_trajectory
    The plugin support only **position** command 

## Gazebo
Add `gazebo_ros_joint_pose_trajectory` plugin to urdf

```xml
<gazebo>
    <plugin name="gazebo_ros_joint_pose_trajectory"
        filename="libgazebo_ros_joint_pose_trajectory.so">
        <update_rate>2</update_rate>
    </plugin>
</gazebo>
```

## cli

```
ros2 topic pub -1 /set_joint_trajectory trajectory_msgs/msg/JointTrajectory  '{header: {frame_id:
world}, joint_names: [joint1, joint2], points: [  {positions: {1,1}} ]}'
```


## code

Add python node and set entry point called `trajectory`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.qos import qos_profile_system_default


class MyNode(Node):
    def __init__(self):
        node_name="minimal"
        super().__init__(node_name)
        self.pub = self.create_publisher(JointTrajectory,
                                         "/set_joint_trajectory",
                                         qos_profile=qos_profile_system_default
                                         )
        
        self.send_position()
        self.get_logger().info("Hello ROS2")

    def send_position(self):
        msg = JointTrajectory()
        header = Header()
        header.frame_id = "world"
        header.stamp = self.get_clock().now().to_msg()
        msg.header = header
        msg.joint_names = ["joint1", "joint2"]
        
        points = JointTrajectoryPoint()
        points.positions = [1.0, 1.0]
        points.velocities = []
        points.accelerations = []
        points.effort = []
        points.time_from_start = Duration(sec=10)

        msg.points = [points]
        self.pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

##### running

```bash
ros2 run rrbot_application trajectory --ros-args -p use_sim_time:=True
```

!!! warning use_sim_time:=True
    If we set the time stamp in message header we must sync between ROS and gazebo time
    using `use_sim_time` parameter
     

---

## Reference
[plugin code](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_joint_pose_trajectory.cpp)