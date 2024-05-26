---
tags:
    - tf2
    - ros
    - static
---

# tf2 static broadcaster
Publish one time on /tf_static

!!! note "scipy"
    Package use `Rotation` from scipy package
     

```python title="static broadcaster"
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation as R



class StaticFramePublisher(Node):
   """
   Broadcast transforms that never change.

   This example publishes transforms from `world` to a static turtle frame.
   The transforms are only published once at startup, and are constant for all
   time.
   """

   def __init__(self, transformation):
      super().__init__('static_tf2_broadcaster')

      self._tf_publisher = StaticTransformBroadcaster(self)

      # Publish static transforms once at startup
      self.make_transforms(transformation)

   def make_transforms(self, transformation):
      static_transformStamped = TransformStamped()
      static_transformStamped.header.stamp = self.get_clock().now().to_msg()
      static_transformStamped.header.frame_id = 'world'
      static_transformStamped.child_frame_id = transformation[1]
      static_transformStamped.transform.translation.x = float(transformation[2])
      static_transformStamped.transform.translation.y = float(transformation[3])
      static_transformStamped.transform.translation.z = float(transformation[4])
      quat = R.from_euler("xyz", [
         float(transformation[5]), 
         float(transformation[6]), 
         float(transformation[7])]).as_quat()
      static_transformStamped.transform.rotation.x = quat[0]
      static_transformStamped.transform.rotation.y = quat[1]
      static_transformStamped.transform.rotation.z = quat[2]
      static_transformStamped.transform.rotation.w = quat[3]

      self._tf_publisher.sendTransform(static_transformStamped)


def main():
   logger = rclpy.logging.get_logger('logger')

   # obtain parameters from command line arguments
   if len(sys.argv) < 8:
      logger.info('Invalid number of parameters. Usage: \n'
                  '$ ros2 run learning_tf2_py static_turtle_tf2_broadcaster'
                  'child_frame_name x y z roll pitch yaw')
      sys.exit(0)
   else:
      if sys.argv[1] == 'world':
            logger.info('Your static turtle name cannot be "world"')
            sys.exit(0)

   # pass parameters and initialize node
   rclpy.init()
   node = StaticFramePublisher(sys.argv)
   try:
      rclpy.spin(node)
   except KeyboardInterrupt:
      pass

   rclpy.shutdown()
```

```bash title="usage"
ros2 run tf_demo static_broadcaster r1 1 0 0 0 0 0
```

```bash title="echo tf_static"
ros2 topic echo /tf_static 
#
transforms:
- header:
    stamp:
      sec: 1716139459
      nanosec: 394842009
    frame_id: world
  child_frame_id: r1
  transform:
    translation:
      x: 1.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
---

```