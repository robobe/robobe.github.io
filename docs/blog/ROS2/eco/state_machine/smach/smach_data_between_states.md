---
tags:
    - smach
    - state machine
    - ros2
---

# Smach passing data between states

## Demo

```python title="state_machine_ud.py"
import rclpy
from rclpy.node import Node
import smach
import smach_ros
import time


log = rclpy.logging.get_logger(__name__)

class Foo(smach.State):
    def __init__(self):
        super().__init__(
            outcomes=["outcome1", "outcome2"],
            input_keys=["foo_counter_in"],
            output_keys=["foo_counter_out"]    
        )
        self.counter = 0

    def execute(self, ud):
        time.sleep(1)       # todo: how to sleep in ros
        if self.counter < 10:
            ud.foo_counter_out = ud.foo_counter_in+1
            self.counter += 1
            return "outcome1"
        else:
            return "outcome2"
        
class Bar(smach.State):
    def __init__(self):
        super().__init__(
            outcomes=["outcome2"],
            input_keys=["bar_counter_in"]
        )

    def execute(self, ud):
        log.info(f"---------{ud.bar_counter_in}-----------")        
        time.sleep(1)
        return "outcome2"
    
class MyNode(Node):
    def __init__(self):
        node_name="smach_sm"
        super().__init__(node_name)
        self.get_logger().info("Hello ROS2 smach")

        self.sm = smach.StateMachine(outcomes=["outcome4", "outcome5"])
        self.sm.userdata.sm_counter = 0

        with self.sm:
            smach.StateMachine.add("FOO", Foo(), 
                                   transitions={"outcome1": "BAR", "outcome2": "outcome4"},
                                   remapping={
                                       "foo_counter_in": "sm_counter",
                                       "foo_counter_out": "sm_counter"
                                   }
                                )
            smach.StateMachine.add("BAR", Bar(), 
                                   transitions={"outcome2": "FOO" },
                                    remapping={
                                        "bar_counter_in": "sm_counter"
                                    }   
                                )

        
        self.sis = smach_ros.IntrospectionServer("my_smach_introspection_server", self.sm, "/SM_ROOT")
        self.sis.start()
        outcome = self.sm.execute()

    def close(self):
        self.sis.stop()

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    
    try:
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("User exit")
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
```

### Run

![](images/state_machine_with_userdata.png)

---

## Reference
- [](https://blog.csdn.net/weixin_43455581/article/details/97136945)