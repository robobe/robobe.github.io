---
tags:
    - py_trees
    - bt
    - behavior_trees
    - ros2
    - blackboard
---

Simple Tree with one behavior that save data from subscriber to blackboard

```python
sub_demo = py_trees_ros.subscribers.ToBlackboard(
    name="sub2bb",
    topic_name="my_topic",
    topic_type=String,
    qos_profile=qos_profile_system_default,
    blackboard_variables = {'chatter': None}
)
```

## Demo
```bash title="tree"
ros2 run py_trees_demo simple_tree_sub
```

```bash title="pub topic"
ros2 topic pub /my_topic std_msgs/msg/String "{ data: hello2 }"
```

```bash title="blackboard watcher"
py-trees-blackboard-watcher
```

![](images/subscriber_to_blackboard.png)


---

## Tree

```python title="tree" linenums="1" hl_lines="9-15 28-31 44"
import py_trees
import py_trees_ros.trees
import py_trees.console as console
import rclpy
from std_msgs.msg import String
from rclpy.qos import qos_profile_system_default

def tutorial_create_root() -> py_trees.behaviour.Behaviour:
    sub_demo = py_trees_ros.subscribers.ToBlackboard(
        name="sub2bb",
        topic_name="my_topic",
        topic_type=String,
        qos_profile=qos_profile_system_default,
        blackboard_variables = {'chatter': None}
    )
    root = py_trees.composites.Selector(name="Tasks", memory=False)

    root.add_child(sub_demo)

    return root

def main(args=None):
    """
    Entry point for the demo script.
    """
    rclpy.init(args=None)
    root = tutorial_create_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(node_name="foo", timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e: #pylint: disable=[invalid-name]
        console.logerror(console.red + f"failed to setup the tree, aborting [{str(e)}]" + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()

    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
``` 