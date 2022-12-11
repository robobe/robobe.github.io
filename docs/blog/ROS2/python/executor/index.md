---
title: ROS2 rclpy executor
tags:
    - ros2
    - python
    - executor
---

By default, rclpy offers two different executors for the user to choose from:

- SingleThreadedExecutor (default)
- MultiThreadedExecutor

**SingleThreadedExecutor**:  executes callbacks in a single thread, one at a time, and thus the previous callback must always finish before a new one can begin execution.

**MultiThreadedExecutor**: executing several callbacks simultaneously.

### Callback groups

- MutuallyExclusiveCallbackGroup
- ReentrantCallbackGroup

**MutuallyExclusiveCallbackGroup**: allows the executor to execute only one of its callbacks simultaneously (TODO)

**ReentrantCallbackGroup**: allows the executor to schedule and execute the group’s callbacks in any way the executor sees fit, (TODO)



callback examples
- subscription callback
- timer callback
- service callback (request on server)
- action server and client callback
- Future done callback

---

# Reference
- [Deadlocks in rclpy and how to prevent them with use of callback groups](https://karelics.fi/deadlocks-in-rclpy/)
- [Executors](https://docs.ros.org/en/humble/Concepts/About-Executors.html)