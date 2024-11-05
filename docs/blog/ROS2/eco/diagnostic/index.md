---
tags:
    - diagnostic
    - DiagnosticTask
---
# ROS2 diagnostic

ROS diagnostics gives you the ability to monitor your system. There are two components in its architecture:

- **updater**
- **aggregator**
  
  The **updater** publishes diagnostic status messages on the `/diagnostics` topic, and the **aggregator** listens to these statuses, aggregates them, and publishes the results on the `/diagnostics_agg` topic.

A system can have multiple updaters. They are meant to communicate with devices such as motors, sensors, computers, batteries, etc, in order to retrieve and publish relevant status data. The information being published on the /diagnostics topic is flat  

The aggregator collects, categorizes, and groups the statuses from all the system components. You can take a look at what is being published on the /diagnostics and /diagnostics_agg topics with the rqt_runtime_monitor and rqt_robot_monitor tools


### DiagnosticTask
DiagnosticTask is an abstract base class for collecting diagnostic data. 

A DiagnosticTask has a name, and a function that is called to create a DiagnosticStatusWrapper. 

DiagnosticsTask subclass by

- CompositeDiagnosticTask
- FrequencyStatus
- GenericFunctionDiagnosticTask
- Heartbeat
- TimeStampStatus

### diagnostics_msg

- [DiagnosticStatus.msg](https://docs.ros2.org/foxy/api/diagnostic_msgs/msg/DiagnosticStatus.html)
- [DiagnosticArray](https://docs.ros2.org/foxy/api/diagnostic_msgs/msg/DiagnosticArray.html)

```
octet OK=0
octet WARN=1
octet ERROR=2
octet STALE=3
octet level
string name
string message
string hardware_id
diagnostic_msgs/msg/KeyValue[] values
```

### diagnostic_aggregator
Aggregator is a node that subscribes to `/diagnostics`, processes it and republishes aggregated data on `/diagnostics_agg`.

The aggregator creates a series of analyzers to handle incoming `DiagnosticStatus`

[API](https://docs.ros.org/en/humble/p/diagnostic_aggregator/generated/classdiagnostic__aggregator_1_1Aggregator.html#class-documentation)


![](images/schema.png)


### diagnostics viewers
```bash title="install"
sudo apt install ros-humble-rqt-robot-monitor
sudo apt install ros-humble-rqt-runtime-monitor
```

---

![](images/config_with_monitor_viewer.png)

---

## Demos
- [Diagnostic status function - - minimal implementation as diagnostic function](diagnostic_status_function.md)
- [DiagnosticTask class - minimal implementation as diagnostic task class](diagnostic_task_demo.md)
- [Builtin tasks (FrequencyStatus)](diagnostic_builtin_demo.md)
- [Remove task](diagnostic_remove_task_demo.md)
- [Composite task](diagnostic_composite.md)

---
# Reference
- [REP-107 Diagnostic System for Robots Running ROS](https://www.ros.org/reps/rep-0107.html)
- [ros diagnostic](https://nlamprian.me/blog/software/ros/2018/03/21/ros-diagnostics/)
- [wiki](https://wiki.ros.org/diagnostics)
- [python example](https://github.com/ros/diagnostics/blob/ros2/diagnostic_updater/diagnostic_updater/example.py)
### projects to check
- [rviz_2d_overlay_plugins](https://github.com/teamspatzenhirn/rviz_2d_overlay_plugins))

