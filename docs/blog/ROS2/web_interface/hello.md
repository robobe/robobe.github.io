---
title: Part1 - Web interfcae
description: ROS2 Web interface
date: "2022-04-19"
banner: ../images/ros_bridge.png
tags:
    - ros2
    - web
    - rosbridge
---

## Rosbridge
Rosbridge provides a JSON API to ROS functionality for non-ROS programs

[rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)



```bash title="install"
sudo apt-get install ros-foxy-rosbridge-suite
```

### Run
Open websocket on port 9090 as default

```bash title="run bridge"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

---

## roslibjs
roslibjs is the core JavaScript library for interacting with ROS from the browser.  
It uses WebSockets to connect with rosbridge and provides publishing, subscribing, service calls, actionlib, TF, URDF parsing, and other essential ROS functionality

### Run
Demo web page using `roslibjs` library

```js title="basic connection"
<html>
<head>
  <meta charset="utf-8" />

  <script type="text/javascript" src="http://static.robotwebtools.org/roslibjs/current/roslib.min.js"></script>

  <script type="text/javascript" type="text/javascript">
    var ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });

    ros.on('connection', function () {
      document.getElementById("status").innerHTML = "Connected";
    });

    ros.on('error', function (error) {
      document.getElementById("status").innerHTML = "Error";
    });

    ros.on('close', function () {
      document.getElementById("status").innerHTML = "Closed";
    });
  </script>
</head>

<body>
  <h1>Simple ROS User Interface</h1>
  <p>Connection status: <span id="status"></span></p>
</body>
</html>
```



---

---

# Reference
- [roslibjs](http://wiki.ros.org/roslibjs)
- [Robot Web Tools](http://robotwebtools.org/)
- [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)
- [ROS web tutorial part 1 - rosbridge server and roslibjs](https://msadowski.github.io/ros-web-tutorial-pt1/)