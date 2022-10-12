---
title: Part5 - ROS Web interface with vue
description: ROS2 Web interface using roslibjs and vue3
date: "2022-04-19"
banner: ../images/ros_bridge.png
tags:
    - ros2
    - vue
    - rosbridge
---
# Bridge
```bash title="run node"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

# Web
- index.html: load libraries and HTML template
- app.js: Create vue app and init roslib connection


```html title="index.html" linenums="1" hl_lines="5 6 16"
<html>

<head>
  <meta charset="utf-8" />
  <script src="http://static.robotwebtools.org/roslibjs/current/roslib.min.js"></script>
  <script src="https://unpkg.com/vue@3.0.2"></script>
</head>

<body>
  <h1>Simple ROS User Interface</h1>
  <div id="app">
    <p>ros status: {{status}}</p>

  </div>

  <script src="app.js"></script>

</body>

</html>
```

```js title="app.js" linenums="1" hl_lines="14"
const app = Vue.createApp({
  data() {
    return {
      status: "---",
      param: 0
    }
  },
  mounted() {
    this.ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });
  },
})
vm = app.mount('#app')

vm.ros.on('connection', () => {
    vm.status = "--connected--";
});
```

---

## Parameter
```js title="declared"
var my_param = new ROSLIB.Param({
  ros: vm.ros,
  name: '/node:param_name'
});
```

```js title="get / set"
//Get
my_param.get(function (value) {
  if (value != null) {
    console.log(value);
  }
});

//Set
my_param.set(5)
```