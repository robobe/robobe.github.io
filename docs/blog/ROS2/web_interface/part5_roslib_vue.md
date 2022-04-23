---
title: Part5 - ROS Web interface with vue
description: ROS2 Web interface
date: "2022-04-19"
banner: ../images/ros_bridge.png
tags:
    - ros2
    - vue
    - rosbridge
---

```html title="index.jtml" linenums="1" hl_lines="1"
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

```js title="app.js" linenums="1" hl_lines="1"
const app = Vue.createApp({
    data() { 
            return {
                status: "---"
            }
    },
    mounted() {
        this.ros = new ROSLIB.Ros({
          url : 'ws://localhost:9090'
        });
    
        this.ros.on('connection', () => {
          this.status = "--connected--";
        });
      }
})
app.mount('#app')
```