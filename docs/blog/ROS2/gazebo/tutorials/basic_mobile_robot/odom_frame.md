---
title: Odom frame
tags:
    - gazebo classic
    - tutorial
    - odom
---


## diff drive
using odom tf from diff drive plugin

- change `publish_odom_tf` from false to true
  
```xml
<publish_odom_tf>true</publish_odom_tf>
```

![](images/gazebo_rviz_odom_frame.png)