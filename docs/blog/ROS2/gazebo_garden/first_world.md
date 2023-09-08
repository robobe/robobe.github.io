---
tags:
    - gazebo
    - gz
    - sdf
    - world
    - vscode tips
    - ros2
---

# World

```xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.8" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
 xsi:noNamespaceSchemaLocation="http://sdformat.org/schemas/root.xsd" >
    <world name="demo_world">
        <physics type="ignore">
            <real_time_factor>1.0</real_time_factor>
            <max_step_size>0.001</max_step_size>
        </physics>
    </world>
</sdf>
```

!!! tip "VSCode schema auto compete"
    VSCode support xml schema auto complete via [Red Hat XML](https://marketplace.visualstudio.com/items?itemName=redhat.vscode-xml)

    sdf format declare schema at uri (sdformat.org schema)[http://sdformat.org/schemas/root.xsd]
     
    config VSCode to support schema auto complete add `xsi` and `xsi:noNamespaceSchemaLocation` attribute to `sdf` root tag

    ```xml
    <sdf version="1.8" 
        xmlns:xsi="http://  www.w3.org/2001/XMLSchema-instance"
        xsi:noNamespaceSchemaLocation="http://sdformat.org/schemas/root.xsd" >
    </sdf>
    ```

---

## Run

```
GZ_SIM_RESOURCE_PATH=/home/user/gz_ws/src/gz_demos/worlds gz sim -r -v 2 world_demo.sdf
```

## ROS2

```python 

```


## gz Topic and List

### Demo: Service

```bash title="service list"
gz service --list
#
/gazebo/resource_paths/add
/gazebo/resource_paths/get
/gazebo/resource_paths/resolve
/gazebo/worlds
```

```bash title="get info about the service"
gz service -s /gazebo/worlds -i
#
gz.msgs.Empty, gz.msgs.StringMsg_V
```

```bash title="run service"
gz service -s /gazebo/worlds \
--reqtype gz.msgs.Empty \
--reptype gz.msgs.StringMsg_V \
--timeout 300 \
--req ''
#
data: "demo_world"

```

```
gz service -s /gazebo/resource_paths/get \
--reqtype gz.msgs.Empty \
--reptype gz.msgs.StringMsg_V \
--timeout 300 \
--req ''
```


```
ign topic pub /keyboard/keypress ignition.msgs.Int32 'data: 10'
ign topic pub -t /keyboard/keypress ignition.msgs.Int32 -p 100

```

```
ros2 run ros_ign_bridge \
    bridge_node \
    --ros-args \
    -p config_file:=$PWD/src/gz_demos/config/ign2ros.yaml

```