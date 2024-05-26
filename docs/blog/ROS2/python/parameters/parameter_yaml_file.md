---
tags:
    - ros2
    - parameter
    - param
    - python
    - yaml
---

# Parameter file
Demo base on previous created package


```python title="minimal node"
import rclpy
from rclpy.node import Node
import sys

class BasicParams(Node):
    def __init__(self):
        super().__init__('basic_param')
        self.declare_parameter('my_str', value="string data")
        self.declare_parameter('my_int', value=10)
        
        self.my_str = self.get_parameter("my_str").value
        self.my_int = self.get_parameter("my_int").value

        self.get_logger().info(f"my_str: {self.my_str}")
        self.get_logger().info(f"my_int: {self.my_int}")

def main():
    rclpy.init(args=sys.argv)
    node = BasicParams()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

```bash title="usage"
ros2 run simples param_basic
[INFO] [1716311923.922650805] [basic_param]: my_str: string data
[INFO] [1716311923.922939105] [basic_param]: my_int: 10
```

```yaml title="param_basic.yaml"
/basic_param:
  ros__parameters:
    my_int: 30
    my_str: new string data
```

```bash title="usage with param file"
ros2 run simples param_basic --ros-args --params-file src/simples/config/param_basic.yaml
[INFO] [1716312022.420769922] [basic_param]: my_str: new string data
[INFO] [1716312022.421097070] [basic_param]: my_int: 30
```

!!! tip "support namespace"
    ```
    ros2 run simples param_basic --ros-args \
    -r __ns:=/demo \
    --params-file src/simples/config/param_basic.yaml
    ```

    When we add namespace to the node there no match to the param file node name
    Add `/*/` or `/**/` to support wildcard matching

    ```yaml
    /**/basic_param:
    ros__parameters:
        my_int: 30
        my_str: new string data
    ```

## Dump

### Using cli

```bash
ros2 param set /basic_param my_int 10
```

```bash
ros2 param dump /basic_param
 
/basic_param:
  ros__parameters:
    my_int: 10
    my_str: new string data
```

---

### Idea: Save param file from node

- Add Empty service to save/dump node params as yaml file to robot location


```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import yaml
from pathlib import Path
import sys

class BasicParams(Node):
    def __init__(self):
        super().__init__('basic_param')
        self.declare_parameter('my_str', value="string data")
        self.declare_parameter('my_int', value=10)
        full_name = f"{self.get_namespace()}/{self.get_name()}/save_parameters".replace("//", "/")
        self.create_service(Empty, full_name, self.server_handler)
        self.my_str = self.get_parameter("my_str").value
        self.my_int = self.get_parameter("my_int").value

        self.get_logger().info(f"my_str: {self.my_str}")
        self.get_logger().info(f"my_int: {self.my_int}")

        # self.add_on_set_parameters_callback(self.handler)

    def server_handler(self, req, resp):
        data = {k:v.value for k,v in self._parameters.items()}
        yaml_output = {"/**/" + self.get_name(): {'ros__parameters': data}}
        full_path = Path("/tmp").joinpath(self.get_name()+".yaml").as_posix()
        with open(full_path, "w", encoding="utf-8") as f:
            yaml.dump(yaml_output, 
                      stream=f,
                      default_flow_style=False)
            
            
            
        return resp
```

```bash title="usage"
ros2 service call /basic_param/save_parameters std_srvs/srv/Empty {}
```

todo: read param_file location from `sys.argv`
