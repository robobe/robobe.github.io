---
title: ROS2 Python 
tags:
    - ros2
    - python
    - setup.py
---

## Minimal Node
Minimal python node Template

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        node_name="minimal"
        super().__init__(node_name)
        self.get_logger().info("Hello ROS2")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("User exit")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
```

---

## Setup tips

- Copy none python files to `share` folders
- Map entry points to python modules



### copy none python files

```python
data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]

def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=package_files(data_files, [
        'models/',
        'launch/',
        'worlds/',
        "config",
        "urdf"
    ]),
)
```


### Entry points
```python title="map entry points"
# entry points
entry_points={
        'console_scripts': [
            "minimal_node=py_tutorial_pkg.minimal_node:main",
        ]
```

---

## Tutorials
- [pub sub namespace and remapping](pub_sub_ns_remapping.md)


