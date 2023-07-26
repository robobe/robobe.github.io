---
tags:
    - rosbags
    - tools
    - ros
---

# Rosbags

Rosbags is the pure python library for everything rosbag. It contains:

- highlevel easy-to-use interfaces,
- rosbag2 reader and writer,
- rosbag1 reader and writer,
- extensible type system with serializers and deserializers
- efficient converter between rosbag1 and rosbag2,
- and more.
[continue reading](https://gitlab.com/ternaris/rosbags)


### install
```bash
pip install rosbags

```

### usage
Convert ros1 bag file to folder contain sqlite3 db file and yaml file with bag metadata
 
```bash title="convert ros1 to ros2 bag"
./.local/bin/rosbags-convert <bah file>.bag
```

