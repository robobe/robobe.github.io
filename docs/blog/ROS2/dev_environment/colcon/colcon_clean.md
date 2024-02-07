---
tags:
    - ros2
    - colcon
    - clean
---

# Colcon-clean

An extension for colcon-core to clean package workspaces
[more](https://github.com/colcon/colcon-clean)

## install

```
sudo apt install python3-colcon-clean
```

## usage

### Subverbs
- workspace
- packages


```bash
colcon clean workspace -y
```

```bash
colcon clean packages --package-select <package name>

```

[more]()