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
mkdir -p ~/ws/src && cd ~/ws
wget https://raw.githubusercontent.com/colcon/colcon.readthedocs.org/main/colcon.repos
vcs import src < colcon.repos
colcon build
colcon test
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