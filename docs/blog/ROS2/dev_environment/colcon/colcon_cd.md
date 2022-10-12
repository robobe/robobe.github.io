---
title: Colcon_cd
description: A shell function for colcon-core to change the current working directory
date: "2022-04-30"
banner: colcon.png
tags:
    - ros2
    - colcon_cd
---
A shell function for colcon-core to change the current working directory

!!! Note
    `colcon_cd` search pkg from current directory tree

## install and setup
```bash title="apt"
sudo apt install python3-colcon-common-extensions
```

```bash title="setup"
Add `colcon_cd` to `.bashrc`
# debian install
echo 'source /usr/share/colcon_cd/function/colcon_cd.sh' >> ~/.bashrc
```