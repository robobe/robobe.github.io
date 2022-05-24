---
title: Jetson Nano
description: Install jetson nano
date: "2022-24-05"
banner: images/nano.png
tags:
    - nvidia
    - nano
---


```yaml title="/ansible/roles/jetson/defaults/main.yaml"
---
ubuntu_release: focal

new_user:
  name: user
  shell: /bin/bash
  password: user
```

# Reference
- [Create your own image for jetson nano board](https://pythops.com/post/create-your-own-image-for-jetson-nano-board)