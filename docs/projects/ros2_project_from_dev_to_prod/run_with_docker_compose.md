---
title: Run with docker compose
tags:
    - ros2
    - projects
    - deploy
---

```yaml
version: '3'
services:
  ros:
    image: humble:dev
    volumes:
      - /home/user/projects/py_cook_ws/src:/tmp
```