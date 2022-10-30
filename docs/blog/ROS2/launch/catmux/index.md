---
title: catmux
tags:
    - tmux
    - launch
---

A python package that wraps launching applications with a tmux script 

## install
```
pip3 install --user catmux
```

## usage

```yaml title="config_demo.yaml"
---
parameters:
  enabled_mavproxy: true
windows:
  - name: sitl
    layout: tiled
    splits:
      - commands:
        - echo "1"
      - commands:
        - echo "2"
    
  - name: mavproxy
    if: enabled_mavproxy
    layout: tiled
    splits:
      - commands:
        - echo "1"
```

```bash title="run"
catmux_create_session config_demo.yaml

```