---
title: Tmux and tmuxp
description: 
date: "2022-05-02"
banner: ../ros2.png
tags:
    - tmux
    - tmuxp
    - 101
---

```bash title="install"
pip install --user tmuxp
```

## config

- session_name
- list of `windows`
- list of `panes` for every window in `windows`

```yaml title="config.yaml"
session_name: 2-pane-vertical
windows:
  - window_name: my test window
    panes:
      - echo hello
      - echo hello
```

## usage
```bash
tmuxp load config.yaml
```