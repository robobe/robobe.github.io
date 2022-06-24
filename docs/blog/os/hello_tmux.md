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

# tmux
### install
```bash
sudo apt install tmux
```

### config file
`~/.tmux.conf` or `/etc/tmux.conf`

```bash title="remap prefix from 'C-b' to 'C-a'
unbind C-b
set-option -g prefix C-a
bind-key C-a send-prefix
```

```bash title="close session"
bind C-c kill-session
```

```bash title="support mouse"
set -g mouse on
```

#### like terminator
- split windows to pane
- resize window
- navigate panes (need to resolve)

```bash title="terminator short cuts"
 # do like terminator
bind -n C-E split-window -h
bind -n C-S-Left resize-pane -L 3
bind -n C-S-Right resize-pane -R 3
bind -n C-S-Up resize-pane -U 3
bind -n C-S-Down resize-pane -D 3
bind -n C-O split-window -v

# switch panes using Alt-arrow without prefix (not working)
bind -n M-Left select-pane -L
bind -n M-Right select-pane -R
bind -n M-Up select-pane -U
bind -n M-Down select-pane -D
```

---

# tmuxp
A session manager for tmux

### install
```bash title="install"
sudo apt install tmuxp
#or
pip install --user tmuxp
```

### config file

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

### usage
```bash
tmuxp load config.yaml
```

# tmux conf example

```bash title="~/.tmux.conf"
# unbind
unbind C-b
unbind '"'
unbind %

# base1 numbering
set -g base-index 1
setw -g pane-base-index 1

#bind ctrl-a as a prefix
set-option -g prefix C-a
bind-key C-a send-prefix
# kill session
bind C-c kill-session

# mouse
set -g mouse on

 # do like terminator
bind -n C-E split-window -h
bind -n C-S-Left resize-pane -L 3
bind -n C-S-Right resize-pane -R 3
bind -n C-S-Up resize-pane -U 3
bind -n C-S-Down resize-pane -D 3
bind -n C-O split-window -v

# switch panes using Alt-arrow without prefix (not working)
bind -n M-Left select-pane -L
bind -n M-Right select-pane -R
bind -n M-Up select-pane -U
bind -n M-Down select-pane -D

# Shift arrow to switch windows

bind n next-window
bind p previous-window

bind c new-window -c "#{pane_current_path}"

bind r source-file ~/.tmux.conf
```