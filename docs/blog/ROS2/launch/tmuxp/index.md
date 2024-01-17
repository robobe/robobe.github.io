---
tags:
    - tmux
    - launch
    - tmuxp
    - ros2
    - tips
---

# tmuxp

A session manager for tmux


## install
```
pip3 install tmuxp
```

## usage

```yaml title="config_demo.yaml"
session_name: gazebo
windows:
  - window_name: gazebo
    layout: tiled
    shell_command_before:
      - cd ros workspace
      - source install/setup.bash
    panes:
      - ros2 launch ...
      - ros2 launch ...
```

```bash title="run"
tmuxp load config_demo.yaml

```

---

## tmux tips
### Copy / Paste
Copy / paste between tmux pane / window

!!! note ""
    **ctrl-a** replace **ctrl-b** as tmux prefix
     

1. Select source pane to copy from
2. Press (ctrl-a) and `[` to enter copy mode
3. Press `Ctrl-spacebar` to start copy session
4. Select desire text witch arrows
5. Press `ctrl-w` to copy the text to **tmux-buffer**
6. Select another pane /window
7. Press (ctrl-a) and `]` to paste buffer


### Copy from tmux to linux clipboard

!!! tip copy to tmux
    Copy from linux buffer to tmux using `ctrl shift-v`


1. Install `xclip`    
2. Add config line to `.tmux.conf`

```
bind C-c run "tmux save-buffer - | xclip -i -sel clipboard"

```

---

## tmux config
- replace c-b with c-a
- more

```bash title=".tmux.conf"
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

bind C-a run "tmux save-buffer - | xclip -i -sel clipboard"

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

# settings

```