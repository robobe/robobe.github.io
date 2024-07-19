---
tags:
    - ros2
    - docker
    - arm
    - rpi
    - core
---

# ROS2 Humble docker running on RPI
Build docker on X64 machine
Run docker on RPi (ARM) machine


## Build
### Docker file
```Dockerfile
# https://github.com/athackst/dockerfiles/blob/main/ros2/humble.Dockerfile
# docker build -f .devcontainer/Dockerfile.humble_dev --target gazebo_dev -t humble:dev .
###########################################
# Base image
###########################################
FROM ubuntu:22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

# upgrade
RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 core
RUN sudo add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-core \
    python3-argcomplete \
  && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV DEBIAN_FRONTEND=

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*


RUN apt-get update && apt-get install -y --no-install-recommends \
  ## dev-ops
  tmux \
  tmuxp \
  python3-pip \
  vim \
  inetutils-ping \
&& rm -rf /var/lib/apt/lists/*

COPY .devcontainer/.tmux.conf /etc/tmux.conf
```


### Build
```
docker buildx build --platform "linux/arm64" --tag humble/arm:prod -f Dockerfile.prod ..
  .
```

## Usage
```bash
docker run -it --rm \
--name=prod \
--hostname=prod \
--user=user \
humble/arm:prod \
/bin/bash

```

### misl
#### tmux config file
```ini
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