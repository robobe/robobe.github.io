---
tags:
    - vcstool
    - ros2
    - package manager
    - project settings
---
# vcstool hello
vcstool is a **version control** system tool designed to make working with multiple repositories easier. It is most commonly used for getting open source repositories from GitHub, without having to maintain them yourselves in your project.

```bash
sudo apt install python3-vcstool
```

## usage
### .repos
- create file `project.repos` for example


```yaml
repositories:
  turtlebot3/turtlebot3:
    type: git
    url: https://github.com/ROBOTIS-GIT/turtlebot3.git
    version: galactic-devel
```

### clone / import
- from workspace folder


```bash
vcs import src < project.repos

```