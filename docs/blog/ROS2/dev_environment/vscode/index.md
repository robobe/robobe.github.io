---
tags:
    - ros2
    - vscode
    - devcontainer
    - docker
---

# ROS2 and VSCode



## Dockerfile

```Dockerfile
FROM althack/ros2:humble-full


# Example of installing programs
RUN apt-get update \
    && apt-get install -y \
      vim \
      # ros
      ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

```

!!! note user and sudo
     Allison base docker add user `ros` and install and config `sudo`

    ```bash
    # Create a non-root user
    ARG USERNAME=ros
    ARG USER_UID=1000
    ARG USER_GID=$USER_UID

    RUN groupadd --gid $USER_GID $USERNAME \
      && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME

    # Set up sudo
    RUN apt-get update \
      && apt-get install -y sudo \
      && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
      && chmod 0440 /etc/sudoers.d/$USERNAME \
      && rm -rf /var/lib/apt/lists/*
    ```



---

## devcontainer

```json title="devcontainer.json"
{
    "name": "ros2",
    "workspaceFolder": "/workspaces/ros_tutorial",
    "build": {
        "context": "..",
        "dockerfile": "Dockerfile"
    },
    "remoteUser": "ros",
    "runArgs": [
        "--env-file=env",
        "--network=host",
        "--ipc=host",
        "--cap-add=SYS_PTRACE",
        "--security-opt=seccomp:unconfined",
        "--security-opt=apparmor:unconfined",
        "--device=/dev/dri:/dev/dri"
	  ],
    "containerEnv": {
        "DISPLAY": "${localEnv:DISPLAY}",
        "QT_X11_NO_MITSHM": "1"
    },
    "mounts": [
        "source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind,consistency=cached"
    ],
    "customizations": {
        "vscode": {
            "extensions": [
                
            ],
            "settings": [

            ]
        }
    }
}
```

---

### Environment variable
environment variable set in container using `--env-file` argument

```title="env"
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {line_number})"
```


---

## Python

```json
{
  "python.autoComplete.extraPaths": [
    "/opt/ros/humble/lib/python3.10/site-packages",
    "/opt/ros/humble/local/lib/python3.10/dist-packages",
    "/my_project/build/package1",
    "/my_project/build/package2"
  ],
  "python.analysis.extraPaths": [
    "/opt/ros/humble/lib/python3.10/site-packages",
    "/opt/ros/humble/local/lib/python3.10/dist-packages",
    "/my_project/build/package1",
    "/my_project/build/package2"
  ]
}
```

---

## Reference
- [ROS 2 and VSCode](https://picknik.ai/vscode/docker/ros2/2024/01/23/ROS2-and-VSCode.html)
- [Allison docker file](https://github.com/athackst/dockerfiles)
  - [docker hub](https://hub.docker.com/r/althack/ros2)
- [osrf docker file](https://hub.docker.com/r/osrf/ros)