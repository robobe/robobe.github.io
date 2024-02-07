---
tags:
    - ros2
    - vscode
    - devcontainer
    - docker
    - settings
---

# VSCode devcontainer for ROS2 project

```json title="base devcontainer.json"
{
    "name": "ROS devcontainer",
    "dockerFile": "Dockerfile",
    "workspaceFolder": "/workspaces/smach_ws",
    "remoteUser": "ros",
    "runArgs": [
        "--hostname=dev"
    ],
    "customizations": {
        "vscode": {
            "settings": {
                "python.analysis.extraPaths": [
                    "/opt/ros/humble/lib/python3.10/site-packages/"
                ],
                // Autocomplete from ros python packages
                "python.autoComplete.extraPaths": [
                    "/opt/ros/humble/lib/python3.10/site-packages/"
                ],
                "search.exclude": {
                    "**/build": true,
                    "**/install": true,
                    "**/log": true
                }
            },
            "extensions": [
				"DotJoshJohnson.xml",
				"ms-python.python",
				"ms-vscode.cpptools",
				"redhat.vscode-yaml",
				"streetsidesoftware.code-spell-checker",
				"twxs.cmake",
				"yzhang.markdown-all-in-one"
            ]
        }
    }
}
```


## Dockerfile

```Dockerfile title="base on Allison template"
FROM althack/ros2:humble-dev 

# ** [Optional] Uncomment this section to install additional packages. **
#
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update \
#    && apt-get -y install --no-install-recommends <your-package-list-here> \
#    #
#    # Clean up
#    && apt-get autoremove -y \
#    && apt-get clean -y \
#    && rm -rf /var/lib/apt/lists/*
# ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
```


---

## X11 support
- Add to `runArgs`

```
"--volume=/tmp/.X11-unix:/tmp/.X11-unix"
```

- Add containerEnv
```json
"containerEnv": {
    "DISPLAY": "${localEnv:DISPLAY}" // Needed for GUI try ":0" for windows
}
```

---

## devcontainer tips

- map ports

```
"appPort": []
```

- Docker context

```
"context": ".."
```

## settings

- python analysis extra path for ros package

```bash
install/<package>/local/bin/python3.10/dist-packages

# merge-install
install/local/bin/python3.10/dist-packages
```

---

## Reference
- [Allison vscode workspace ](https://github.com/athackst/vscode_ros2_workspace)
- [Allison humble dockerfile](https://github.com/athackst/dockerfiles/blob/main/ros2/humble.Dockerfile)
- [Setup ROS 2 with VSCode and Docker](https://docs.ros.org/en/iron/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html)
- [devcontainer for ros2 project](https://dev.to/koheikawata/devcontainer-for-ros2-project-31ng)