---
tags:
    - gazebo
    - classic
    - vscode
    - devcontainer
    - docker
---
Install gazebo on docker
Run docker using gpu acceleration using (Intel/nvidia)

- Using Docker
- Using VSCode devcontainer

---

## Docker
Build gazebo-11 docker using install script from gazebo site [install script](http://get.gazebosim.org)

!!! note "Hardware acceleration"
    [Check this post for more ](https://www.blackcoffeerobotics.com/blog/5-ways-to-speedup-gazebo-simulations)
    **Intel**

    install

    ```bash
    apt-get -y install libgl1-mesa-glx libgl1-mesa-dri
    ```

    Add to docker run

    ```bash
    docker run -it --privileged --net=host \    
    --name test_image_container \
    --env="QT_X11_NO_MITSHM=1"  \
    --env="DISPLAY"  \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix \
    --device=/dev/dri:/dev/dri \
    test_image:latest
    ```
     

### Docker file
- Base on ubuntu 22.04
- Add none root user
- Install dependencies
- Install Gazebo using `install script`

```
FROM ubuntu:22.04 as base

ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # [Optional] Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  # Cleanup
  && rm -rf /var/lib/apt/lists/* 



RUN apt-get update \
  && apt-get install -y \
  build-essential \
  cmake \
  git \
  gnupg \
  lsb-release \
  wget \
  vim \
  curl \
  libgl1-mesa-glx \
  libgl1-mesa-dri \
  # Cleanup
  && rm -rf /var/lib/apt/lists/* 

RUN curl -sSL http://get.gazebosim.org | sh
```

### Run
- Using Intel hardware acceleration

```bash
docker run -it --rm \
--name gz11 \
--hostname gz11 \
--privileged \
-v /home/user/projects/gazebo_tutorial:/workspaces/gazebo_tutorial \
--env="DISPLAY=$DISPLAY"  \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="$XAUTHORITY:$XAUTHORITY" \
--env="XAUTHORITY=$XAUTHORITY" \
--net=host \
 --device=/dev/dri:/dev/dri \
gazebo:11 \
/bin/bash
```

---

## VSCode devcontainer

```json
{
    "name": "gazebo11",
    "workspaceFolder": "/workspaces/gazebo_tutorial/",
    "image": "gazebo:11",
    "remoteUser": "user",
    "runArgs": [
        "--name=gz11",
        "--hostname=gz11",
        "--privileged",
        "--network=host",
        "--hostname=gz",
        "--device=/dev/dri:/dev/dri" # intel acceleration
    ],
    //"postStartCommand": "source /workspaces/gazebo_tutorial/.devcontainer/post_start_script.sh",
    "containerEnv": {
        "DISPLAY": "unix:0",
        "QT_X11_NO_MITSHM": "1"
    },
    "mounts": [
        "source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind,consistency=cached"
      ],
    "customizations": {
        "vscode": {
            "extensions": [
                "ms-vscode.cpptools",
                "twxs.cmake",
                "albert.tabout",
                "redhat.vscode-xml"
            ],
            "settings": {
                "[xml]": {
                    "editor.defaultFormatter": "redhat.vscode-xml"
                }
            }
        }
    }  
}
```

---

## Reference
- [Gazebo Simulator : 5 Ways to Speedup Simulations](https://www.blackcoffeerobotics.com/blog/gazebo-simulator-5-ways-to-speedup-simulations)