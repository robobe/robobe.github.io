---
title: Run GUI application on docker
tags:
    - docker
---

Run x11 app on docker image


### Dockerfile

- Install x11-app package
- Run `xeyes`


```dockerfile
FROM ubuntu:22.04 AS base

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
  && rm -rf /var/lib/apt/lists/* \
  && echo 'export PS1="🐳 \u@\h: \w\a\ # "' > /home/$USERNAME/.bashrc

RUN apt-get update \
  && apt-get install -y x11-apps

CMD ["/usr/bin/xeyes"]
```

---

### Usage

!!! tip Don't forget
    The xhost command adds or deletes host names on the list of machines from which the X Server accepts connections.
    ```bash
    xhost + local:docker
    ```

#### cli

```
docker run -it \
--rm \
--user user \
--workdir /home/user \
--hostname dev \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
ubuntu:gui
```


#### docker-compose
```yaml title="docker-compose.yaml"
version: "3.0"
services:
  ubuntu:
    image: ubuntu:gui
    hostname: dev
    user: user
    working_dir: /home/user
    environment:
      - DISPLAY=${DISPLAY}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
    network_mode: host
```
     
