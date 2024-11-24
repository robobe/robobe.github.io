---
title: Docker 
tags:
    - docker
---
- [Usage examples](docker_usage.md)
- [Docker Volume](docker_volume.md)
- [Docker Network](docker_networking.md)
- [Docker with none root user](docker_none_root_user.md)
- [Run X11 app on docker image](docker_gui_tester.md)
- [Docker with nvida support](docker_nvidia_install.md)

---

## Template
### Add none root user
[Docker with none root user](docker_none_root_user.md)

```dockerfile
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
```


### using apt

```dockerfile
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        # app list
    #cleanup
    && rm -rf /var/lib/apt/lists/*
```

### python

```dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-venv \
    python-is-python3 \
    wget \
  && rm -rf /var/lib/apt/lists/*
```

---

## Tips

```bash title="remove none images"
x```


---

# Reference
- [Docker Build: A Beginner’s Guide to Building Docker Images](https://stackify.com/docker-build-a-beginners-guide-to-building-docker-images/)
- [docker cheat-sheet](https://github.com/ChristianLempa/cheat-sheets/blob/main/tools/docker.md)