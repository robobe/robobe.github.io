---
tags:
    - vscode
    - dev
    - docker
    - dev container
---
# VSCode dev containers

[devcontainer for python project](python_project.md)

---

# docker tips
## Installing non-root user
```Dockerfile
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # [Optional] Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo git-core bash-completion \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  # Cleanup
  && rm -rf /var/lib/apt/lists/*

USER $USERNAME
WORKDIR /home/$USERNAME
```

---

# dev container Tips
## set non root user
```json
"remoteUser": "nonroot"
```

```json
"containerUser": "nonroot"
```

---

# Reference
- [Use a Docker container as a development environment with Visual Studio Code](https://learn.microsoft.com/en-us/training/modules/use-docker-container-dev-env-vs-code/)
- [Dev Container metadata reference](https://containers.dev/implementors/json_reference/)
