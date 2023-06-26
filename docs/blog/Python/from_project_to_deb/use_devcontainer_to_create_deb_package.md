---
tags:
    - deb
    - stdeb
    - python
    - project
    - package
    - docker
    - devcontainer
---

# Using Docker to generated debian package using stdeb
Install and run `stdeb` on Docker container using vscode `.devcontainer`

- Add tasks to build and inspect the deb file
  
## Dockerfile
```dockerfile title="Dockerfile" linenums="1" hl_lines="1"
FROM ubuntu:22.04

RUN apt-get update && \
    apt-get install -y \
        python3 \
        python3-pip \
        python3-venv \
        net-tools \
        iputils-ping \
        && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y \
        python3-all \
        debhelper \
        dh-python \
        python3-stdeb \
        locales \
        locales-all \
        && rm -rf /var/lib/apt/lists/*

# Set the locale
ENV LC_ALL en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8

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
  
CMD ["/bin/bash"]
```

## devcontainer

```json
{
    "build": {
        "dockerfile": "Dockerfile",
        "context": ".."
    },
    "remoteUser": "user",
    "containerUser": "user"
}
```

## vscode tasks
```json title="vscode tasks"
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "build deb",
            "type": "shell",
            "command": "python3 setup.py --command-packages=stdeb.command bdist_deb",
            "problemMatcher": []

        },
        {
            "label": "deb package info",
            "type": "shell",
            "command": "dpkg -I deb_dist/python3-${input:pkg_name}*.deb",
            "problemMatcher": []
        },
        {
            "label": "list deb files",
            "type": "shell",
            "command": "dpkg -c deb_dist/python3-${input:pkg_name}*.deb",
            "problemMatcher": []
        }

    ],
    "inputs": [
        {
            "id": "pkg_name",
            "type": "promptString",
            "description": "name from setup.py"
        }
    ]
}
```