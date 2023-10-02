---
tags:
    - vscode
    - remote
    - ssh
    - debug
    - python
---

# VSCode python remote debugging 
Using [ssh remote](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-ssh) extension and vscode-server to run python remote debugging


---

## Demo
Using ssh remote to debug python app run on docker

- Create docker image base on ubuntu with python and openssh-server
- Add user `user` to docker with password user

```Dockerfile
FROM ubuntu:22.04 as python_base
ARG version
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Set the locale
ENV LC_ALL en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo tzdata \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/* 

RUN echo 'user:user' | chpasswd

RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    vim \
    iputils-ping \
    net-tools \
    openssh-server \
    && apt-get -y clean && rm -rf /var/lib/apt/lists/*


ENV PATH=${PATH}:/home/user/.local/bin

# Add ssh keys check 
COPY dockers/id_ed25519.pub /home/user/.ssh/authorized_keys
RUN chown user:user /home/user/.ssh/authorized_keys && chmod 600 /home/user/.ssh/authorized_keys

EXPOSE 22
CMD ["/usr/sbin/sshd", "-D"]
```

```bash title="build image"
docker build -t ubuntu:python -f Dockerfile .
```

```bash title="run docker"
docker run -it --rm --name dev \
--hostname dev \
--user user \
-p 2222:22 \
ubuntu:python \
/bin/bash
```

!!! note "ssh as daemon"
    For known run `/bin/bash` and run ssh manually using
    `sudo service ssh start`

#### install debugpy

```
pip install debugpy
```

#### connect using ssh remote

##### Connect
![](images/connect_to_host.png)

Add new SSH host or connect to exists

![](images/connect_to_host_2.png)

Connection open new VSCode and download vscode-server into `~/.vscode-server` folder

Install python extension on remote

![](images/remote_extension.png)

![](images/install_python_ext_on_remote.png)

##### Debug
Open remote application in VSCode and
Set break point and 

![](images/set_breakpoint.png)

---

###### Create launch configuration

![](images/ssh_remote_connect.png)
     

```json title="launch.json"
"configurations": [
        {
            "name": "Python: Remote Attach",
            "type": "python",
            "request": "attach",
            "connect": {
                "host": "localhost",
                "port": 5678
            },
            "pathMappings": [
                {
                    "localRoot": "${workspaceFolder}",
                    "remoteRoot": "."
                }
            ],
            "justMyCode": true
        }
    ]
```


#### Run debugger

```bash title="from vscode terminal"
python3 -m debugpy --listen 0.0.0.0:5678 --wait-for-client hello.py
```

#### Connect with vscode debug
Press `F5` or launch selected configuration

![](images/ssh_remote_debug.png)