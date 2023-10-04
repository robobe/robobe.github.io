---
tags:
    - vscode
    - docker
    - remote
    - devcontainer
---
# Develop on a remote Docker host

Using:
- [ssh remote extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-ssh)
- [Docker extension](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker)


## Demo

!!! tip "ssh key base authentication"
    ```bash
    ssh-copy-id 
    ```
     
Connect to remote docker run on RPI

### Settings

```json
"docker.environment": {
    "DOCKER_HOST": "ssh://ubuntu@rpi4"
}
```

### Dev container

```json
{
  "image": "Dockerfile",
  "workspaceFolder": "/workspace",
  "workspaceMount": "source=remote-workspace,target=/workspace,type=volume"
}
```

#### using docker volume

```
docker volume ls
DRIVER    VOLUME NAME
local     remote-workspace
local     vscode
```

#### using host folder

```
"workspaceMount": "source=/absolute/path/on/remote/machine,target=/workspace,type=bind,consistency=cached"
```

!!! tip
    Create to source folder before open devcontainer
     
```
"workspaceMount": "source=/home/ubuntu/workspace,target=/workspace,type=bind,consistency=cached"
```


![](images/open_folder_in_container.png)



## Dockerfile

```dockerfile
```

---

## Reference
- [Develop on a remote Docker host](https://code.visualstudio.com/remote/advancedcontainers/develop-remote-host)
- [Setting Locale in Docker](https://leimao.github.io/blog/Docker-Locale/)