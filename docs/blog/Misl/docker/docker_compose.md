---
tags:
    - docker
    - compose
    - tutorial
---

# Docker compose

Is a tool to define docker multi containers

!!! note "docker-compose vs docker compose"
    The docker compose (with a space) is a **newer** project 

    [From Stack overflow](https://stackoverflow.com/questions/66514436/difference-between-docker-compose-and-docker-compose)
     

## install
[How to Install Docker Compose on Ubuntu 22.04](https://linuxhint.com/install-docker-compose-ubuntu-22-04/)



  
```bash
mkdir -p ~/.docker/cli-plugins/
```

- Check release version from [github](https://github.com/docker/compose/)

```bash
curl -SL https://github.com/docker/compose/releases/download/v2.19.1/docker-compose-linux-x86_64 -o ~/.docker/cli-plugins/docker-compose
```

```bash
chmod +x ~/.docker/cli-plugins/docker-compose
```

```bash
docker compose version
# check docker engine version for compose file format compatibility
docker version
```


## Reference
- [Compose file version 3 reference](https://docs.docker.com/compose/compose-file/compose-file-v3/)