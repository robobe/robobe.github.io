---
title: Docker 
tags:
    - docker
---


## Docker desktop
[Install Docker Desktop on Ubuntu](https://docs.docker.com/desktop/install/ubuntu/)

- Download deb
- install with `apt`
  

```sudo apt-get install ./docker-desktop-<version>-<arch>.deb```


## useful cli
```bash title="remove all containers"
docker rm $(docker ps -a -q)
```

```bash title="remove all images"
sudo docker rmi $(sudo docker images -q)
```

---

## Dockerfiles
### minimal ubuntu

```dockerfile
--8<-- blog/Misl/docker/dockerfiles/ubuntu/minimal/Dockerfile
```

noninteractive – You use this mode when you need zero interaction while installing or upgrading the system via apt

--no-install-recommends By passing this option, the user lets apt-get know not to consider recommended packages as a dependency to install.

# run 
```bash
# run as root
docker run -it --rm --name="test" ubuntu_base /bin/bash

# run as user
docker run -u user -it --rm --name="test" ubuntu_base /bin/bash

```

---

# Reference
- [Behnam Asadi Docker concepts](https://ros-developer.com/2017/11/08/docker/)