---
title: Docker - custom image
tags:
    - docker
---

# commit

- Run base image (without --rm attribute)
- Install and change the container
- Exit/Pause the container
- Commit the changes to new image

### demo
install git client and save container as new image
  
```bash
# run base image
docker run -it --name my_ubuntu ubuntu:latest

# update and install
apt update
apt install git
exit

# exit running container
docker ps -a
CONTAINER ID   IMAGE     COMMAND   CREATED          STATUS                     PORTS     NAMES
325852406c0a   ubuntu    "bash"    33 seconds ago   Exited (0) 5 seconds ago             my_ubuntu

# commit
# docker commit <container id> <new image name>
docker commit 325852406c0a ea/ubuntu-git

# list images check for new one
docker images
ea/ubuntu-git   latest    1db4558d31c9   5 seconds ago   188MB
ubuntu          latest    27941809078c   7 weeks ago     77.8MB

# run and check
docker run -it --name my_ubuntu --rm ea/ubuntu-git:latest
## run git
git
```

---

# Docker file


# Reference
- [Docker file cheat sheet](https://kapeli.com/cheat_sheets/Dockerfile.docset/Contents/Resources/Documents/index)