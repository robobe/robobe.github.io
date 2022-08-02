---
title: Hello docker - install
tags:
    - docker
    - tutorial
---

!!! note
    docker image are immutable

    docker container are temporary and not persists state


| Command  | Description  |
|---|---|
| pull  | Download docker from Registry/Hub to local storage  |
| images  | list all images  |
| search  |   |
| commit  | commit container file changes into new image  |
| rm  | Remove one or more containers  |



### Executing the Docker Command Without Sudo

```bash
# add user to docker group
sudo usermod -aG docker ${USER}
# execute new session or logout
su - ${USER}
# check
groups
#check if docker seen in the output
```

### Save and Restore containers

```bash
docker image save -o image.tar <image_id>
```

```bash
docker image load -i image.tar 
```

### login as none root
```
docker run -it --rm --user user <image>
```