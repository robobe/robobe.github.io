---
tags:
    - docker
    - container
    - state
---

# Docker container state and management command from cli

![](images/docker_container_state.png)


### Created
Docker assigns the created state to the containers that were never started ever since they were created.
NO CPU or memory is used by the containers in this state.

```bash
docker create -it --name my_container busybox
```

### Running

```bash
docker start my_container
docker inspect -f '{{.State.Status}}' my_container
#
running
```

```bash
docker run -it --name my_container busybox
```

```bash
docker stop my_container
docker inspect -f '{{.State.Status}}' my_container
#
exited
```

### Pause
Pause the state of docker container that suspends all the processes in the container
A paused container consumes the same memory used while running but no CPU. 

```bash
docker pause my_container
docker inspect -f '{{.State.Status}}' my_container
#
paused
```

```bash
docker unpause my_container
docker inspect -f '{{.State.Status}}' my_container
#
running
```

### Delete

```
docker rm my_container
```

!!! tip "container resource usage statistics"
    ```
    docker stats --no-stream
    ```
     
---

## commands
### remove all stopped containers

```bash
docker rm $(docker ps -a -q -f status=exited)
```

---

## Reference
- [States of a Docker Container](https://www.baeldung.com/ops/docker-container-states)