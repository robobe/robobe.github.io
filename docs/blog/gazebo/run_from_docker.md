---
tags:
    - gz
    - gazebo
    - garden
    - docker
---

```
docker pull althack/gz:garden-base
docker pull althack/gz:garden-dev

```

```bash
xhost + local:
```

```bash
docker run -it \
--net host \
--privileged \
--rm --name gz \
--hostname gz \
-e DISPLAY=:0 \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
althack/gz:garden-base \
/bin/bash
```

```bash
docker run -it \
--net host \
--privileged \
--rm --name gz \
--hostname gz \
-e DISPLAY=:0 \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v /home/user/xxx:/xxx \
althack/gz:garden-base \
/bin/bash
```