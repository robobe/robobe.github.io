---
tags:
    - gazebo
    - dev
    - env
    - docker
---


```
sudo apt install -y nvidia-container-toolkit

```


### x11
```bash title="X11"
docker run -it --rm \
--gpus all \
--net=host \
--user=user \
--hostname=gz \
--env DISPLAY=$DISPLAY \
--env QT_X11_NO_MITSHM=1 \
--volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
harmonic:cuda \
/bin/bash
```

```
```

---

```bash title="wayland"
docker run -it --rm \
--gpus all \
--net=host \
--user=user \
--hostname=gz \
--env WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
--env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
--env QT_QPA_PLATFORM=wayland \
--env OGRE_RENDER_SYSTEM=OpenGL3 \
--env OGRE_WINDOW_MODE=windowed \
--volume /run/user/$(id -u)/$WAYLAND_DISPLAY:/run/user/$(id -u)/$WAYLAND_DISPLAY \
--volume $XDG_RUNTIME_DIR:$XDG_RUNTIME_DIR \
harmonic:cuda \
/bin/bash
```

