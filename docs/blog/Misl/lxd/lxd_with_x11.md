---
title: LXD - Running GUI application
tags:
    - lxd
    - lxc
    - x11
    - profile
---



```c title="x11.profile"
config:
  environment.DISPLAY: :0
  environment.PULSE_SERVER: unix:/home/ubuntu/pulse-native
  nvidia.driver.capabilities: all
  nvidia.runtime: "true"
  user.user-data: |
    #cloud-config
    runcmd:
      - 'sed -i "s/; enable-shm = yes/enable-shm = no/g" /etc/pulse/client.conf'
    packages:
      - x11-apps
      - mesa-utils
      - pulseaudio
description: GUI LXD profile
devices:
  PASocket1:
    bind: container
    connect: unix:/run/user/1000/pulse/native
    listen: unix:/home/ubuntu/pulse-native
    security.gid: "1000"
    security.uid: "1000"
    uid: "1000"
    gid: "1000"
    mode: "0777"
    type: proxy
  X0:
    bind: container
    connect: unix:@/tmp/.X11-unix/X1
    listen: unix:@/tmp/.X11-unix/X0
    security.gid: "1000"
    security.uid: "1000"
    type: proxy
  mygpu:
    type: gpu
name: x11
used_by: []
```

!!! warning "$DISPLAY"
    ```
    X0:
      bind: container
      connect: unix:@/tmp/.X11-unix/X1
    ```

    The number `X1` drive from `$DISPLAY` environment variable


```bash
# Create profile
lxc profile create x11
# Edit/load profile
cat x11.profile | lxc profile edit x11
# add profile to container
lxc profile add ubuntu2204 x11
# or launch with profile
lxc launch  ubuntu2204 --user 1000  --profile default --profile x11 /bin/bash
```

---

### Check

---

# Reference
- [Running X11 software in LXD containers ](https://blog.simos.info/running-x11-software-in-lxd-containers/)