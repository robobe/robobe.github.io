---
title: Eclipse cyclonedds python bindings
tags:
    - dds
    - cyclonedds
---

## LAB
Install cyclonedds on lxc container, config vscode as remote container

- Install cyclonedds python bindings from pip
- Config VSCode remote container
- Run [Hello world](https://github.com/eclipse-cyclonedds/cyclonedds-python/tree/master/examples/helloworld) example


```
rtspsrc location=rtsp://<user></user>:<pass>@<ip>:<port>/<channel> caps=“application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264” latency=100 ! queue ! rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink
```