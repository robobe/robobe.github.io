---
tags:
    - gstreamer
    - pipe
    - shmsink
    - shmsrc
    - intersink
    - intersrc
---

## connect two pipes
- **nter*** element are design to share and multiplex content between two
pipelines running in the same process
- **shm*** element are design to share data between pipes run on separate process


### intervideosink / intervideosrc
Connect pipe line in same process

[Decoupled pipelines in GStreamer](https://erit-lvx.medium.com/decoupled-pipelines-in-gstreamer-e67b24fd21c)

Complete pipeline = pipeline source + pipeline sink

```bash
gst-launch-1.0 videotestsrc \
! video/x-raw,width=640,height=480,format=RGB \
! videoconvert \
! intervideosink \
intervideosrc \
! queue \
! video/x-raw,width=640,height=480,format=RGB \
! videoconvert \
! autovideosink

```

## shmsink / shmsrc
Connect pipeline between process

```bash title="send"
gst-launch-1.0 videotestsrc \
! video/x-raw,width=640,height=480,format=RGB,framerate=30/1 \
! videoconvert \
! shmsink socket-path=/tmp/foo sync=true  wait-for-connection=false shm-size=10000000 
```

```bash title="recive"
gst-launch-1.0 shmsrc socket-path=/tmp/foo \
! video/x-raw,width=640,height=480,format=RGB,framerate=30/1 \
! videoconvert \
! autovideosink
```