---
title: Gstreamer
description: Gstreamer
date: "2022-11-04"
banner: images/gstreamer.png
tags:
    - video
    - gstreamer
    - nvidia
---

## Install
```bash
sudo apt-get install libgstreamer1.0-0 \
    gstreamer1.0-dev \
    gstreamer1.0-tools \
    gstreamer1.0-doc

sudo apt-get install gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good 

# nvidia plugs
sudo apt-install deepstream-6.0
```

## Demos

```bash title="very basic sample"
gst-launch-1.0 videotestsrc \
! videoconvert \
! autovideosink
```

### Streaming
#### streaming source
```bash title="H265"
gst-launch-1.0 videotestsrc \
! video/x-raw,width=640,height=480,framerate=25/1,format=I420 \
! timeoverlay valignment=4 halignment=1 \
! nvvidconv \
! 'video/x-raw(memory:NVMM),width=640,height=480' \
! nvv4l2h265enc insert-sps-pps=1 idrinterval=15 \
! h265parse \
! rtph265pay \
! udpsink port=5000 host="192.168.1.207"
```

#### read vide with sdp file
```bash title="sdp"
c=IN IP4 192.168.1.207
m=video 5000 RTP/AVP 96
a=rtpmap:96 H265/90000
```

---

## rtsp client
```bash title="ffprob"
ffprobe rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mp4


