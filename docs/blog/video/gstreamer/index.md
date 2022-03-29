---
title: Gstreamer
description: Gstreamer
date: "2022-11-04"
banner: gstreamer.png
tags:
    - video
    - gstreamer
---


## rtsp client
```cpp title="ffprob"
ffprobe rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mp4

# ffprobe
ffprobe <rtsp url>
# Result
Stream #0:0: Audio: aac (LC), 12000 Hz, stereo, fltp
Stream #0:1: Video: h264 (High), yuv420p(progressive), 240x160 [SAR 32:27 DAR 16:9], 24 fps, 24.08 tbr, 90k tbn, 48 tbc

```


```bash title="pipe"


# 

gst-launch-1.0 rtspsrc location=rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mp4 latency=100 \
! queue \
! rtph264depay \
! h264parse \
! avdec_h264 \
! videoconvert \
! videoscale ! video/x-raw,width=640,height=480 \
! autovideosink
```

```
gst-launch-1.0 rtspsrc location=rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mp4  latency=0 buffer-mode=auto \
! decodebin \
! autovideosink sync=false
```

## Send and play
pppppppppppppppp