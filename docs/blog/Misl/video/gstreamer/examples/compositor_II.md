---
tags:
    - gstreamer
    - compositor
    - alpha
---

Using compositor and alpha plugin to merge to image together

```
gst-launch-1.0 videotestsrc \
! videoconvert \
! compositor name=comp \
sink_0::xpos=0 sink_0::ypos=0 \
sink_1::xpos=0 sink_1::ypos=0 \
! videoconvert \
! autovideosink \
videotestsrc pattern=ball \
! videoconvert \
! alpha method=custom target-r=0 target-g=0 target-b=0 black-sensitivity=128 white-sensitivity=0 \
! videoconvert \
! comp.
```

## alpha


---
```bash title="source"
gst-launch-1.0 videotestsrc pattern=ball \
! video/x-raw, framerate=30/1, format=I420 \
! jpegenc \
! rtpjpegpay \
! udpsink host=127.0.0.1 port=5000
```

```bash
gst-launch-1.0 videotestsrc \
! videoconvert \
! compositor name=comp \
sink_0::xpos=0 sink_0::ypos=0 \
sink_1::xpos=0 sink_1::ypos=0 \
! videoconvert \
! autovideosink \
udpsrc port=5000 \
! application/x-rtp, encoding-name=JPEG,payload=26 \
! queue \
! rtpjpegdepay \
! jpegdec \
! videoconvert \
! video/x-raw, format=I420 \
! alpha method=custom target-r=0 target-g=0 target-b=0 black-sensitivity=128 white-sensitivity=0 \
! videoconvert \
! comp.
```