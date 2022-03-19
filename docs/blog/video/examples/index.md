---
title: FFMpeg and Gstreamer Examples
description: FFMpeg and Gstreamer 
date: "2022-11-04"

tags:
    - video
    - ffmpeg
    - gstreamer
    - cheat-sheet
---

## jpeg
- Stream jpeg stream over udp using gstreamer
- Play stream using FFPlay and sdp file

```title="Terminal1: gstreamer udp stream"
gst-launch-1.0 -v videotestsrc  \
 ! video/x-raw,width=640,height=480 \
 ! timeoverlay \
 ! tee name="local" \
 ! queue \
 ! videoconvert \
 ! autovideosink sync=false\
 local. \
 ! queue \
 ! jpegenc \
 ! rtpjpegpay \
 ! udpsink host=127.0.0.1 port= 5000
```

```title="udp.sdp"
m=video 5000 RTP/AVP 26
a=rtpmap:26 JPEG/90000;
c=IN IP4 127.0.0.1
```

```title="Terminal 2: ffplay"
ffplay \
    -protocol_whitelist file,rtp,udp \
    -fflags nobuffer \
    -flags low_delay \
    -framedrop \
    -i udp.sdp
```