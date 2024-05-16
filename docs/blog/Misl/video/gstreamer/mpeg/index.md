---
tags:
    - mpeg
    - gstreamer
    - 
---

MPEG-TS (Transport stream) are multimedia **container** format, it's use for transmit video and audio over unreliable channels

!!! note "MPEG-TS vs MP4"
     mpeg-ts and mp4 are both containers
     - mpeg-ts design for streaming
     - mp4 is more for file storage
     - mpeg-ts can contain multiple channels audio, video and data


!!! note "What is MPEG"
    mpeg are standard for video and audio compression (MPEG-1, MPEG-2 ...)

## Stream

```bash
gst-launch-1.0 -ve videotestsrc \
! video/x-raw, framerate=30/1 \
! videoconvert \
! x264enc \
! mpegtsmux \
! rtpmp2tpay \
! udpsink host=127.0.0.1 port=5000
```

```bash
sudo gst-launch-1.0 -v udpsrc port=5000 buffer-size=10000000 \
! tsparse \
! tsdemux  parse-private-sections=TRUE \
! h264parse \
! avdec_h264 \
! videoconvert \
! autovideosink

```

!!! warning ""
    ```
    GstTSDemux:tsdemux0: CONTINUITY: Mismatch packet 12, stream 15
    ```

    increase buffer size

    ```
    udpsrc port=5000 buffer-size=10000000
    ```
     