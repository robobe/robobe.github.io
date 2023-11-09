---
tags:
    - gstreamer
    - udp
---

## RAW

```bash title="send"
gst-launch-1.0 videotestsrc \
! video/x-raw, width=640, height=480, framerate=10/1, format=I420 \
! rtpvrawpay ! 'application/x-rtp, media=(string)video, encoding-name=(string)RAW' \
! udpsink host=127.0.0.1 port=5000
```

```bash title="receiver"
 gst-launch-1.0 udpsrc port=5000 \
 ! 'application/x-rtp, media=(string)video, encoding-name=(string)RAW, sampling=(string)YCbCr-4:2:0, depth=(string)8, width=(string)640, height=(string)480, framerate=10/1' \
 ! queue \
 ! rtpvrawdepay \
 ! video/x-raw, format=I420 \
 ! autovideosink
```

## mjpeg

```bash title="send"
gst-launch-1.0 videotestsrc pattern=ball \
! video/x-raw, width=320, height=240, framerate=10/1, format=I420 \
! jpegenc \
! rtpjpegpay \
! udpsink host=127.0.0.1 port=5000
```

```bash title="receiver"
gst-launch-1.0 udpsrc port=5000 \
! application/x-rtp, encoding-name=JPEG,payload=26 \
! rtpjpegdepay \
! jpegdec \
! autovideosink
```