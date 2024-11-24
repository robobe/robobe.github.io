---
tags:
    - gstreamer
    - udp
    - stream
---

## RAW

```bash title="send"
gst-launch-1.0 videotestsrc \
! video/x-raw, width=640, height=480, framerate=10/1, format=I420 \
! rtpvrawpay ! application/x-rtp, media=video, encoding-name=RAW \
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

```bash title="receiver"
 gst-launch-1.0 udpsrc port=5000 \
 ! 'application/x-rtp, media=(string)video, encoding-name=(string)RAW, sampling=(string)YCbCr-4:2:0, depth=(string)8, width=(string)640, height=(string)480, framerate=10/1' \
 ! rtpvrawdepay \
 ! videoconvert \
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

## h264

```bash title="send"
gst-launch-1.0 videotestsrc \
! video/x-raw, width=640, height=480, framerate=10/1, format=I420 \
! x264enc \
! rtph264pay \
! udpsink host=127.0.0.1 port=5000
```

```bash title="receiver"
gst-launch-1.0 udpsrc port=5000 \
! 'application/x-rtp, encoding-name=H264, payload=96' \
! rtph264depay \
! h264parse \
! avdec_h264 \
! videoconvert \
! autovideosink
```

### H264 gray scale
```bash title="send"
gst-launch-1.0 videotestsrc \
! video/x-raw, width=640, height=480, framerate=10/1, format=GRAY8 \
! videoconvert \
! video/x-raw, format=I420 \
! x264enc \
! rtph264pay \
! udpsink host=127.0.0.1 port=5000
```

```bash title="receiver"
gst-launch-1.0 udpsrc port=5000 \
! 'application/x-rtp, encoding-name=H264, payload=96' \
! queue \
! rtph264depay \
! avdec_h264 \
! videoconvert \
! autovideosink
```

## h265

```bash title="send"
gst-launch-1.0 videotestsrc \
! video/x-raw, width=640, height=480, framerate=30/1, format=I420 \
! x265enc \
! rtph265pay \
! udpsink host=127.0.0.1 port=5000
```

```bash title="receiver"
gst-launch-1.0 udpsrc port=5000 \
! application/x-rtp, encoding-name=H265, payload=96 \
! rtph265depay \
! h265parse \
! avdec_h265 \
! videoconvert \
! autovideosink
```

```bash title="receiver option 2"
gst-launch-1.0 udpsrc port=5000 \
! application/x-rtp,encoding-name=H265,payload=96 \
! rtph265depay \
! h265parse \
! queue \
! avdec_h265 \
! autovideosink
```

## VP9
### Gray scale

```bash
gst-launch-1.0 videotestsrc \
! video/x-raw,format=GRAY8,width=640,height=480,framerate=30/1 \
! videoconvert \
! video/x-raw,format=I420 \
! vp9enc  threads=4  \
! rtpvp9pay \
! udpsink host=127.0.0.1 port=5000
```

```bash
gst-launch-1.0 udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=VP9,payload=96" \
! rtpvp9depay \
! queue \
! vp9dec \
! videoconvert \
! autovideosink
```