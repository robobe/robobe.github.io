---
tags:
    - gstreamer
    - acceleration
    - va-api
    - intel
---

# Gstreamer hardware acceleration

```
sudo apt install intel-media-va-driver
sudo apt install gstreamer1.0-vaapi
```

```bash title="simple"
gst-launch-1.0 videotestsrc \
! video/x-raw,format=RGBA \
! videoconvert \
! vaapisink
```

## h264 demo
### CPU oriented

```bash title="send"
gst-launch-1.0 videotestsrc \
! video/x-raw, width=640, height=480, framerate=30/1, format=RGB \
! videoconvert \
! x264enc \
! rtph264pay \
! udpsink host=127.0.0.1 port=5000
```


```bash title="receiver"
gst-launch-1.0 udpsrc port=5000 \
! application/x-rtp, encoding-name=H264, payload=96 \
! rtph264depay \
! h264parse \
! avdec_h264 \
! videoconvert \
! autovideosink
```


### Hardware oriented
```bash title="send"
gst-launch-1.0 videotestsrc \
! video/x-raw, width=640, height=480, framerate=30/1, format=RGB \
! videoconvert \
! vaapih264enc \
! h264parse \
! rtph264pay \
! udpsink host=127.0.0.1 port=5000
```

```bash title="receiver"
gst-launch-1.0 udpsrc port=5000 \
! application/x-rtp, encoding-name=H264, payload=96 \
! rtph264depay \
! h264parse \
! queue \
! vaapih264dec \
! video/x-raw, width=640, height=480, framerate=30/1, format=RGB \
! videoconvert \
! vaapisink

```

![](images/hardware_video_top.png)

|  element           | CPU    |  HW     |
| ------------------ | ------ | ------- |
| 264enc             | 40-50% |         |
| 264dec + display   | 6%     | video:0 |
| 264dec + fake sink | 0.7%   |         |
| vaapih264enc             | 9%     |         |
| vaapih264dec + display   | 3%     | video:3 |



## intel

```bash
sudo apt install intel-gpu-tools
```

```bash
sudo  sudo intel_gpu_top
```

![](images/intel_gpu_top.png)

## va-api

```bash
gst-inspect-1.0 | grep vaapi
vaapi:  vaapiav1dec: VA-API AV1 decoder
vaapi:  vaapidecodebin: VA-API Decode Bin
vaapi:  vaapih264dec: VA-API H264 decoder
vaapi:  vaapih264enc: VA-API H264 encoder
vaapi:  vaapih265dec: VA-API H265 decoder
vaapi:  vaapih265enc: VA-API H265 encoder
vaapi:  vaapijpegdec: VA-API JPEG decoder
vaapi:  vaapijpegenc: VA-API JPEG encoder
vaapi:  vaapimpeg2dec: VA-API MPEG2 decoder
vaapi:  vaapipostproc: VA-API video postprocessing
vaapi:  vaapisink: VA-API sink
vaapi:  vaapivp8dec: VA-API VP8 decoder
vaapi:  vaapivp9dec: VA-API VP9 decoder
```

