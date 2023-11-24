---
tags:
    - gstreamer
    - tee
    - queue
---



```bash title="minimal tee"
gst-launch-1.0 videotestsrc \
! tee name=tp \
tp. ! queue ! videoconvert ! autovideosink \
tp. ! queue ! videoconvert ! fpsdisplaysink
```

---

## Work example tee and udpsink
```bash
gst-launch-1.0 videotestsrc \
! tee name=tp \
tp. ! queue ! videoconvert ! jpegenc ! rtpjpegpay ! udpsink host=127.0.0.1 port=5000 \
tp. ! queue ! videoconvert ! fpsdisplaysink
```

```bash
gst-launch-1.0 udpsrc port=5000 \
! application/x-rtp, encoding-name=JPEG,payload=26 \
! rtpjpegdepay \
! jpegdec \
! autovideosink
```

---


## example with comp and tee one branch over udp 
```
src1 ->                 autovideosink
        compositor ->
src2 ->                 jpegenc -> rtp -> udpsink
```

```bash
gst-launch-1.0 videotestsrc \
! videoconvert \
! compositor name=comp \
sink_0::xpos=0 sink_0::ypos=0 \
sink_1::xpos=100 sink_1::ypos=100 \
! videoconvert \
! tee name=tp \
tp. ! queue ! videoconvert ! autovideosink \
tp. ! queue ! video/x-raw, width=420, height=340, framerate=10/1, format=I420 \
! jpegenc \
! rtpjpegpay \
! udpsink host=127.0.0.1 port=5000 \
videotestsrc pattern=ball \
! videoconvert \
! comp.
```

```bash
gst-launch-1.0 udpsrc port=5000 \
! application/x-rtp, encoding-name=JPEG,payload=26 \
! rtpjpegdepay \
! jpegdec \
! autovideosink
```

---

## tee only select stream before compositor
- src1 -> comp
- src2 -> tee -> comp
            - -> jpeg -> udp
  
```bash
gst-launch-1.0 videotestsrc \
! videoconvert \
! compositor name=comp \
sink_0::xpos=0 sink_0::ypos=0 \
sink_1::xpos=100 sink_1::ypos=100 \
! videoconvert \
! autovideosink \
videotestsrc pattern=ball \
! videoconvert name="ball_convert" \
! tee name=tp \
tp. ! queue ! videoconvert ! comp. \
tp. ! queue ! video/x-raw, width=420, height=340, framerate=10/1, format=I420 \
! jpegenc \
! rtpjpegpay \
! udpsink host=127.0.0.1 port=5000 
```


---

```bash 
gst-launch-1.0 videotestsrc \
! videoconvert \
! compositor name=comp \
sink_0::xpos=0 sink_0::ypos=0 \
sink_1::xpos=320 sink_1::ypos=0 \
! videoconvert \
! autovideosink \
videotestsrc pattern=ball \
! videoconvert \
! comp.
```


gst-launch-1.0 udpsrc port=5000 \
! application/x-rtp, encoding-name=JPEG,payload=26 \
! rtpjpegdepay \
! jpegdec \
! autovideosink

```bash 
gst-launch-1.0 \
compositor name=m \
    sink_1::xpos=50 sink_1::ypos=50 \
    sink_2::xpos=400 sink_2::ypos=50  \
! videoconvert \
! autovideosink \
videotestsrc pattern=black \
! video/x-raw, format=I420, framerate=5/1, width=740, height=480 \
! m. \
udpsrc port=5001 \
! application/x-rtp, encoding-name=JPEG,payload=26 \
! rtpjpegdepay \
! jpegdec \
! video/x-raw, format=I420, \
! queue \
! m.sink_2 \
udpsrc port=5000 \
! application/x-rtp, encoding-name=JPEG,payload=26 \
! rtpjpegdepay \
! jpegdec \
! video/x-raw, format=I420, \
! queue \
! m.sink_1
```

```bash
gst-launch-1.0 videotestsrc \
! videoconvert name="ball_convert" \
! jpegenc \
! rtpjpegpay \
! udpsink host=127.0.0.1 port=5000 
```

```bash
gst-launch-1.0 videotestsrc  pattern=ball \
! videoconvert name="ball_convert" \
! jpegenc \
! rtpjpegpay \
! udpsink host=127.0.0.1 port=5001 
```