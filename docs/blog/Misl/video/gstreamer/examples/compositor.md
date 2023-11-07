---
tags:
    - gstreamer
    - pipe
    - compositor
---

# Compositor 

- "xpos": The x-coordinate position of the top-left corner of the picture 
- "ypos": The y-coordinate position of the top-left corner of the picture 
- "width": The width of the picture; the input will be scaled if necessary 
- "height": The height of the picture; the input will be scaled if necessary 
- "alpha": The transparency of the picture; between 0.0 and 1.0. 
- "zorder": The z-order position of the picture in the composition

```bash
gst-launch-1.0 \
compositor name=m \
    sink_1::xpos=50 sink_1::ypos=50 sink_1::width=150\
    sink_2::xpos=400 sink_2::ypos=50  \
! videoconvert \
! autovideosink \
videotestsrc pattern=white \
! video/x-raw, format=I420, framerate=5/1, width=740, height=480 \
! m. \
videotestsrc pattern=red \
! video/x-raw, format=I420, framerate=5/1, width=300, height=200 \
! queue \
! m.sink_2 \
videotestsrc pattern=green \
! video/x-raw, format=I420, framerate=5/1, width=300, height=200 \
! queue \
! m.sink_1
```

![](images/compositor_demo.png)

---

## Reference
- [gstreamer](https://gstreamer.freedesktop.org/documentation/compositor/index.html?gi-language=c)