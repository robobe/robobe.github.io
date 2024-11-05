---
tags:
    - gstreamer
    - rtsp
    - python
---

# Create RTSP server using gstreamer

## install
```bash
sudo apt-get install libglib2.0-dev libgstrtspserver-1.0-dev gstreamer1.0-rtsp
```

## code
python simple example

```python
import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GObject, GLib

loop = GLib.MainLoop()
Gst.init(None)

class TestRtspMediaFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self):
        GstRtspServer.RTSPMediaFactory.__init__(self)

    def do_create_element(self, url):
        s_src = "videotestsrc ! video/x-raw,rate=30,width=320,height=240,format=I420"
        s_h264 = "x264enc tune=zerolatency"
        pipeline_str = f"( {s_src} ! queue max-size-buffers=1 name=q_enc ! {s_h264} ! rtph264pay name=pay0 pt=96 )"
        return Gst.parse_launch(pipeline_str)


class GstreamerRtspServer():
    def __init__(self):
        self.rtspServer = GstRtspServer.RTSPServer()
        factory = TestRtspMediaFactory()
        factory.set_shared(True)
        mountPoints = self.rtspServer.get_mount_points()
        mountPoints.add_factory("/stream1", factory)
        self.rtspServer.attach(None)

if __name__ == '__main__':
    s = GstreamerRtspServer()
    loop.run()
```

## usage

```bash
gst-launch-1.0 playbin uri=rtsp://127.0.0.1:8554/stream1
```

or

```bash
gst-launch-1.0 -v rtspsrc do-rtcp=TRUE location=rtspt://127.0.0.1:8554/stream1 ! application/x-rtp, media=video, encoding-name=H264      ! queue     ! rtph264depay     ! h264parse     ! avdec_h264     ! videoconvert ! autovideosink
```