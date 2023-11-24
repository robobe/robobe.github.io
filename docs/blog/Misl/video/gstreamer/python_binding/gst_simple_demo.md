---
tags:
    - gstreamer
    - gi
    - python
    - 
---

# Change property example
- Change video source every 2 sec


!!! tip 
     using `GLib.idle_add` method


```python
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject, GLib
from threading import Thread
from time import sleep

Gst.init(None)
loop = GLib.MainLoop()

def seq():
    counter = 1
    while True:
        yield counter
        counter += 1
        if counter > 20:
            counter = 1

pipeline = Gst.parse_launch("videotestsrc name=src ! autovideosink")
pipeline.set_state(Gst.State.PLAYING)
index = 1

def change_pattern(pattern_id):
    video_source = pipeline.get_by_name("src")
    video_source.set_property("pattern", pattern_id)


def seq_handler():
    counter = seq()
    while True:
        GLib.idle_add(change_pattern, next(counter))
        sleep(2)



t = Thread(target=seq_handler, daemon=True)
t.start()
try:
    loop.run()
except KeyboardInterrupt:
    print()

# free resources
pipeline.set_state(Gst.State.NULL)

```
