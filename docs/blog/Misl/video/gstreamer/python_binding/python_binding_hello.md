---
title: Gstreamer python binding
description: Gstreamer python binding
date: "2022-18-05"
banner: gstreamer.png
tags:
    - video
    - gstreamer
    - gi
---

# Install
Gstreamer python bindings implement by `PyGObject`

!!! Note
    PyGObject is a Python package which provides bindings for GObject based libraries such as GTK, GStreamer and many more.

```bash title="install" linenums="1" hl_lines="3 4 9"
# Install mark library installed other dependencies
sudo apt install \
libgirepository1.0-dev \
libcairo2-dev \
python3-dev \
gir1.2-gtk-3.0

pip3 install pycairo
pip3 install PyGObject
```

[PyGObject site ubuntu Install instruction](https://pygobject.readthedocs.io/en/latest/getting_started.html#ubuntu-logo-ubuntu-debian-logo-debian)

---

# Simple example
[example from gstreamer site](https://gstreamer.freedesktop.org/documentation/tutorials/basic/hello-world.html?gi-language=python)


```python title="gst" linenums="1" hl_lines="11"
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject, GLib

Gst.init(None)

pipeline = Gst.parse_launch("videotestsrc name=src ! autovideosink")
pipeline.set_state(Gst.State.PLAYING)

# wait until EOS or error
bus = pipeline.get_bus()
msg = bus.timed_pop_filtered(
    Gst.CLOCK_TIME_NONE,
    Gst.MessageType.ERROR | Gst.MessageType.EOS
)

# free resources
pipeline.set_state(Gst.State.NULL)
```


---

