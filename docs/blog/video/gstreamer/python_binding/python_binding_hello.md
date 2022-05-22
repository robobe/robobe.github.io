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

```bash title="install" linenums="1" hl_lines="3 8"
# Install only mark library
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

```python title="gst" linenums="1" hl_lines="2"
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import sys
import traceback

Gst.init(sys.argv)
command = "videotestsrc ! autovideosink"
pipeline = Gst.parse_launch(command)
pipeline.set_state(Gst.State.PLAYING)
loop = GLib.MainLoop()
 
try:
    loop.run()
except:
    traceback.print_exc()
```

---

# Reference
- [demo to check](https://ittone.ma/ittone/python-how-to-create-gstreamer-rtsp-server-with-variable-frame-rate/)