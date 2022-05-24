import sys
import numpy as np
import traceback
import time
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initializes Gstreamer, it's variables, paths
Gst.init(sys.argv)

PIPELINE = """videotestsrc num-buffers=100 \
    ! video/x-raw, width=320, height=240 \
    ! queue  \
    ! video/x-raw, format=BGR
    ! appsink name=sink sync=true  max-buffers=1 drop=true  emit-signals=true"""

def on_new_sample(app_sink):
    sample = app_sink.emit('pull-sample')
    caps = sample.get_caps()

    # Extract the width and height info from the sample's caps
    height = caps.get_structure(0).get_value("height")
    width = caps.get_structure(0).get_value("width")

    buf = sample.get_buffer()
    array = np.ndarray((height, width, 3),
            buffer=buf.extract_dup(0, buf.get_size()), 
            dtype=np.uint8)
    print(array.shape)
    return Gst.FlowReturn.OK


def on_message(bus: Gst.Bus, message: Gst.Message, loop: GLib.MainLoop):
    mtype = message.type
    if mtype == Gst.MessageType.EOS:
        print("End of stream")
        loop.quit()

    elif mtype == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print(err, debug)
        loop.quit()

    elif mtype == Gst.MessageType.WARNING:
        err, debug = message.parse_warning()
        print(err, debug)

    return True

pipeline = Gst.parse_launch(PIPELINE)
bus = pipeline.get_bus()
# allow bus to emit messages to main thread
bus.add_signal_watch()
# Start pipeline
pipeline.set_state(Gst.State.PLAYING)
# Init GObject loop to handle Gstreamer Bus Events
loop = GLib.MainLoop()

# Add handler to specific signal
bus.connect("message", on_message, loop)

appsink = pipeline.get_by_name("sink")
handler_id = appsink.connect("new-sample", on_new_sample)

try:
    loop.run()
except Exception:
    traceback.print_exc()
    loop.quit()

# Stop Pipeline
pipeline.set_state(Gst.State.NULL)