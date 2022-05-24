import sys
import numpy as np
import traceback
import time
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initializes Gstreamer, it's variables, paths
Gst.init(sys.argv)

PIPELINE = "appsrc name=app_src ! video/x-raw,width=640,height=480,format=BGR,framerate=10/1 ! videoconvert ! autovideosink"

def ndarray_to_gst_buffer(array: np.ndarray) -> Gst.Buffer:
    """Converts numpy array to Gst.Buffer"""
    return Gst.Buffer.new_wrapped(array.tobytes())


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
appsource = pipeline.get_by_name("app_src")
bus = pipeline.get_bus()
# allow bus to emit messages to main thread
bus.add_signal_watch()
# Start pipeline
pipeline.set_state(Gst.State.PLAYING)
# Init GObject loop to handle Gstreamer Bus Events
loop = GLib.MainLoop()

# Add handler to specific signal
bus.connect("message", on_message, loop)

# Push buffer and check
for _ in range(10):
    arr = np.random.randint(low=0,high=255,size=(480,640,3),dtype=np.uint8)
    appsource.emit("push-buffer", ndarray_to_gst_buffer(arr))
    time.sleep(1/10)
appsource.emit("end-of-stream")

try:
    loop.run()
except Exception:
    traceback.print_exc()
    loop.quit()

# Stop Pipeline
pipeline.set_state(Gst.State.NULL)