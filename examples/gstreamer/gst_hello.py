import sys
import traceback
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initializes Gstreamer, it's variables, paths
Gst.init(sys.argv)

PIPELINE = "videotestsrc num-buffers=100 ! autovideosink"

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

try:
    loop.run()
except Exception:
    traceback.print_exc()
    loop.quit()

# Stop Pipeline
pipeline.set_state(Gst.State.NULL)