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