from gz.msgs.stringmsg_pb2 import StringMsg

from gz.transport import AdvertiseMessageOptions
from gz.transport import Node

# Create a transport node
node = Node()

# Advertise a topic
topic = "/foo"
msg_type_name = StringMsg.DESCRIPTOR.full_name
pub_options = AdvertiseMessageOptions()
pub = node.advertise(topic, msg_type_name, pub_options)

# Publish a message
msg = StringMsg()
msg.data = "hello"
pub.publish(msg)