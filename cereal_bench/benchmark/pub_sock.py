from cereal import log
from cereal.services import service_list
import cereal.messaging as messaging

events = [evt for evt in log.Event.schema.union_fields if evt in service_list.keys()]

print("Events: {}".format(events))

small_sock = events[0]

pub_small_sock = messaging.pub_sock(small_sock)

# run this script as preamble to an interative python session:
# python3 -i sub_sock.py
# then run this in the interactive session:
# >>> msg = b'hello'
# >>> pub_small_sock.send(msg)

# In another terminal (on the same machine), run:
# python3 sub_sock.py 
# to receive the message
