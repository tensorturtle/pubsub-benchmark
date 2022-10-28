from cereal import log
from cereal.services import service_list
import cereal.messaging as messaging

events = [evt for evt in log.Event.schema.union_fields if evt in service_list.keys()]

print("Events: {}".format(events))

small_sock = events[0]

sub_small_sock = messaging.sub_sock(small_sock, conflate=False, timeout=None)

while True:
    received = sub_small_sock.receive()
    print("Received: {}".format(received))


print("HI")