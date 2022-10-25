import time 

import cereal.messaging as messaging

s_to_ms = lambda s: round(s * 1000, 3)

sm = messaging.SubMaster(['carState'])

while True:
    t1 = time.time()
    sm.update(0)
    t2 = time.time()
    print(f"Updating submaster: {s_to_ms(t2 - t1)}ms")

    t3 = time.time()
    print(sm['carState'])
    t4 = time.time()
    print(f"Printing carState: {s_to_ms(t4 - t3)}ms")
