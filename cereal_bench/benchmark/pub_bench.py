import time

import numpy as np

import cereal.messaging as messaging

pm = messaging.PubMaster(['small'])

while True:
    msg = messaging.new_message('small')
    msg.small = np.random.randint()
    pm.send('small', msg)
    time.sleep(0.1)
    