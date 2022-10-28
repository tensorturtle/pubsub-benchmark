import time

import numpy as np

import cereal.messaging as messaging

pm = messaging.PubMaster(['carParams','carState'])


msg = messaging.new_message('carParams')
msg.carParams.carName = "COMMA BODY"
msg.carParams.notCar = True
pm.send('carParams', msg)

while True:
    msg = messaging.new_message('carState')
    msg.carState.vEgo = np.random.random()
    msg.carState.vEgoCluster = np.random.random()
    pm.send('carState', msg)
    time.sleep(0.1)
    