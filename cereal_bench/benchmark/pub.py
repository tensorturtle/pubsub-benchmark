import cereal.messaging as messaging

pm = messaging.PubMaster(['carState'])


i = 0
while True:
    msg = messaging.new_message('carState')
    msg.carState.rightBlinker = True
    msg.carState.leftBlinker = False
    pm.send('carState', msg)