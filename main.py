from quadcopter import *
'''
THIS SHOULD RUN ON THE RASPERRY
'''

#creating a quadcopter object
quadricottero = quadcopter(4,27,17,22,False)
#quadricottero.set_all_speed(1300)

while True:
    quadricottero.balance_PID()    
    quadricottero.publish_info()
