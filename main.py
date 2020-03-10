from quadcopter import *
'''
THIS SHOULD RUN ON THE RASPERRY
'''

#creating a quadcopter object
quadricottero = quadcopter(4,17,27,22)

while True:
    quadricottero.balance_PID()
    #print(quadricottero.m1,quadricottero.m2,quadricottero.m3,quadricottero.m4)
    #print("\n")
    #time.sleep(0.1)

