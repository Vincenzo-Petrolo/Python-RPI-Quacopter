from quadcopter import *
'''
THIS SHOULD RUN ON THE RASPERRY
'''

#creating a quadcopter object
quadricottero = quadcopter(4,17,27,22,True)
#quadricottero.set_all_speed(1300)
while True:
    quadricottero.balance_PID()    
    #quadricottero.publish_info()
    print(quadricottero.m1,quadricottero.m2,quadricottero.m3,quadricottero.m4)
    #print("\n")
    #time.sleep(0.1)

