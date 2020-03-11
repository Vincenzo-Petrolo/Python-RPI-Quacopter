from quadcopter import *
'''
THIS SHOULD RUN ON THE RASPERRY
'''

#creating a quadcopter object
quadricottero = quadcopter(4,17,27,22,True)
#quadricottero.set_all_speed(1300)
i = 800
while i != 500:
    i -= 20
    print(i)
    quadricottero.set_all_speed(i)
    time.sleep(0.5)

while i != 1300:
    i += 20
    print(i)
    quadricottero.set_all_speed(i)
    time.sleep(0.5)


while True:
    quadricottero.balance_PID()    
    #quadricottero.publish_info()
    print(quadricottero.m1,quadricottero.m4)
    #print("\n")
    #time.sleep(0.1)

quadricottero.set_all_speed(0)
