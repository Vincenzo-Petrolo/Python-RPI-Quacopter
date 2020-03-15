from quadcopter import *
'''
THIS SHOULD RUN ON THE RASPERRY
'''

#creating a quadcopter object
quadricottero = quadcopter(4,17,27,22,False)

quadricottero.set_m1_speed(700)
quadricottero.set_m2_speed(700)
quadricottero.set_m3_speed(700)
quadricottero.set_m4_speed(700)
    