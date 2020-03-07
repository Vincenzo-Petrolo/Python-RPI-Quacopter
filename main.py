from quadcopter import *
'''
THIS SHOULD RUN ON THE RASPERRY
'''

#creating a quadcopter object
quadricottero = quadcopter(4,17,27,22)

while True:
    quadricottero.balance_PID()
