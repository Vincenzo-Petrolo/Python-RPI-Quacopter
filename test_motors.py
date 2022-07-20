from quadcopter import *
import time
'''
    Testing motors availability before launch
'''
quadricottero = quadcopter(4,27,17,22,False)
motors = quadricottero.get_motors()
for motor in motors:
        for v in range(1000,1400):
            motor.set_speed(v,True)
            quadricottero.publish_info()
            time.sleep(0.1)
        motor.slow_stop()
quadricottero.stop()
