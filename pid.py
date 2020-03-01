'''
Designed to adjust motor speed from gyroscope datas

CONFIG ONE                     CONFIG TWO (best)
1         3                         1
 \       /                          |
  \     /                           |
   \   /                            |
    \ /                             |
    / \                 2-----------------------3       
   /   \                            |
  /     \                           |
 /       \                          | 
2         4                         4
'''

from simple_pid import PID


class PID_roll_controller():
    '''
    this class manages only the roll of the quadcopter, basing on the configuration chosen it returns the speeds of the motors
    '''
    def __init__(self,Ki,Kd,Kp,speed,interval,inf_limit,sup_limit):
        self.pid            = PID(Kp,Ki,Kd,setpoint=speed,sample_time=interval,output_limits=(inf_limit,sup_limit),auto_mode=True)

    '''
    to define: functions that interact with given motor depending on the orientation of the sensor that stands on the drone
    '''


class PID_yaw_controller():
    '''
    this class manages only the yaw of the quadcopter, basing on the configuration chosen it returns the speeds of the motors
    '''
    def __init__(self,Ki,Kd,Kp,speed,interval,inf_limit,sup_limit):
        self.pid            = PID(Kp,Ki,Kd,setpoint=speed,sample_time=interval,output_limits=(inf_limit,sup_limit),auto_mode=True)

    '''
    to define: functions that interact with given motor depending on the orientation of the sensor that stands on the drone
    '''


class PID_pitch_controller():
    '''
    this class manages only the pitch of the quadcopter, basing on the configuration chosen it returns the speeds of the motors
    '''
    def __init__(self,Ki,Kd,Kp,speed,interval,inf_limit,sup_limit):
        self.pid            = PID(Kp,Ki,Kd,setpoint=speed,sample_time=interval,output_limits=(inf_limit,sup_limit),auto_mode=True)

    '''
    to define: functions that interact with given motor depending on the orientation of the sensor that stands on the drone
    '''