
class quadcopter():
    def __init__(self,m1_pin,m2_pin,m3_pin,m4_pin):
        self.pitch  = 0
        self.roll   = 0
        self.yaw    = 0
        self.m1     = 0
        self.m2     = 0
        self.m3     = 0
        self.m4     = 0
        self.m1_pin = m1_pin
        self.m2_pin = m2_pin
        self.m3_pin = m3_pin
        self.m4_pin = m4_pin

    def get_roll(self):
        #function that calculates roll
        return self.roll
    
    def get_pitch(self):
        #function that calculates pitch
        return self.pitch
    
    def get_yaw(self):
        #function that calculates yaw
        return self.yaw
    
    def is_stable(self):
        #depending on roll,pitch.yaw values, if they are approximately close to 0 then return True else False
        if self.get_pitch() < 0.05 and self.get_roll() < 0.05 and self.get_yaw() < 0.05:
            return True
        return False
    
    def set_m1_speed(self,speed):
        self.m1 = speed
        #function that set that speed
    def set_m2_speed(self,speed):
        self.m2 = speed
        #function that set that speed
    def set_m3_speed(self,speed):
        self.m3 = speed
        #function that set that speed
    def set_m4_speed(self,speed):
        self.m4 = speed
        #function that set that speed
    
    