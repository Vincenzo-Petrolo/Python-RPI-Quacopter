#class for motors

class motor():
    def __init__(self,pin,MIN,MAX,pi):
        self.pin = pin
        self.MIN = MIN
        self.MAX = MAX
        self.speed = 0
        self.pi  = pi #used to interact with the raspberry pi
    
    def set_speed(self,speed,no_limits):
        if (no_limits is False):
            if (speed > self.MAX):
                self.speed = self.MAX
            elif (speed < self.MIN):
                self.speed = self.MIN
            else:
                self.speed = speed
        else:
            self.speed = speed

    def slow_stop(self):
        while (self.speed >1000):
            self.speed -= 10
            self.set_speed(self.speed,True)

    def hard_stop(self):
        self.set_speed(900,True)

    def get_speed(self):
        return self.speed
