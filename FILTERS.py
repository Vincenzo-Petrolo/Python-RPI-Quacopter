import math
def complementary_filter(angle,acc,gyro,dt,alpha):
        return ((alpha)*(angle + gyro*dt) + (1-alpha)*acc)*(1/(1-alpha*dt))