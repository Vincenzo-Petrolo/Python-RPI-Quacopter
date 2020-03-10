def complementary_filter(angle,acc,gyro,dt,alpha):
        if (gyro == 0 and acc == 0):
                return angle
        return (alpha)*(angle + gyro*dt) + (1-alpha)*acc