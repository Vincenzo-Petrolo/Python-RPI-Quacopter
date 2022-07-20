import math


class FILTER(object):
        def __init__(self) -> None:
                self.last_number_inputs = 10

                self.queue = [0] * self.last_number_inputs

                self.current_samples = 0
                

        def complementary_filter(self,angle,acc,gyro,dt,alpha):
                return ((alpha)*(angle + gyro*dt) + (1-alpha)*acc)#*(1/(1-alpha*dt))

        def compute_average(self,queue, n_samples):
                total = 0

                for sample in queue:
                        total += sample

                return total/self.last_number_inputs

        def history_filter(self,angle,acc,gyro,dt,alpha):
                #print(self.queue)
                instant_angle = self.complementary_filter(angle,acc,gyro,dt,alpha)

                # Mod last_number_inputs queue
                self.queue[self.current_samples] = instant_angle
                # Increase the value
                self.current_samples = (self.current_samples + 1) % self.last_number_inputs


                return self.compute_average(self.queue, self.last_number_inputs)

                