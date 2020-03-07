from simple_pid import PID
import pigpio #importing GPIO library
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import MPU6050
import time
import math

rad_to_deg = 180/3.141592654

class quadcopter():
    def __init__(self,m1_pin,m2_pin,m3_pin,m4_pin):
        self.pitch  = 0
        self.roll   = 0
        self.yaw    = 0
        self.m1     = 0                         #speeds
        self.m2     = 0
        self.m3     = 0
        self.m4     = 0
        self.MAX    = 2000
        self.MIN    = 1000                      #to define
        self.m1_pin = m1_pin
        self.m2_pin = m2_pin
        self.m3_pin = m3_pin
        self.m4_pin = m4_pin
        self.now    = time.time_ns()
        self.desidred_angle = 0
        self.pi = pigpio.pi()                   #object rpi
        
        #PID parameters
        self.Kp     = 1
        self.Ki     = 0
        self.Kd     = 1

        self.pidX    = PID(Kp=self.Kp,Ki=self.Ki,Kd=self.Kd,setpoint=self.desidred_angle)
        self.pidY    = PID(Kp=self.Kp,Ki=self.Ki,Kd=self.Kd,setpoint=self.desidred_angle)
        
        #MPU starts
        MPU6050.MPU_Init()

    def get_elapsed_time(self):
        before     = self.now
        self.now   = time.time_ns()
        return (self.now - before)

    def get_roll_yaw_pitch(self):           #yaw doesnt get calc
    #Read Accelerometer raw value
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)

        #Read Gyroscope raw value
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)

        #Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ax = acc_x/16384.0
        Ay = acc_y/16384.0
        Az = acc_z/16384.0

        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0

        acc_x_angle = math.atan(Ay)/math.sqrt(math.pow(Ax,2) + math.pow(Az,2))*rad_to_deg
        acc_y_angle = math.atan(Ax)/math.sqrt(math.pow(Ay,2) + math.pow(Az,2))*rad_to_deg

        self.roll = 0.98 * (self.roll + Gx*self.get_elapsed_time()) + 0.02*acc_x_angle
        self.pitch = 0.98 * (self.pitch + Gy*self.get_elapsed_time()) + 0.02*acc_y_angle

        return (self.roll,self.pitch,self.yaw)
    
    def get_error(self):
        vect    = self.get_roll_yaw_pitch()
        errorX  = vect[0] - self.desidred_angle
        errorY  = vect[1] - self.desidred_angle

        return (errorX,errorY)
    
    def is_stable(self):
        vect    = self.get_roll_yaw_pitch
        #depending on roll,pitch.yaw values, if they are approximately close to 0 then return True else False
        if vect[0] < 1 and vect[1] < 1:
            return True
        return False
    
    def set_m1_speed(self,speed):
        self.m1 = speed
        self.pi.set_servo_pulsewidth(self.m1_pin,self.m1)
    def set_m2_speed(self,speed):
        self.m2 = speed
        self.pi.set_servo_pulsewidth(self.m2_pin,self.m2)
    def set_m3_speed(self,speed):
        self.m3 = speed
        self.pi.set_servo_pulsewidth(self.m3_pin,self.m3)
    def set_m4_speed(self,speed):
        self.m4 = speed
        self.pi.set_servo_pulsewidth(self.m4_pin,self.m4)
    

    def balance_PID(self):
        vect = self.get_roll_yaw_pitch()

        pid_response_x  = self.pidX(vect[0])
        pid_response_y  = self.pidY(vect[1])

        #X balancing
        self.set_m1_speed(self.m1+pid_response_x)
        self.set_m4_speed(self.m4+pid_response_x)
        #the - sign is due to the postive/negative default orientation of the MPU
        self.set_m2_speed(self.m2-pid_response_x)
        self.set_m3_speed(self.m3-pid_response_x)

        #Y balancing
        self.set_m4_speed(self.m4+pid_response_y)
        self.set_m3_speed(self.m3+pid_response_y)
        
        self.set_m1_speed(self.m1-pid_response_y)
        self.set_m2_speed(self.m2-pid_response_y)


