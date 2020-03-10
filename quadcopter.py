from simple_pid import PID
from FILTERS import *
import os
os.system("sudo killall pigpiod")
os.system ("sudo pigpiod") #Launching GPIO library
import pigpio
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
from MPU6050 import *
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
        self.MIN    = 700                      #to define
        self.m1_pin = m1_pin
        self.m2_pin = m2_pin
        self.m3_pin = m3_pin
        self.m4_pin = m4_pin
        self.now    = time.time_ns()
        self.desidred_angle = 0
        self.pi = pigpio.pi()                   #object rpi
        
        self.time   = 0

        #PID parameters
        self.Kp     = 1
        self.Ki     = 0
        self.Kd     = 1

        self.pidX    = PID(Kp=self.Kp,Ki=self.Ki,Kd=self.Kd,setpoint=self.desidred_angle)
        self.pidY    = PID(Kp=self.Kp,Ki=self.Ki,Kd=self.Kd,setpoint=self.desidred_angle)
        
        #MPU starts

        #ESC calibration

        #self.calibrate()

    def get_elapsed_time(self):
        self.time   = self.time + pow(10,-4)
        return self.time                   #returns sampling frequency of MPU6050

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
        Ax = int(acc_x/16384.0 + 0.12890625 - 0.002197265625)
        Ay = int(acc_y/16384.0 - 0.058837890625)
        Az = int(acc_z/16384.0)

        Gx = int(gyro_x/131.0 + 0.732824427480916 - 0.007633587786259555)    #is the offset, this value is temporary
        Gy = int(gyro_y/131.0 + 0.03816793893129771)
        Gz = int(gyro_z/131.0)
        '''
        print("ACC: ")
        print(Ax,Ay,Az)
        print("\n")

        print("GYRO: ")
        print(Gx,Gy,Gz)
        print("\n")
        '''

        self.roll   = int(complementary_filter(self.roll,Ax,Gx,self.get_elapsed_time(),0.99))   #instead of 1 there should be the period 1/f where f is the frequency of the signal coming from the MPU6050
        self.pitch  = int(complementary_filter(self.pitch,Ay,Gy,self.get_elapsed_time(),0.99))   #instead of 1 there should be the period 1/f where f is the frequency of the signal coming from the MPU6050

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
        if (speed > self.MAX):
            self.m1 = self.MAX
        else:
            if (speed < self.MIN):
                self.m1 = self.MIN
            else:
                self.m1 = speed

        self.pi.set_servo_pulsewidth(self.m1_pin,self.m1)
    def set_m2_speed(self,speed):
        if (speed > self.MAX):
            self.m2 = self.MAX
        else:
            if (speed < self.MIN):
                self.m2 = self.MIN
            else:
                self.m2 = speed
        
        self.pi.set_servo_pulsewidth(self.m2_pin,self.m2)
    def set_m3_speed(self,speed):
        if (speed > self.MAX):
            self.m3 = self.MAX
        else:
            if (speed < self.MIN):
                self.m3 = self.MIN
            else:
                self.m3 = speed
        
        self.pi.set_servo_pulsewidth(self.m3_pin,self.m3)
    def set_m4_speed(self,speed):
        if (speed > self.MAX):
            self.m4 = self.MAX
        else:
            if (speed < self.MIN):
                self.m4 = self.MIN
            else:
                self.m4 = speed
                
        self.pi.set_servo_pulsewidth(self.m4_pin,self.m4)
    

    def balance_PID(self):
        vect = self.get_roll_yaw_pitch()
        pid_response_x  = int(self.pidX(vect[0]))
        pid_response_y  = int(self.pidY(vect[1]))
        
        print(vect)
        print("\n")

        '''                
        print("PIDX: " + str(pid_response_x) + "PIDY: " + str(pid_response_y))
        print("\n")
        '''

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


    def calibrate(self):   #This is the auto calibration procedure of a normal ESC
        self.pi.set_servo_pulsewidth(self.m1_pin, 0)
        print("Disconnect the battery and press Enter")
        inp = input()
        if inp == '':
            self.pi.set_servo_pulsewidth(self.m1_pin,self.MAX)
            print("Connect the battery NOW.. you will here two beeps, then wait for a gradual falling tone then press Enter")
            inp = input()
            if inp == '':            
                self.pi.set_servo_pulsewidth(self.m1_pin, self.MIN)
                print ("Wierd eh! Special tone")
                time.sleep(7)
                print ("Wait for it ....")
                time.sleep (5)
                print ("Im working on it, DONT WORRY JUST WAIT.....")
                self.pi.set_servo_pulsewidth(self.m1_pin, 0)
                time.sleep(2)
                print ("Arming ESC now...")
                self.pi.set_servo_pulsewidth(self.m1_pin, self.MIN)
                time.sleep(1)
                print ("See.... uhhhhh")
