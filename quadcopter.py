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
    def __init__(self,m1_pin,m2_pin,m3_pin,m4_pin,calibrating):
        self.pitch  = 0
        self.roll   = 0
        self.yaw    = 0
        self.m1     = 0                         #speeds
        self.m2     = 0
        self.m3     = 0
        self.m4     = 0
        self.MAX    = 1300
        self.MIN    = 1100                      #to define
        self.m1_pin = m1_pin
        self.m2_pin = m2_pin
        self.m3_pin = m3_pin
        self.m4_pin = m4_pin
        self.now    = time.time_ns()
        self.desidred_angle = 0
        self.pi = pigpio.pi()                   #object rpi
        
        self.time   = 0

        #PID parameters
        self.Kp     = 0.04
        self.Ki     = 0
        self.Kd     = 0.35

        self.pidX    = PID(Kp=self.Kp,Ki=self.Ki,Kd=self.Kd,setpoint=self.desidred_angle)
        self.pidY    = PID(Kp=self.Kp,Ki=self.Ki,Kd=self.Kd,setpoint=self.desidred_angle)

        #MQTT info
        self.hostname   = "192.168.43.230"
        self.mqttc = mqtt.Client("drone")
        self.mqttc.connect(self.hostname, 1883)
        

        #MPU starts

        #ESC calibration
        if calibrating == True:
            self.calibrate()



    def get_elapsed_time(self):
        self.time   =  1/(30*pow(10,3))
        return self.time                   #returns sampling frequency of MPU6050





















#-----------------------------------------------------------------------------------------------------------------------



    def get_roll_yaw_pitch(self):           
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)
        Ax = acc_x/16384.0
        Ay = acc_y/16384.0
        Az = acc_z/16384.0
        Gx = gyro_x/131.0 
        Gy = gyro_y/131.0 
        Gz = gyro_z/131.0
        

        g = math.sqrt(math.pow(Ax,2)+math.pow(Ay,2)+math.pow(Az,2))

        roll1           = math.acos(Ay/g) * 180/math.pi  - 90 
        
        
        pitch1          = math.acos(Ax/g) * 180/math.pi - 90
        
        
        #yaw1           = -math.acos(Az/g) * 180/math.pi


        #APPLYING FILTERS
        self.roll   = (complementary_filter(self.roll,roll1,Gx,self.get_elapsed_time(),0.98))
        self.pitch  = (complementary_filter(self.pitch,pitch1,Gy,self.get_elapsed_time(),0.98))      
        
        return (self.roll,self.pitch,self.yaw)


    def balance_PID(self):
        vect = self.get_roll_yaw_pitch()

        pid_response_x  = (self.pidX(vect[0]))
        pid_response_y  = -(self.pidY(vect[1]))

        print(vect)
        print("\n")

        print("PIDX: "+str(pid_response_x))
        print("PIDY: "+str(pid_response_y))


        #X balancing
        #self.set_m1_speed(self.m1+pid_response_x)
        self.set_m3_speed(self.m3+pid_response_x)
        #the - sign is due to the postive/negative default orientation of the MPU
        self.set_m2_speed(self.m2-pid_response_x)
        #self.set_m4_speed(self.m4-pid_response_x)

        #Y balancing

        #self.set_m4_speed(self.m4+pid_response_y)
        self.set_m3_speed(self.m3+pid_response_y)
        
        #self.set_m1_speed(self.m1-pid_response_y)
        self.set_m2_speed(self.m2-pid_response_y)


#-----------------------------------------------------------------------------------------------------------------------








































































































    def calibrate(self):   #This is the auto calibration procedure of a normal ESC
        self.pi.set_servo_pulsewidth(self.m1_pin,0)
        self.pi.set_servo_pulsewidth(self.m2_pin,0)
        self.pi.set_servo_pulsewidth(self.m3_pin,0)
        self.pi.set_servo_pulsewidth(self.m4_pin,0)
        print("Disconnect the battery and press Enter")
        inp = input()
        if inp == '':
            self.pi.set_servo_pulsewidth(self.m1_pin,2000)
            self.pi.set_servo_pulsewidth(self.m2_pin,2000)
            self.pi.set_servo_pulsewidth(self.m3_pin,2000)
            self.pi.set_servo_pulsewidth(self.m4_pin,2000)
                
            print("Connect the battery NOW.. you will here two beeps, then wait for a gradual falling tone then press Enter")
            inp = input()
            if inp == '':            
                self.pi.set_servo_pulsewidth(self.m1_pin,700)
                self.pi.set_servo_pulsewidth(self.m2_pin,700)
                self.pi.set_servo_pulsewidth(self.m3_pin,700)
                self.pi.set_servo_pulsewidth(self.m4_pin,700)
                print ("Wierd eh! Special tone")
                time.sleep(7)
                print ("Wait for it ....")
                time.sleep (5)
                print ("Im working on it, DONT WORRY JUST WAIT.....")
                self.pi.set_servo_pulsewidth(self.m1_pin,0)
                self.pi.set_servo_pulsewidth(self.m2_pin,0)
                self.pi.set_servo_pulsewidth(self.m3_pin,0)
                self.pi.set_servo_pulsewidth(self.m4_pin,0)
                time.sleep(2)
                print ("Arming ESC now...")
                self.pi.set_servo_pulsewidth(self.m1_pin,700)
                self.pi.set_servo_pulsewidth(self.m2_pin,700)
                self.pi.set_servo_pulsewidth(self.m3_pin,700)
                self.pi.set_servo_pulsewidth(self.m4_pin,700)
                time.sleep(1)
                print ("See.... uhhhhh")







































































    def set_all_speed(self,speed):
        #self.set_m1_speed(speed)
        self.set_m2_speed(speed)
        self.set_m3_speed(speed)
        #self.set_m4_speed(speed)

    #MQTT FUNCTIONS

    def publish_info(self):
        '''
        the topics are of type drone/(angle,motor_speed)/([roll,pitch,yaw],[m1,m2,m3,m4])
        '''
        self.mqttc.publish("drone/angle/roll",str(self.roll))
        self.mqttc.publish("drone/angle/pitch",str(self.pitch))
        self.mqttc.publish("drone/angle/yaw",str(self.yaw))


        self.mqttc.publish("drone/motor_speed/m1",str(self.m1))
        self.mqttc.publish("drone/motor_speed/m2",str(self.m2))
        self.mqttc.publish("drone/motor_speed/m3",str(self.m3))
        self.mqttc.publish("drone/motor_speed/m4",str(self.m4))



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
        if (speed < self.MIN):
            self.m1 = self.MIN
        if (speed >= self.MIN and speed <= self.MAX):
            self.m1 = speed
        self.pi.set_servo_pulsewidth(self.m1_pin,self.m1)

    def set_m2_speed(self,speed):
        if (speed > self.MAX):
            self.m2 = self.MAX
        if (speed < self.MIN):
            self.m2 = self.MIN
        if (speed >= self.MIN and speed <= self.MAX):
            self.m2 = speed
        
        self.pi.set_servo_pulsewidth(self.m2_pin,self.m2)


    def set_m3_speed(self,speed):
        if (speed > self.MAX):
            self.m3 = self.MAX
        if (speed < self.MIN):
            self.m3 = self.MIN
        if (speed >= self.MIN and speed <= self.MAX):
            self.m3 = speed
        
        self.pi.set_servo_pulsewidth(self.m3_pin,self.m3)


    def set_m4_speed(self,speed):
        if (speed > self.MAX):
            self.m4 = self.MAX
        if (speed < self.MIN):
            self.m4 = self.MIN
        if (speed >= self.MIN and speed <= self.MAX):
            self.m4 = speed
        
        self.pi.set_servo_pulsewidth(self.m4_pin,self.m4)
    