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

import motor
rad_to_deg = 180/3.141592654


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("#")
 


class quadcopter():
 

    

    def __init__(self,m1_pin,m2_pin,m3_pin,m4_pin,calibrating):
        '''
            Angles of the quadcopter
        '''
        self.pitch  = 0
        self.roll   = 0
        self.yaw    = 0
        '''
            Constants for the quadcopter
            MIN : is the minimum speed for each of the quadcopter motors
            PITCH_DES_ANGLE : is the upper limit for the speed
            ROLL_DES_ANGLE  : the angle of pitch desired, is is 0 degrees in idle
           HOSTNAME : the broker to witch connect 
        '''
        self.MIN    = 1100
        self.MAX    = 1300
        self.PITCH_DES_ANGLE = 0
        self.ROLL_DES_ANGLE  = 0
        self.HOSTNAME         = "localhost"
        '''
            Creating the pi object for interactions with motors
        '''
        self.pi = pigpio.pi()                   #object rpi
        
        '''
            The quadcopter is composed by 4 motors
        '''
        self.motors  = [motor(m1_pin,self.MIN,self.MAX,self.pi),
                        motor(m2_pin,self.MIN,self.MAX,self.pi),
                        motor(m3_pin,self.MIN,self.MAX,self.pi),
                        motor(m4_pin,self.MIN,self.MAX,self.pi)
                        ]
  
        '''
            The quadcopter balancing is done via a PID controller
            The values of the PID, might be different for the ROLL and PITCH angles
        '''
        #ROLL
        self.KpR     = 0.04
        self.KiR     = 0
        self.KdR     = 0
        self.pidX    = PID(Kp=self.KpR,Ki=self.KiR,Kd=self.KdR,setpoint=self.ROLL_DES_ANGLE)
        #PITCH
        self.KpP     = 0.04
        self.KiP     = 0
        self.KdP     = 0
        self.pidY    = PID(Kp=self.Kp,Ki=self.Ki,Kd=self.Kd,setpoint=self.PITCH_DES_ANGLE)


        '''
            MQTT is used as standar communication protocol between the android app and the quadcopter. It is not a big deal, but the simplest protocol i know, and for this reason the one i will use for the moment.
        '''
        self.mqttc = mqtt.Client()
        self.mqttc.on_connect = on_connect
        self.mqttc.on_message = self.on_message
        self.mqttc.connect(self.HOSTNAME)
        self.mqttc.loop_start()
       
        '''
            If one wants to calibrate it, during the creation the calibrate method is called, hence calibrating the motors
        '''
        if calibrating == True:
            self.calibrate()


    '''
        The on_message method is used to handle MQTT messages. When a message is received, the method performs checks on the TOPICS, for example if the topic is STOP, then the quadcopter will procede to the STOP method
    '''

    def on_message(self,client, userdata, msg):
        stringa = str(msg.payload).replace("b'",'')
        stringa = str(stringa).replace("'",'')

        '''
        if ("left" in msg.topic) == True:
            vector  = str(stringa).split(" ")

            if module == 100:
                if phase > 315 and phase <= 45:
                    #m1 and m4 +10, m2 and m3 -10
                    self.set_m1_speed(self.m1 + 10)
                    self.set_m4_speed(self.m4 + 10)
                    self.set_m2_speed(self.m2 - 10)
                    self.set_m3_speed(self.m3 - 10)
                if phase > 45 and pahse <= 135:
                    #all motors +10
                    self.set_m1_speed(self.m1 + 10)
                    self.set_m4_speed(self.m4 + 10)
                    self.set_m2_speed(self.m2 + 10)
                    self.set_m3_speed(self.m3 + 10)           

                if phase > 135 and phase <= 225:
                    #m2 and m3 +10, m1 and m4 -10
                    self.set_m1_speed(self.m1 - 10)
                    self.set_m4_speed(self.m4 - 10)
                    self.set_m2_speed(self.m2 + 10)
                    self.set_m3_speed(self.m3 + 10)

                if phase > 225 and phase <= 315:
                    #all motors -10        
                    self.set_m1_speed(self.m1 - 10)
                    self.set_m4_speed(self.m4 - 10)
                    self.set_m2_speed(self.m2 - 10)
                    self.set_m3_speed(self.m3 - 10)
            if module == 0:
                self.set_all_speed((self.MAX+self.MIN)/2)

        if "right" in str(msg.topic):
            vector  = str(msg.payload).split(' ')
            phase   = int(vector[0])
            module  = int(vector[1])

            if module == 100:
                if phase > 315 and phase <= 45:
                    #set desired roll to -20 degrees
                    self.roll_desired_angle = -20
                if phase > 45 and pahse <= 135:
                    #set desired pitch to 20 degrees
                    self.pitch_desired_angle = 20
                if phase > 135 and phase <= 225:
                    #set desired roll to 20 degrees
                    self.roll_desired_angle = 20
                if phase > 225 and phase <= 315:
                    #set desired pitch to -20 degrees        
                    self.pitch_desired_angle = -20
            if module == 0:
                self.pitch_desired_angle    = 0
                self.roll_desired_angle     = 0
            '''

    '''
        The get_roll_yaw_pitch method, returns an array containing
                    [ ROLL , PITCH , YAW ]
        Notes on the achieving of the angles: The angles, expecially for PITCH,ROLL
        are achieved from the MPU6050 and in order to correct some imperfection 
        a complementary filter is applied.
    '''
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

    '''
        The balance_PID method, is used to balance the quadcopter motors.
        They are balanced by adding or subtracting to the speeds of the motors
        the results of the PID[ROLL|PITCH].
    '''


    def balance_PID(self):
        vect = self.get_roll_yaw_pitch()
        '''
            In these two lines the error is calculated by the difference of the 
            actual angle - the desidred_angle.
        '''
        pid_response_R  = (self.pidR( vect[0] - self.ROLL_DES_ANGLE))
        pid_response_P  = -(self.pidP( vect[1] - self.PITCH_DES_ANGLE))
        '''
        print(vect)
        print("\n")

        print("PIDX: "+str(pid_response_x))
        print("PIDY: "+str(pid_response_y))
        '''

        '''
            after the response is calculated we change the speeds of the motors
            according to that response.
        '''
        #ROLL
        self.motors[0].set_speed(self.motor_1.get_speed() + pid_response_R,False)
        self.motors[1].set_speed(self.motor_2.get_speed() - pid_response_R,False)
        #PITCH
        self.motors[2].set_speed(self.motor_3.get_speed() + pid_response_P,False)
        self.motors[3].set_speed(self.motor_4.get_speed() - pid_response_P,False)


    '''
        The calibrate() method calibrates the motors following a procedure found on
        the internet.
    '''
    def calibrate(self):   #This is the auto calibration procedure of a normal ESC
        for motor in self.motors:
            motor.set_speed(0,True)

        print("Disconnect the battery and press Enter")
        inp = input()
        if inp == '':
            for motor in self.motors:
                motor.set_speed(2000,True)
            print("Connect the battery NOW.. you will here two beeps, then wait for a gradual falling tone then press Enter")
            inp = input()
            if inp == '':            
                for motor in self.motors:
                    motor.set_speed(700,True)
                print ("Wierd eh! Special tone")
                time.sleep(7)
                print ("Wait for it ....")
                time.sleep (5)
                print ("Im working on it, DONT WORRY JUST WAIT.....")
                for motor in self.motors:
                    motor.set_speed(0,True)
                time.sleep(2)
                print ("Arming ESC now...")
                for motor in self.motors:
                    motor.set_speed(700,True)
                time.sleep(1)
                print ("See.... uhhhhh")

    '''
        This method is used to change all the speeds of the motors
    '''
    def set_all_speed(self,speed):
        for motor in self.motors:
            motor.set_speed(speed,False)

    '''
        This method is used to publish telemetry data to MQTT
    '''
    def publish_info(self):
        '''
        the topics are of type drone/(angle,motor_speed)/([roll,pitch,yaw],[m1,m2,m3,m4])
        '''
        self.mqttc.publish("drone/angle/roll",str(self.roll))
        self.mqttc.publish("drone/angle/pitch",str(self.pitch))
        self.mqttc.publish("drone/angle/yaw",str(self.yaw))

        string = "drone/motor_speed/motor_"
        i = 0
        for motor in self.motors:
            i += 1
            topic = string + i
            self.mqttc.publish(topic,str(motor.get_speed()))
            
