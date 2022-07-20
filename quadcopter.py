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
from motor import *
import time
from AngleMeterAlpha import AngleMeterAlpha

rad_to_deg = 180/3.141592654


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("#")
 
class quadcopter():

    def __init__(self,m1_pin,m2_pin,m3_pin,m4_pin,calibrating):
        '''
            Angles of the quadcopter
                    [ROLL,PITCH,YAW]
        '''
        self.angleMeter = AngleMeterAlpha()
        self.angleMeter.measure()

        self.angles = [0,0,0]
        '''
            Constants for the quadcopter
            MIN : is the minimum speed for each of the quadcopter motors
            PITCH_DES_ANGLE : is the upper limit for the speed
            ROLL_DES_ANGLE  : the angle of pitch desired, is is 0 degrees in idle
           HOSTNAME : the broker to witch connect 
        '''
        self.MIN    = 1000
        self.MAX    = 1400
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
                        motor(m4_pin,self.MIN,self.MAX,self.pi),
                        motor(m3_pin,self.MIN,self.MAX,self.pi)
                        ]
  
        self.time   = 0
        self.deltat = 0
        '''
            The quadcopter balancing is done via a PID controller
            The values of the PID, might be different for the ROLL and PITCH angles
        '''
        #ROLL
        self.KR      = [0.4,0,0]
        self.pidR    = PID(Kp=self.KR[0],Ki=self.KR[1],Kd=self.KR[2],setpoint=self.ROLL_DES_ANGLE)
        #PITCH
        self.KP      = [0.04,0,0]
        self.pidP    = PID(Kp=self.KP[0],Ki=self.KP[1],Kd=self.KP[2],setpoint=self.PITCH_DES_ANGLE)


        '''
            MQTT is used as standar communication protocol between the android app and the quadcopter. It is not a big deal, but the simplest protocol i know, and for this reason the one i will use for the moment.
        '''
        self.mqttc = mqtt.Client()
        self.mqttc.on_connect = on_connect
        self.mqttc.on_message = self.on_message
        self.mqttc.connect(self.HOSTNAME)
        self.mqttc.loop_start()

        '''
            Variables for generic informations.
        '''
        self.power = False
        '''
            Filters section
        '''
        self.filter0 = FILTER()
        self.filter1 = FILTER()
        '''
            calibrate it, during the creation the calibrate method is called, hence calibrating the motors
        '''
        if calibrating == True:
            self.calibrate()

   

    def on_message(self,client, userdata, msg):
        stringa = str(msg.payload).replace("b'",'')
        stringa = str(stringa).replace("'",'')

        if ("STOP" in msg.topic) == True:
            self.emergency_stop()
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
    def get_roll_pitch(self):           

        self.angles = [self.angleMeter.get_complementary_roll(), self.angleMeter.get_complementary_pitch()]
        
        return self.angles


    '''
        The balance_PID method, is used to balance the quadcopter motors.
        They are balanced by adding or subtracting to the speeds of the motors
        the results of the PID[ROLL|PITCH].
    '''
    def balance_PID(self):
        # Update the angle values
        [roll, pitch] = self.get_roll_pitch()


        '''
            In these two lines the error is calculated by the difference of the 
            actual angle - the desidred_angle.
        '''
        pid_response_R  = self.pidR( roll - self.ROLL_DES_ANGLE)
        pid_response_P  = self.pidP( pitch - self.PITCH_DES_ANGLE)

        print(f"PIDR: {pid_response_R} | PIDP: {pid_response_P}")


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
            motor.set_speed(speed,True)

    '''
        This method is used to publish telemetry data to MQTT
    '''
    def publish_info(self):
        '''
        the topics are of type drone/(angle,motor_speed)/([roll,pitch,yaw],[m1,m2,m3,m4])
        '''
        self.mqttc.publish("drone/angle/roll",str(self.angles[0]))
        self.mqttc.publish("drone/angle/pitch",str(self.angles[1]))

        string = "drone/motor_speed/motor_"
        i = 0
        for motor in self.motors:
            i += 1
            topic = string + str(i)
            self.mqttc.publish(topic,str(motor.get_speed()))

    '''
        Use this to start the quadcopter
    '''
    def start(self):
        # Destroy the file
        for v in range(800,1200):
            self.set_all_speed(v)
            print(v)
            time.sleep(0.01)
            self.start_time = round(time.time() * 1000)
        while True:
            # Balance the PID
            self.balance_PID()
            # Update the angle values
            self.get_roll_pitch()
            # Print to stdout
            print(f"ROLL: {round(self.angles[0],3)} | PITCH: {round(self.angles[1],3)}")
            # Send data over MQTT
            self.publish_info()
            time.sleep(0.01)


    '''
        Use this in order to stop the motors
    '''
    def stop(self):
        for motor in self.motors:
            motor.slow_stop()

    '''
        Use this in case of emergency, be careful propellers might fly away
    '''
    def emergency_stop(self):
        for motor in self.motors:
            motor.slow_stop()