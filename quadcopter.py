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
import RPi.GPIO as GPIO

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
        self.MIN    = 1100
        self.MAX    = 1500
        self.PITCH_DES_ANGLE = 0
        self.ROLL_DES_ANGLE  = 0
        self.YAW_DES_ANGLE  = 0
        self.HOSTNAME         = "localhost"
        '''
            Creating the pi object for interactions with motors
        '''
        GPIO.setmode(GPIO.BCM)
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
        #YAW
        self.KY      = [2,0,0]
        self.pidY    = PID(Kp=self.KY[0],Ki=self.KY[1],Kd=self.KY[2],setpoint=self.YAW_DES_ANGLE)


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
    def get_roll_pitch_yaw(self):           

        self.angles = [self.angleMeter.get_complementary_pitch(), self.angleMeter.get_complementary_roll(), self.angleMeter.getYaw()]
        
        return self.angles
    

    '''
        The balance_PID method, is used to balance the quadcopter motors.
        They are balanced by adding or subtracting to the speeds of the motors
        the results of the PID[ROLL|PITCH].
    '''
    def balance_PID(self):
        # Update the angle values
        [roll, pitch, yaw] = self.get_roll_pitch_yaw()


        '''
            In these two lines the error is calculated by the difference of the 
            actual angle - the desidred_angle.
        '''
        pid_response_R  = self.pidR( roll - self.ROLL_DES_ANGLE)
        pid_response_P  = self.pidP( pitch - self.PITCH_DES_ANGLE)
        pid_response_Y  = self.pidY( yaw - self.YAW_DES_ANGLE)

        # Correct the motors using the PITCH
        self.motors[0].set_speed(self.motors[0].speed - int(pid_response_P),False)
        self.motors[1].set_speed(self.motors[1].speed - int(pid_response_P),False)
        self.motors[2].set_speed(self.motors[2].speed + int(pid_response_P),False)
        self.motors[3].set_speed(self.motors[3].speed + int(pid_response_P),False)


        # Correct the motors using the ROLL
        self.motors[0].set_speed(self.motors[0].speed - int(pid_response_R),False)
        self.motors[3].set_speed(self.motors[3].speed - int(pid_response_R),False)
        self.motors[1].set_speed(self.motors[1].speed + int(pid_response_R),False)
        self.motors[2].set_speed(self.motors[2].speed + int(pid_response_R),False)

        # Correct the motors using the YAW
        self.motors[0].set_speed(self.motors[0].speed - int(pid_response_Y),False)
        self.motors[1].set_speed(self.motors[1].speed + int(pid_response_Y),False)
        self.motors[2].set_speed(self.motors[2].speed - int(pid_response_Y),False)
        self.motors[3].set_speed(self.motors[3].speed + int(pid_response_Y),False)



        print(f"PIDR: {round(pid_response_R,2)} | PIDP: {round(pid_response_P,2)}  | PIDY: {round(pid_response_Y, 2)}")
        print(f"ROLL: {round(self.angles[0],3)} | PITCH: {round(self.angles[1],3)} | YAW : {round(self.angles[2], 3)}")

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
                self.arm()

    def control(self): 
        speed = 1300    # change your speed if you want to.... it should be between 700 - 2000
        print("Controls - a to decrease speed & d to increase speed OR q to decrease a lot of speed & e to increase a lot of speed")
        while True:
            for motor in self.motors:
                motor.set_speed(speed,False)
            inp = input()
            
            if inp == "q":
                speed -= 100    # decrementing the speed like hell
                print("speed = %d" % speed)
            elif inp == "e":    
                speed += 100    # incrementing the speed like hell
                print("speed = %d" % speed)
            elif inp == "d":
                speed += 10     # incrementing the speed 
                print("speed = %d" % speed)
            elif inp == "a":
                speed -= 10     # decrementing the speed
                print("speed = %d" % speed)
            elif inp == "stop":
                self.stop()          #going for the stop function
                break
            elif inp == "arm":
                self.arm()
                break	
            else:
                print("WHAT DID I SAID!! Press a,q,d or e")
        '''
        This method is used to change all the speeds of the motors
    '''
    def arm(self): #This is the arming procedure of an ESC 
        print("Arming")
        for speed in range(900, 1500):
            for motor in self.motors:
                print(f"Speed: {speed}")
                motor.set_speed(speed, True)
                time.sleep(.001)
        
        for speed in reversed(range(900,1500)):
            for motor in self.motors:
                print(f"Speed: {speed}")
                motor.set_speed(speed, True)
                time.sleep(.001)
 
        #self.control() 
    
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
        self.arm()
        #self.test_motors()
        self.start_time = round(time.time() * 1000)
        # End of testing, starting motors
        self.start_motors()
        # Starting pid test
        while True:
            # Balance the PID
            self.balance_PID()
            # Update the angle values
            self.get_roll_pitch_yaw()
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
    
    def test_single_motor(self, n_motor):
        for speed in range(900, 1200):
            self.motors[n_motor].set_speed(speed, True)
            time.sleep(.01)
    
    def test_motors(self):
        # Single motor testing
        self.set_all_speed(0)
        print("Testing motor 1")
        self.test_single_motor(0)
        self.set_all_speed(0)
        print("Testing motor 2")
        self.test_single_motor(1)
        self.set_all_speed(0)
        print("Testing motor 3")
        self.test_single_motor(2)
        self.set_all_speed(0)
        print("Testing motor 4")
        self.test_single_motor(3)
        self.set_all_speed(0)

    def start_motors(self):
        self.set_all_speed(900)
        for speed in range(900, 1100):
            self.set_all_speed(speed)