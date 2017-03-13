#!/usr/bin/env python
# -*- coding: iso-8859-1 -*-

'''
# This is an example of controlling the rotation of motors using encoders

# BrickPi's first Arduino runs motor ports A & b, as well as sensor ports 1 & 2. Arduino 2 does the rest of them.
# You may get better performance (certainly fewer communication errors with the BrickPi, and less work using my version
#   of the BrickPi.py driver) if you place all your motors and sensors on the same Arduino (if possible).
# My version of BrickPi.py won't bother to poll (triggered by BrickPi.UpdateValues()) an Arduino without gear.
'''

from __future__ import print_function
from __future__ import division
from multiprocessing import Process, Pipe
from builtins import input
from MPU9250 import *
import time
from BrickPi import * 				#import BrickPi.py file to use BrickPi operations
from MultiMotorDriving import * 	#So can do precision motor rotations
import pygame
import threading
import trackingwpbak
import time
import RPi.GPIO as GPIO                    #Import GPIO library
GPIO.setmode(GPIO.BOARD)                     #Set GPIO pin numbering 

lightLeft = 22
lightRight = 12
GPIO.setup(lightLeft, GPIO.IN)
GPIO.setup(lightRight, GPIO.IN)


##TRIG = 22                                  #Associate pin 12 to TRIG
##ECHO = 12                                  #Associate pin 22 to ECHO
##GPIO.setup(TRIG,GPIO.OUT)                  #Set pin as GPIO out
##GPIO.setup(ECHO,GPIO.IN)                   #Set pin as GPIO in

RESET = 15
GPIO.setup(RESET, GPIO.IN, GPIO.PUD_DOWN)

pygame.init()
pygame.mixer.init()
magFoundSnd = pygame.mixer.Sound("vegetaover9000.wav")

speed = -100
turn45 = 255
turn90 = 510
turn180 = 1050
turn360 = 2200

colors=["Black","Blue","Green","Yellow","Red", "White","Brown"]

BrickPiSetup()  	#Setup the serial port for communication (***NB*** APP MUST BE RUN IN SUDO MODE ***NB***)
ultraSonicLeft = PORT_2
ultraSonicMiddle = PORT_1
ultraSonicRight = PORT_3

leftMotor = PORT_A
rightMotor = PORT_B
BrickPi.SensorType[ultraSonicLeft] = TYPE_SENSOR_ULTRASONIC_CONT
BrickPi.SensorType[ultraSonicMiddle] = TYPE_SENSOR_ULTRASONIC_CONT
BrickPi.SensorType[ultraSonicRight] = TYPE_SENSOR_ULTRASONIC_CONT

motors = [leftMotor, rightMotor]
BrickPi.MotorEnable[leftMotor] = 1	#Don't turn these off - set its speed to 0 to stop a motor
BrickPi.MotorEnable[rightMotor] = 1
BrickPiSetupSensors()       		#Send the properties of sensors to BrickPi
BrickPi.Timeout = 30000     		#So motors won't stop cause of lack of contact (30 seconds)
BrickPiSetTimeout()				# (BrickPi's default is 250 msec (really meeses with motor reliability))

dis = 0

# Hopefully this is timer that runs in the background and fires every 20ms .

class TimerThread(object):
    """ Threading example class
    The run() method will be started and it will run in the background
    until the application exits.
    """

    def __init__(self, interval=.02):
        """ Constructor
        :type interval: float
        :param interval: Check interval, in seconds
        """
        self.interval = interval

        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True                            # Daemonize thread
        thread.start()                                  # Start the execution

    def run(self):
        """ Method that runs forever """
        while True:
            BrickPiUpdateValues()
            magCheck()
            #print('Doing something imporant in the background')

            time.sleep(self.interval)
  
def turnLeft(i):
    if i == 0:
        zeroEncoders()
#        continue
    result = BrickPiUpdateValues() 
    if not result :					# if updating values succeeded
       # print ("=============")
        encoderStartLeft = BrickPi.Encoder[leftMotor]
        encoderStartRight = BrickPi.Encoder[rightMotor]
#        print ( "Encoder Value: " + str(encoderStartLeft) + ' ' + str(encoderStartRight))	# print the encoder raw 
        power=[speed, speed] # 0 to 255
        deg = [i, -i]
        #maxWheelSpeedDiff = motorRotateDegree (power, deg, motors, sampling_time=0.0) #to use BrickPi's version
        maxWheelSpeedDiff = motorRotateDeg (power, deg, motors, sampling_time=0.1)		#to use MultiMotorDriving's super version
        encoderEndLeft = BrickPi.Encoder[leftMotor]
        encoderEndRight = BrickPi.Encoder[rightMotor]
        BrickPiSense()

def turnRight(i):
    if i == 0:
        zeroEncoders()
#        continue
    result = BrickPiUpdateValues() 
    if not result :					# if updating values succeeded
      #  print ("=============")
        encoderStartLeft = BrickPi.Encoder[leftMotor]
        encoderStartRight = BrickPi.Encoder[rightMotor]
#        print ( "Encoder Value: " + str(encoderStartLeft) + ' ' + str(encoderStartRight))	# print the encoder raw 
        power=[speed, speed] # 0 to 255
        deg = [-i, i]
        #maxWheelSpeedDiff = motorRotateDegree (power, deg, motors, sampling_time=0.0) #to use BrickPi's version
        maxWheelSpeedDiff = motorRotateDeg (power, deg, motors, sampling_time=0.01)# changed sampling time from 0.0 to 0.01 RH to use MultiMotorDriving's super version
        encoderEndLeft = BrickPi.Encoder[leftMotor]
        encoderEndRight = BrickPi.Encoder[rightMotor]
        BrickPiSense()
        
def turnBack(i):
    if i == 0:
        zeroEncoders()
#        continue
    result = BrickPiUpdateValues() 
    if not result :					# if updating values succeeded
       # print ("=============")
        encoderStartLeft = BrickPi.Encoder[leftMotor]
        encoderStartRight = BrickPi.Encoder[rightMotor]
#        print ( "Encoder Value: " + str(encoderStartLeft) + ' ' + str(encoderStartRight))	# print the encoder raw 
        power=[speed, speed] # 0 to 255
        deg = [i, i]
        #maxWheelSpeedDiff = motorRotateDegree (power, deg, motors, sampling_time=0.0) #to use BrickPi's version
        maxWheelSpeedDiff = motorRotateDeg (power, deg, motors, sampling_time=0.1)		#to use MultiMotorDriving's super version
        encoderEndLeft = BrickPi.Encoder[leftMotor]
        encoderEndRight = BrickPi.Encoder[rightMotor]
        BrickPiSense()
        
def go():
    BrickPi.MotorSpeed[leftMotor] = speed
    BrickPi.MotorSpeed[rightMotor] = speed
    BrickPiUpdateValues()
def stop():
    BrickPi.MotorSpeed[leftMotor] = 0
    BrickPi.MotorSpeed[rightMotor] = 0
    BrickPiUpdateValues()
    
def goback():
    BrickPi.MotorSpeed[leftMotor] = -speed
    BrickPi.MotorSpeed[rightMotor] = -speed
    BrickPiUpdateValues()

def chooseDir():
    turnLeft(turn90)
    BrickPiUpdateValues()
    rreading = BrickPi.Sensor[ultraSonic]
    time.sleep(1)
    turnRight(turn180)
    BrickPiUpdateValues()
    lreading = BrickPi.Sensor[ultraSonic]
    time.sleep(1)
    #if(lreading > rreading):
    turnLeft(turn180)
       # time.sleep(1)
   # go()


##def getgpioRead():
##  while 1:
##      global dis
##      GPIO.output(TRIG, False)                 #Set TRIG as LOW
##      #print ("Waitng For Sensor To Settle")
##      time.sleep(.06)                            #Delay of 2 seconds
##
##      GPIO.output(TRIG, True)                  #Set TRIG as HIGH
##      time.sleep(0.00001)                      #Delay of 0.00001 seconds
##      GPIO.output(TRIG, False)                 #Set TRIG as LOW
##
##      while GPIO.input(ECHO)==0:               #Check whether the ECHO is LOW
##        pulse_start = time.time()              #Saves the last known time of LOW pulse
##
##      while GPIO.input(ECHO)==1:               #Check whether the ECHO is HIGH
##        pulse_end = time.time()                #Saves the last known time of HIGH pulse 
##
##      pulse_duration = pulse_end - pulse_start #Get pulse duration to a variable
##
##      dis = pulse_duration * 17150        #Multiply pulse duration by 17150 to get dis
##      dis = round(dis, 2)            #Round to two decimal points
  

##  if dis > 2 and dis < 400:      #Check whether the dis is within range
##    print ("dis:",dis - 0.5,"cm")  #Print dis with 0.5 cm calibration
##  else:
##    print ("Out Of Range")                   #display out of range
      
mpu9250 = MPU9250()
mag = mpu9250.readMagnet()
magx = abs(mag['x'])
magy = abs(mag['y'])
magz = abs(mag['z'])
magTot = magx + magy + magz
maggoal = 400 #85
##
soundCnt = 0
##threadFrontUS = threading.Timer(.06, getgpioRead)
##threadFrontUS.start()

pipe_pasta, pipeSend = Pipe()
pd = Process(target = trackingwpbak.trackingMain, args=(pipeSend,))
pd.start()

def forwardAvoid():
    t_end = time.time() + 6
    while time.time() < t_end:
        result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors
        #print ("left: ", BrickPi.Sensor[ultraSonicLeft])
        #print ("right: ", BrickPi.Sensor[ultraSonicRight])
      #  getgpioRead()
        print(BrickPi.Sensor[ultraSonicMiddle])
        if not result :
            if  BrickPi.Sensor[ultraSonicMiddle] < 20:#BrickPi.Sensor[PORT] stores the value obtained from sensor

                stop()
                turnBack(360)
                turnRight(90)
                #print (dis)
               # time.sleep(.5)
            else:
           # if 1:
                BrickPiUpdateValues()
                if BrickPi.Sensor[ultraSonicRight]  > 100:
                    BrickPi.MotorSpeed[leftMotor] = -100
                elif BrickPi.Sensor[ultraSonicRight] < 30:
                    BrickPi.MotorSpeed[leftMotor] = -30
                else:
                    BrickPi.MotorSpeed[leftMotor] = -BrickPi.Sensor[ultraSonicRight]
                    
                if BrickPi.Sensor[ultraSonicLeft] > 100:
                    BrickPi.MotorSpeed[rightMotor] = -100
                elif BrickPi.Sensor[ultraSonicLeft] < 30:
                    BrickPi.MotorSpeed[rightMotor] = -30                
                else:
                    BrickPi.MotorSpeed[rightMotor] = -BrickPi.Sensor[ultraSonicLeft]
                BrickPiUpdateValues()
            BrickPiUpdateValues()

        if GPIO.input(lightLeft) == GPIO.LOW:
            stop()
            print ("found left white!")
            turnBack(550)
            turnRight(turn180)
        if GPIO.input(lightRight) == GPIO.LOW:
            stop()
            print ("found right white!")
            turnBack(550)
            turnLeft(turn180)

        time.sleep(.1)

def magCheck():
    global soundCnt
##    global pipe_pasta
##    
##    if pipe_pasta.poll() == True:
##    
##        inView=pipe_pasta.recv()
##        print(inView)
    mag = mpu9250.readMagnet()
    mx = abs(mag['x'])
    my = abs(mag['y'])
    mz = abs(mag['z'])
    xtot = magx-mx
    ytot = magy-my
    ztot = magz-mz
    difference = xtot +ytot+ztot
   # print (difference)

#######
## Once it enters this condition
## the light sensor goes off, but might not be an issue
## if switching it out anyways.
######
    while abs(difference) > maggoal:
       # print ("right")
       # goback()
        stop()
        #print(GPIO.input(RESET))
        print("magnet found")
        gohome()
       # waitForTracking()
"""
the inViews seems backwards check in trackingwpbak 's send_thread
to see if we need to switch the trues and false or we can change it here
"""

##def waitForTracking():
##    global pipe_pasta
##    global pipeSend
##    while GPIO.input(RESET) == GPIO.LOW:
##        if pipe_pasta.poll() == True:
##            message = pipe_pasta.recv()
##            if message == "Ready"
##            pipeSend.send("Ready")
##                gohome()
##        


def gohome():
    global pipe_pasta
#    global dis
    while GPIO.input(RESET) == GPIO.LOW:
        stop()
        i = 0
        inView = "False"
        print("pipe inview poll")
        print(pipe_pasta.poll())
        print("reset: ")
        print(GPIO.input(RESET))
        if pipe_pasta.poll() == True:
        
            inView=pipe_pasta.recv()
            print(inView)
            while i<4 and inView == "False" and GPIO.input(RESET) == GPIO.LOW:
                print(inView)
                if pipe_pasta.poll() == True:
                
                    inView=pipe_pasta.recv()
                    print(inView)
                
                    if inView == "False":
                        print("inView 90")
                        turnRight(turn90)
                        stop()
                        time.sleep(.05)
                i += 1
            
            forwardAvoid()
####        time.sleep(6)
    
        
##        if soundCnt == 0:
##            magFoundSnd.play()
##            time.sleep(3)
##            magFoundSnd.stop()
##            BrickPiSetupSensors()
##            soundCnt = 1
##        

while True:
    #print(dis)
    result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors
    #print ("left: ", BrickPi.Sensor[ultraSonicLeft])
    #print ("right: ", BrickPi.Sensor[ultraSonicRight])
  #  getgpioRead()
    
    if not result :
        print(BrickPi.Sensor[ultraSonicMiddle])
        print("Right color:", GPIO.input(lightRight))
        if  BrickPi.Sensor[ultraSonicMiddle] < 20:#BrickPi.Sensor[PORT] stores the value obtained from sensor

            
            stop()
            turnBack(360)
            turnRight(90)
            #print (dis)
            #time.sleep(.5)
        else:
       # if 1:
            BrickPiUpdateValues()
            if BrickPi.Sensor[ultraSonicRight]  > 100:
                BrickPi.MotorSpeed[leftMotor] = -100
            elif BrickPi.Sensor[ultraSonicRight] < 50:
                BrickPi.MotorSpeed[leftMotor] = -50
            else:
                BrickPi.MotorSpeed[leftMotor] = -BrickPi.Sensor[ultraSonicRight]
                
            if BrickPi.Sensor[ultraSonicLeft] > 100:
                BrickPi.MotorSpeed[rightMotor] = -100
            elif BrickPi.Sensor[ultraSonicLeft] < 50:
                BrickPi.MotorSpeed[rightMotor] = -50
            else:
                BrickPi.MotorSpeed[rightMotor] = -BrickPi.Sensor[ultraSonicLeft]
            BrickPiUpdateValues()
        BrickPiUpdateValues()
        
        #for two light sensors
        if GPIO.input(lightLeft) == GPIO.LOW:
            stop()
            print ("found left white!")
            turnBack(550)
            turnRight(turn180)
        if GPIO.input(lightRight) == GPIO.LOW:
            stop()
            print ("found right white!")
            turnBack(550)
            turnLeft(turn180)

        #for lego light sensor
##        color_sensor = BrickPi.Sensor[color]
##        if color_sensor == 6:
##            stop()
##            print ("found white!")
##            turnBack(550)
##            turnRight(turn180)
            
        magCheck()
        time.sleep(.1)
    time.sleep(.1)



