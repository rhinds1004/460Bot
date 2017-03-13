from __future__ import print_function
from __future__ import division

import RPi.GPIO as GPIO           # import RPi.GPIO module
import time
#from __future__ import braces
GPIO.setmode(GPIO.BCM)            # choose BCM or BOARD 
senA = 12
senB = 20
LED_ON = 16
GPIO.setup(senA, GPIO.IN)
GPIO.setup(senB, GPIO.IN)
GPIO.setup(LED_ON, GPIO.OUT)
GPIO.output(LED_ON, GPIO.HIGH)

start_time_A = 0
start_time_B = 0
curr_time_senA = 0
curr_time_senB = 0

def my_callbackA(channel):
    global curr_time_senA
    curr_time_senA = time.time()

def my_callbackB(channel):
    global curr_time_senB
    curr_time_senB = time.time()    

GPIO.add_event_detect(senA, GPIO.FALLING, callback=my_callbackA)
#GPIO.add_event_detect(senB, GPIO.FALLING, callback=my_callbackB)
a = True

##time_out is the length of time beyond which you consider the sensor reading completely black

def readSensor(num_sen, pin_list, read_arr, time_out):
    i = 0
    for i from [0:len(read_arr)]: ##pretty sure syntax is wrong
        read_arr[i] = time_out
        GPIO.setup(pin_list[i], GPIO.OUT)
        GPIO.output(pin_list[i], GPIO.HIGH)
    time.sleep(.1)
    start_time = time.time()
    while((curr_time = time.time() - start_time ) < time_out):
        for i from [0:len(read_arr)]: ##pretty sure syntax is wrong
            GPIO.setup(pin_list[i], GPIO.IN)
            if GPIO.input(pin_list[i]) == GPIO.LOW and curr_time < read_arr[i]:
                          read_arr[i] = curr_time

try:
    while True:
        GPIO.setup(senA, GPIO.OUT)
        GPIO.setup(senB, GPIO.OUT)
        GPIO.output(senA, GPIO.HIGH)
        GPIO.output(senB, GPIO.HIGH)
        time.sleep(.1)
        start_time_A = time.time()
        start_time_B= time.time()
        GPIO.setup(senA, GPIO.IN)
        GPIO.setup(senB, GPIO.IN)
        #time.sleep(.1)
        while(GPIO.input(senB) != GPIO.LOW):
            a=1
        curr_time_senB = time.time()
        #print("curr time for sensor A", curr_time_senA)
        #print("curr time for sensor B", curr_time_senB) 
        #print("time for sensor A", curr_time_senA - start_time_A)
        print("time for sensor B", curr_time_senB - start_time_B)
        #a = False

except KeyboardInterrupt:          # trap a CTRL+C keyboard interrupt  
    GPIO.cleanup()                 # resets all GPIO ports used by this program except KeyboardInterrupt:
    
    
              
        
