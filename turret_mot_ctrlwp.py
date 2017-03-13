from __future__ import print_function
from __future__ import division
from multiprocessing import Process, Pipe
import random
import RPi.GPIO as GPIO
import time
import thread
#from multiprocessing import Queue
GPIO.setmode(GPIO.BOARD)

#inputs for y axis motor and limit switches
updown_in_pin1 = 11 #3A
updown_in_pin2 = 13 #4Anote: these are physical pin locations on the board

up_lim_pin = 29 #up direction limit switch
down_lim_pin = 31 #down direction limit switch

#inputs for x axis motor and limit switches
rtlt_in_pin1 = 16  #1A
rtlt_in_pin2 = 18  #2A

rt_lim_pin = 36
lt_lim_pin = 37

#input for fire control motor
fc_in_pin = 38 #4A

#input pin for mini gun
mg_in_pin = 40 #mini gun input pin

missile_count = 0
bullet_count = 0


turret_sleep_interval = 3.8

GPIO.setup(up_lim_pin, GPIO.IN, GPIO.PUD_DOWN)  #once up movement limit is reached a switch is set HIGH.
GPIO.setup(down_lim_pin, GPIO.IN, GPIO.PUD_DOWN) #once down movement limit is reached a switch is set HIGH.
GPIO.setup(rt_lim_pin, GPIO.IN, GPIO.PUD_DOWN)  #once up movement limit is reached a switch is set HIGH.
GPIO.setup(lt_lim_pin, GPIO.IN, GPIO.PUD_DOWN) #once down movement limit is reached a switch is set HIGH.


GPIO.setup(updown_in_pin1, GPIO.OUT) #sets pins to output mode
GPIO.setup(updown_in_pin2, GPIO.OUT)
GPIO.setup(rtlt_in_pin1, GPIO.OUT) #sets pins to output mode
GPIO.setup(rtlt_in_pin2, GPIO.OUT)
GPIO.setup(mg_in_pin, GPIO.OUT)
GPIO.setup(fc_in_pin, GPIO.OUT)

## Checks if turret was left in a max postion in previous execution
## If one of the limit switches is engaged then sets the lock direction variable to true, otherwise false
up_dir_locked = GPIO.input(up_lim_pin)
down_dir_locked = GPIO.input(down_lim_pin)

rt_dir_locked = GPIO.input(rt_lim_pin) 
lt_dir_locked = GPIO.input(lt_lim_pin)

fire_locked = False
time2fire = False

#motor class for turret firing motor since only one input
class Launcher(object):
    def __init__(self, in_pin):
        self.in_pin = in_pin
        GPIO.setup(self.in_pin, GPIO.OUT) #sets pins to output mode
        GPIO.output(self.in_pin, GPIO.LOW)
        #global sf_lim_pin       
    def fire(self):
        GPIO.output(self.in_pin, GPIO.HIGH)      
    def stop(self):
        GPIO.output(self.in_pin, GPIO.LOW)

#motor class for miniGun motor only one input pin  
class MiniGun(object):
    def __init__(self, in_pin):
        self.in_pin = in_pin
        GPIO.setup(self.in_pin, GPIO.OUT) #sets pins to output mode
        GPIO.output(self.in_pin, GPIO.LOW)
        #global sf_lim_pin       
    def single(self):
        GPIO.output(self.in_pin, GPIO.HIGH)
	time.sleep(.0001)
        GPIO.output(self.in_pin, GPIO.LOW)
	
    def fire_all(self):
	GPIO.output(self.in_pin, GPIO.HIGH)
	time.sleep(.0006)
        GPIO.output(self.in_pin, GPIO.LOW)
		
    def stop(self):
        GPIO.output(self.in_pin, GPIO.LOW)

		
#MOTOR =D
class Motor(object):
    def __init__(self, in_pin1, in_pin2):
        self.in_pin1 = in_pin1
        self.in_pin2 = in_pin2
    
        GPIO.setup(self.in_pin1, GPIO.OUT) #sets pins to output mode
        GPIO.setup(self.in_pin2, GPIO.OUT)

    def up(self):
        global up_dir_locked
        global down_dir_locked
        if down_dir_locked == True:
            down_dir_locked = False
        if up_dir_locked == False:
            GPIO.output(self.in_pin1, GPIO.HIGH)
            GPIO.output(self.in_pin2, GPIO.LOW)

            return True
        else:
            return False

    def down(self):
        global up_dir_locked
        global down_dir_locked
        if up_dir_locked == True:
            up_dir_locked = False
        if down_dir_locked == False:
            GPIO.output(self.in_pin1, GPIO.LOW)
            GPIO.output(self.in_pin2, GPIO.HIGH)
            return True
        else:
            return False
        
    def right(self):
        global rt_dir_locked
        global lt_dir_locked
        if lt_dir_locked == True:
            lt_dir_locked = False
        if rt_dir_locked == False:
            GPIO.output(self.in_pin1, GPIO.HIGH)
            GPIO.output(self.in_pin2, GPIO.LOW)
            return True
        else:
            return False

    def left(self):
        global rt_dir_locked
        global lt_dir_locked
        if rt_dir_locked == True:
            rt_dir_locked = False
        if lt_dir_locked == False:
            GPIO.output(self.in_pin1, GPIO.LOW)
            GPIO.output(self.in_pin2, GPIO.HIGH)
            return True
        else:
            return False
        
    def stop(self):
        GPIO.output(self.in_pin1, GPIO.LOW)
        GPIO.output(self.in_pin2, GPIO.LOW)


##functions called by the interrupt routines for each limit switch        
def up_limit_detected(self):
    #chan_list = [11, 13]
##
##    global up_dir_locked
##    up_dir_locked = True
    global updown_in_pin1
    global updown_in_pin2

    GPIO.output(updown_in_pin1, GPIO.LOW)
    GPIO.output(updown_in_pin2, GPIO.LOW)
    time.sleep(.1)
    #print ("up test")
    
def down_limit_detected(self):
    #chan_list = [11, 13]

##    global down_dir_locked
##    down_dir_locked = True
    global updown_in_pin1
    global updown_in_pin2
    GPIO.output(updown_in_pin1, GPIO.LOW)
    GPIO.output(updown_in_pin2, GPIO.LOW)
    time.sleep(.1)
    #print ("down test")

def rt_limit_detected(self):
##    global rt_dir_locked
##    rt_dir_locked = True
    global rtlt_in_pin1
    global rtlt_in_pin2
    GPIO.output(rtlt_in_pin1, GPIO.LOW)
    GPIO.output(rtlt_in_pin2, GPIO.LOW)
    time.sleep(.1)
    #print ("right test")

def lt_limit_detected(self):
##    global lt_dir_locked
##    lt_dir_locked = True
    global rtlt_in_pin1
    global rtlt_in_pin2
    GPIO.output(rtlt_in_pin1, GPIO.LOW)
    GPIO.output(rtlt_in_pin2, GPIO.LOW)
    time.sleep(.1)
    #print ("left test")
    
def shot_fired_detected(self):
    global fc_in_pin
##    missile_count = missile_count + 1
##    
##    print(missile_count)
    GPIO.output(fc_in_pin, GPIO.LOW)

##    time.sleep(3.7)
##    #GPIO.output(fc_in_pin, GPIO.HIGH)
##    fire_locked = True
    #print("shot fired~")
    
##interrupts   
GPIO.add_event_detect(up_lim_pin, GPIO.RISING, callback = up_limit_detected)
GPIO.add_event_detect(down_lim_pin, GPIO.RISING, callback = down_limit_detected)
GPIO.add_event_detect(rt_lim_pin, GPIO.RISING, callback = rt_limit_detected)
GPIO.add_event_detect(lt_lim_pin, GPIO.RISING, callback = lt_limit_detected)
#GPIO.add_event_detect(sf_lim_pin, GPIO.FALLING, callback = shot_fired_detected, bouncetime=700 )

#launcher = Launcher(fc_in_pin)

##def fire_thread(Motor_obj):
##        global launcher
##        global fire_locked
##	print("in fire thread")
##        Motor_obj.fire()
##	time.sleep(turret_sleep_interval)
##	Motor_obj.stop()
##	fire_locked = False
	
##def fire_thread():
##        global fc_in_pin
##        global fire_locked
##        global turret_sleep_interval
##	print("in fire thread")
##        #launcher.fire()
##	GPIO.output(38, GPIO.HIGH)
##	print("gpio input" , GPIO.input(fc_in_pin))
##	time.sleep(3.8)
##	#launcher.stop()
##	GPIO.output(38, GPIO.LOW)
##	fire_locked = False

def fire_thread():
    t_end = time.time() + 3.8
   # print(t_end)
    GPIO.output(38, GPIO.HIGH)
    while time.time() < t_end:
        GPIO.output(38, GPIO.HIGH)
        #print(time.time())
    #print("gpio input" , GPIO.input(fc_in_pin))
    #  time.sleep(3.8)
    #launcher.stop()
    GPIO.output(38, GPIO.LOW)
    #fire_locked = False
    #print("gpio input" , GPIO.input(fc_in_pin))
#thread.start_new_thread( fire_thread,() )

def bullet_thread():
    GPIO.output(40, GPIO.HIGH)
    time.sleep(.1) #do the loop from fire_thrad if this sleep isnt working
    GPIO.output(40, GPIO.LOW)
    

def command_sel(pipe_line):

    try:
        direction = None
	##init of motors
        up_down_motor = Motor(updown_in_pin1, updown_in_pin2)
        rt_lt_motor = Motor(rtlt_in_pin1, rtlt_in_pin2)
        #launcher = Launcher(fc_in_pin)
	#minigun = MiniGun(mg_in_pin)
		
        up_down_motor.stop()
        rt_lt_motor.stop()

        global up_dir_locked
        global down_dir_locked
        global rt_dir_locked
        global lt_dir_locked
        global fire_locked
        global sf_lim_pin
        global up_lim_pin 
        global down_lim_pin 

        global fire_locked
        global rt_lim_pin 
        global lt_lim_pin
        global time2fire
        
        print("waitng for camera to come online!")
        time.sleep(3)
        while True:
            #    cmd = raw_input("Command, u/d/r/l/f/s :")
            direction = "sss"
            if pipe_line.poll() == True:
                
                direction = pipe_line.recv()
                #cmd = "sss"
                #print(direction + "\n")
                #print("\n")
                
                if len(direction) > 0:
                    
                   # print(direction)
                ## maybe set all the *_dir_locked to just read limit pin and
                   ##change if high. might cut down on cpu overhead?
                    if direction[1] == "u" and GPIO.input(up_lim_pin) == GPIO.LOW:
                        
                        up_down_motor.up()
                        
                      #  time.sleep(.5)

                    elif direction[1] == "d" and GPIO.input(down_lim_pin) == GPIO.LOW:
                        up_down_motor.down()
                       # time.sleep(.5)
                    elif direction[0] == "r" and GPIO.input(rt_lim_pin) == GPIO.LOW:
                        rt_lt_motor.right()
                      #  time.sleep(.5)

                    elif direction[0] == "l" and GPIO.input(lt_lim_pin) == GPIO.LOW:
                        rt_lt_motor.left()
                       # time.sleep(.5)
                    elif direction[2] == "m":
                           ## if fire_locked == False:
                             ##       fire_locked = True
                        thread.start_new_thread( fire_thread, () )
                                    
                    elif direction[2] == "1":
                            #thread.start_new_thread( bullet_thread, () )
                        GPIO.output(40, GPIO.HIGH)
                        print("GPIO 40", GPIO.input(40))
                        time.sleep(.2) #do the loop from fire_thrad if this sleep isnt working
                        GPIO.output(40, GPIO.LOW)
                    elif direction[2] == "6":
                            thread.start_new_thread( bullet_thread, () )
                        
                    else:
                        up_down_motor.stop()
                        rt_lt_motor.stop()
                        #launcher.stop()
			#minigun.stop()
            time.sleep(.1)
            up_down_motor.stop()
            rt_lt_motor.stop()
            time.sleep(.3)
            
    except KeyboardInterrupt:
        up_down_motor.stop()
        GPIO.cleanup()
        print ("\n Motors stopped!")
        
    finally:
        GPIO.cleanup()

def test(q):
    print("meself!")
    
##if __name__ == '__main__':
##    #cmd_list = ["u", "d", "l", "r", "f", "s"]
##    cmd_list = ["f","s"]
##
##    ctrl_conn, turret_conn = Pipe()
##    p = Process(target=command_sel, args=(turret_conn,))
##    p.start()
####    while raw_input != "s":
####        
####        GPIO.output(fc_in_pin, GPIO.HIGH)
####        print(GPIO.input(sf_lim_pin))
##   # tur_cmd = raw_input("Command, u/d/r/l/f/s :")
##    while True:
##        #tur_cmd = raw_input("Command, u/d/r/l/f/s :")
##        #ctrl_conn.send(tur_cmd)
##        tur_cmd = raw_input("Command, u/d/r/l/f/s :")
##        if len(tur_cmd) == 3:
##            ctrl_conn.send(tur_cmd)
##        else:
##            ctrl_conn.send("sss")
##    p.join()

    
    

        
        
        
