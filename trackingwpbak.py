from __future__ import print_function
from __future__ import division
from multiprocessing import Process, Pipe
import turret_mot_ctrlwp

from collections import deque
import numpy as np
import argparse
import imutils
import cv2
import time
import threading
from imutils.video import VideoStream
from picamera.array import PiRGBArray
from picamera import PiCamera
from pivideostream import PiVideoStream
import RPi.GPIO as GPIO                    #Import GPIO library
GPIO.setmode(GPIO.BOARD)                     #Set GPIO pin numbering 

"""
Globals
"""
cntrx = 0
cntry = 0
x_cmd = "s"
y_cmd = "s"
fire_cmd = "s"
cmd_to_send = ["s", "s", "s"]
cam_width = 320
cam_height = 240
center_width = cam_width/2
center_height = cam_height/2
extra = 50
xgood = False
ygood = False
missile_count = 0
max_missile_count = 4
bullet_count = 0   ## how many bullets have been shot 
max_bullet_count = 6 ##how many total bullets can be shot before out of ammo
gun_type = None
obj_min_w = 35
camera = None
rawCapture = None
biggest_obj = None
counter = False
dir_count = 0
"""
Globals related to GPIO
"""
RESET = 15
GPIO.setup(RESET, GPIO.IN)
up_lim = 29
rt_lim = 36
lt_lim = 37
GPIO.setup(rt_lim, GPIO.IN, GPIO.PUD_DOWN)  #once up movement limit is reached a switch is set HIGH.
GPIO.setup(lt_lim, GPIO.IN, GPIO.PUD_DOWN)  #once up movement limit is reached a switch is set HIGH.
"""
HAAR Cascade xml files/locations
"""
face_cascade = cv2.CascadeClassifier('/home/pi/opencv-3.2.0/data/haarcascades/haarcascade_frontalface_default.xml')
sphere_cascade = cv2.CascadeClassifier('/home/pi/opencv_katie/tracking/testing/cascadeandrew.xml')
#sphere_cascade = cv2.CascadeClassifier('/home/pi/opencv_katie/tracking/testing/cascadeball.xml')
cone_cascade = cv2.CascadeClassifier('/home/pi/opencv_katie/tracking/testing/20_stage_normal_cone.xml')
invert_cone_cascade = cv2.CascadeClassifier('/home/pi/opencv_katie/tracking/testing/cascadeupsidedown.xml')

"""
Calculates if the x point given via the global cntrx is to the right
left, or on the global center_width which repersents the  0 x point of the
camera's field of vision in cardestian format.

"""
class x_Thread(threading.Thread):
	def __init__(self, interval=.1):
		""" Constructor
		:type interval: float
		:param interval: Check interval, in seconds
		"""
		threading.Thread.__init__(self)
		self.interval = interval
		global cntrx
		global x_cmd
		global rt_lim
		global lt_lim
		global dir_count

		
	def run(self):
		global cntrx
		global x_cmd
		global center_width
		global extra
		global xgood
		global missile_count
		global rt_lim
		global lt_lim
		global dir_count
		""" Method that runs forever """


		while True:
                    if cntrx == None:
                        if GPIO.input(lt_lim) == 1:
                            x_cmd = "r"
                        elif GPIO.input(rt_lim) == 1:
                            x_cmd = "l"
                        else:
                            if x_cmd == "s":
                                x_cmd = "l"
                            x_cmd = x_cmd
                        xgood = False
                        #print("x_cmd: ", x_cmd)
                        time.sleep(.05)
                    elif cntrx == -1:
                        cntrx = cntrx
                    else:                    
			if cntrx < center_width - extra:
				x_cmd = "l"
				xgood = False
			elif cntrx > center_width + extra:
				x_cmd = "r"
				xgood = False
			else:
				x_cmd = "s"
				xgood = True				
			time.sleep(self.interval)
			
"""
Calculates if the y point given via the global cntry is up
down, or on the global center_height which repersents the  0 y point of the
camera's field of vision in cardestian format.

"""
class y_Thread(threading.Thread):
	def __init__(self, interval=.1):
		""" Constructor
		:type interval: float
		:param interval: Check interval, in seconds
		"""
		threading.Thread.__init__(self)
		self.interval = interval 
		global cntry
		global y_cmd
		
	def run(self):
		global cntry
		global y_cmd
		global center_height
		global extra
		global ygood
		""" Method that runs forever """

		while True:
			if cntry < center_height - extra:
				y_cmd = "u"
				ygood = False
			elif cntry > center_height + extra:
        			y_cmd = "d"
				ygood = False
			else:
				y_cmd = "s"
				ygood = True
			time.sleep(self.interval)
"""
Sends the movement commands to the turret via the pipe
connection, p. The commands are in the form of a string.
string element 0: left, right, or stop
string element 1: up, down, or stop
string element 2: fire launcher, or stop
The x direction command is passed between the threads via the global x_cmd
the y direction command is passed between the threads via the global y_cmd
The fire command is decided on based if both x point and y point are withing
the center of the camera's view. The calcuation to fire for the x direction
is passed in via the global xgood and the y direction is passed in via the global
ygood.
When the global missile_count exceeds or is equal to the global max_missile_count
The turret/camera are placed in the center of the vechile and the x direction
motors and the y directions are put into "s" mode. In this state the send_Thread
not relays information to main process that is now in "reload" mode and
just passes if it sees the object specified by checking if the cntrx and cntry
points are 0 or greater. This signifies that atleast of the desired object
has been spotted.
"""
class send_Thread(threading.Thread):
	def __init__(self, p, p_pipe, interval=.1):
		""" Constructor
		:type interval: float
		:param interval: Check interval, in seconds
		"""
		threading.Thread.__init__(self)
		self.interval = interval
		self.dir_pipe = p
		
		self.pasta_pipe = p_pipe
		global cmd_to_send
		global x_cmd
		global y_cmd
		global fire_cmd
		global xgood
		global ygood
		global missile_count
		global bullet_count
		global max_missile_count
		global max_bullet_count
		global up_lim
		global rt_lim
		global cntrx
		global cntry
                global gun_type
		last_time_fired = 0

	def run(self):
		global cmd_to_send
		global x_cmd
		global y_cmd
		global fire_cmd
		global xgood
		global ygood
		global missile_count
		global bullet_count
		global max_missile_count
		global max_bullet_count
		global up_lim
		global rt_lim
		global counter
		global cntrx
		global cntry
                global gun_type
		last_time_fired = time.time()
                last_pos = [0,0]
		""" Method that runs forever """
		
		##check to see if its been over 6 seconds since last fire
		##going to check xgood and ygood and if true
		##will change fire_cmd here.
		##else will be "s"


		while True:
			if xgood == True and ygood == True:
				last_pos = [cntrx, cntrx]
				if gun_type == "m":
					if time.time() - last_time_fired > 6:
						fire_cmd = "m"
						missile_count = missile_count+1
						last_time_fired = time.time()
						print("missile count: ", missile_count)
				if gun_type == "b":
					if cntrx == last_pos[0] and cntry == last_pos[1]:
						fire_cmd = "6"
						bullet_count = 6
					else:
						fire_cmd = "1"
						bullet_count = bullet_count + 1
					print("bullet count: ", bullet_count)
			else:
				fire_cmd = "s"
##                        print("missile count {}", missile_count)
##                        print(bullet_count)
			if missile_count < max_missile_count or bullet_count < max_bullet_count:
				cmd_to_send.insert(0, x_cmd)
				cmd_to_send.insert(1, y_cmd)
				cmd_to_send.insert(2, fire_cmd)
				self.dir_pipe.send("".join(cmd_to_send))
				#print(cmd_to_send)
				
			## the check to see if bot needs to go into "reload" mode
			## counter is to ensure it only centers once
			elif counter == False and missile_count >= max_missile_count and bullet_count >= max_bullet_count:
				while GPIO.input(up_lim) == GPIO.LOW:
					self.dir_pipe.send("sus")
					time.sleep(.5)
				self.dir_pipe.send("sss")
				time.sleep(.5)
				for i in range(0,5):
                                        self.dir_pipe.send("sds")				
				while GPIO.input(rt_lim) == GPIO.LOW:					
                                        self.dir_pipe.send("rss")
					time.sleep(0.5)								
				for i in range(0,23):					
                                        self.dir_pipe.send("lss")                               				
                                self.dir_pipe.send("sss")
				time.sleep(.3)
				counter = True  ## used to make sure it only centers one time
			else:
                                self.dir_pipe.send("sss")
				##maybe pipe screwing it up somehow
				if cntrx < 0 and cntry < 0:
					#print("test true")
					self.pasta_pipe.send("True")
				####is this needed or can we just poll on pasta for a new value on the pipe??
				else:
					#print("test false")
					self.pasta_pipe.send("False")
			#print(fire_cmd)
			
			time.sleep(self.interval)

"""
takes an image file and cascade file name.
scan image file for object to be detected
returns a list  x location, ylocation, width and height of the largest object found
returns None if no objects found.
"""

def detect_largest_obj(img, cascade_to_use):
	global face_cascade
	global sphere_cascade
	global cone_cascade
	global invert_cone_cascade
	biggest_tuple = None
	if cascade_to_use == face_cascade:     
		obj_fnd_list = cascade_to_use.detectMultiScale(img, 1.05, 1)
		#obj_fnd_list = face_cascade.detectMultiScale(img, scaleFactor=1.3, minNeighbors=4, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)
	elif cascade_to_use == sphere_cascade:     
		obj_fnd_list = cascade_to_use.detectMultiScale(img, 1.05, 1)
	elif cascade_to_use == cone_cascade:
		obj_fnd_list = cascade_to_use.detectMultiScale(img, 1.05, 1)
	elif cascade_to_use == invert_cone_cascade:
		obj_fnd_list = cascade_to_use.detectMultiScale(img, 1.05, 1)


	if obj_fnd_list != None:
		obj_area = 0
		biggest_obj_area = 0
		biggest_tuple = None
		for obj_fnd_tuple in obj_fnd_list:
			w = obj_fnd_tuple[2]
			h = obj_fnd_tuple[3]
			obj_area = (w) * (h)
			if obj_area > biggest_obj_area:
				biggest_obj_area = obj_area
				biggest_tuple = obj_fnd_tuple
	return biggest_tuple





   
		
"""
Uses Haar Cascade xml files to scan images for what object.
If an object(s) are found, it calculates the largest area and
sends the center x and y point of the object to the send_Thread
by means of the globals cntrx and cntry. If the global missile count
is greater or equal to global max_missile_count then it switches
Haar cascade files and begins to search for that object.

list returned  by detect_largest_obj function are in this order
list[0] = x position
list[1] = y position
list[2] = width
list[3] = height

gun_type is set to "b" if bullet_count < max_bullet_count and cone_cascade is used
gun_type is set to "m"  if missile_count < max_missile_count and sphere_cascade is used
gun_type is set to "n" then missile_count >= max_missile_count and bullet_count >= max_bullet_count so no gun is availiable 
gun_type is set to "n" by default

"""
def tracking(camera):
	global cntrx
	global x_cmd
	global center_width
	global extra
	global xgood
	global y_cmd
	global fire_cmd
	global ygood
	global missile_count
	global cntry
	global center_height
	global obj_min_w
	global rawCapture
	global inverted
	global biggest_obj
	global max_missile_count
	global bullet_count
	global max_bullet_count
	global gun_type
	count_center = 0

	stream = camera.capture_continuous(rawCapture, format="bgr", use_video_port=True)
	for frame in stream:

		img = frame.array
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		#gray = cv2.equalizeHist(gray)
                biggest_obj = None
	##turret has ammo and mini gun has ammo
                if missile_count < max_missile_count and bullet_count < max_bullet_count:
                    #print("in both")
                    temp1_obj = None
                    temp2_obj = None
                            
                    temp1_obj = detect_largest_obj(gray, sphere_cascade)  ##turret
                    temp2_obj = detect_largest_obj(gray, cone_cascade)      ## mini gun
##                    if temp1_obj != None:
##                            print ("temp1 width", temp1_obj[2])
##                    if temp2_obj != None:
##                            print("temp2 width", temp2_obj[2])
                    if temp1_obj != None and temp2_obj != None:  ##both objects are found
                                        if temp1_obj[2] >= temp2_obj[2]:
                                                    biggest_obj = temp1_obj
                                                    gun_type = "m"
                                                    #print("sphere largest")
                                        else:
                                                    biggest_obj = temp2_obj
                                                    gun_type = "b"
                                                   # print("cone largest")
                                        
                    elif temp1_obj == None and temp2_obj != None:   ##if one is not found , then set var to other object
                                    biggest_obj = temp2_obj
                                    gun_type = "b"
                                    #print("in missile out of ammo")
                    elif temp2_obj == None and temp1_obj != None:     ##if one is not found , then set var to other object
                                    biggest_obj = temp1_obj
                                    gun_type = "m"
                                    #print("in bullet out of ammo")
                    else:                       ## something happened so set back to None 
                                    biggest_obj = None
                                    gun_type = "n"
                                    
		## turret is out of ammo but mini gun has ammo  
		elif missile_count >= max_missile_count and bullet_count < max_bullet_count:
			biggest_obj = detect_largest_obj(gray, cone_cascade)
			gun_type = "b"
		## turret has ammo but mini gun is out of ammo
		elif missile_count < max_missile_count and bullet_count >= max_bullet_count:
			biggest_obj = detect_largest_obj(gray, sphere_cascade)
			gun_type = "m"
		##turret is out of ammo and mini gun is out of ammo
		else:
			#biggest_obj = None
			biggest_obj = detect_largest_obj(gray, invert_cone_cascade)
			gun_type = "n"
			cntrx = -1
			cntry = -1


		if biggest_obj != None:
			#print(biggest_obj)
			if biggest_obj[2] > obj_min_w:
				cntrx = biggest_obj[0] + (biggest_obj[2]/2)
				cntry = biggest_obj[1] + (biggest_obj[3] /2)
				cv2.circle(img, (int(cntrx), int(cntry)), 2, (0,255,0), -1)
                ##this needs work rh
		else:
                        if cntrx == -1 and cntry == -1:
                                cntrx = cntrx
                                cntry = cntry
                        else:
                                #print("no objects found")
                                cntrx = None 
                                cntry = "no_obj"

		cv2.circle(img, (int(center_width), int(center_height)), 3, (0,0,255), -1)
		cv2.imshow("Frame", img)
		rawCapture.truncate(0)
		k = cv2.waitKey(1) & 0xff or KeyboardInterrupt
		time.sleep(.01)

   # print(missile_count)
#this needs work. Dont think I am closing everything down right.

	rawCapture.close()
	camera.close()
	camera.release()
	cv2.destroyAllWindows()
		
	




def trackingMain(pipein):

	global cntrx
	global x_cmd
	global center_width
	global extra
	global xgood
	global y_cmd
	global fire_cmd
	global ygood
	global missile_count
	global cntry
	global center_height
	global obj_min_w
	global rawCapture
	count_center = 0

	dir_conn, turret_conn = Pipe()
	p = Process(target = turret_mot_ctrlwp.command_sel, args=(turret_conn,))
	p.start()
	print("turret motor control online!")

	x_aim_thread = x_Thread(.06)
	x_aim_thread.start()

	y_aim_thread = y_Thread(.06)
	y_aim_thread.start()

	send_cmd_thread = send_Thread(dir_conn , pipein, .6) #might be able to speed this up.
	send_cmd_thread.start()
        #print(up_lim_pin)

	
	camera = PiCamera()
	#camera.hflip = True ## probably need to remove these
	camera.vflip = True ##
	camera.resolution = (cam_width, cam_height)
	camera.framerate = 32
	rawCapture = PiRGBArray(camera, size=(cam_width, cam_height))
	
	print("setting up picamera. 2 second wait")
	
	time.sleep(2)
	while(1):
		
		tracking(camera)

	rawCapture.close()
	camera.close()
	camera.release()
	cv2.destroyAllWindows()
	
		#
if __name__ == '__main__':
    while 1:
        inTrack_pipe, outTrack_pipe = Pipe()
        trackingMain( outTrack_pipe)



