import cv2
import threading
import numpy as np
import sys
import os
import time

import zmq
#import base64
#import picamera
#from picamera.array import PiRGBArray
import argparse
import imutils
#from collections import deque
#import psutil
#import datetime
from rpi_ws281x import *

import robotLight
import move

import laser
import ultrasonic

cascPath = '/home/pi/adeept_raspclaws/server/haarcascade_frontalface_default.xml'
faceCascade = cv2.CascadeClassifier(cascPath)

font = cv2.FONT_HERSHEY_SIMPLEX

framecounter = 0

distance = str(ultrasonic.ultdist())

Y_lock = 0
X_lock = 0

resW = 320	# Resolution width and
resH = 240	# Height
tor = 15		# Tolerance for centering camera

video_cap = cv2.VideoCapture(0)
video_cap.set(3,resW)
video_cap.set(4,resH)
video_cap.set(15,2)

RL=robotLight.RobotLight()
RL.start()
RL.breath(70,70,255)

laser.setup()

def loop():
		# Capture frame for each loop
		ret, frame = video_cap.read()
		
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		
		faces = faceCascade.detectMultiScale(
			gray,
			scaleFactor=1.1,
			minNeighbors=5,
			minSize=(30, 30),
			flags=cv2.CASCADE_SCALE_IMAGE
		)
		
		# Draw rectangle
		for (x, y, w, h) in faces:
			cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
			targetX = int(x+(w/2))
			targetY = int(y+(h/2))
			print(len(faces))
			
		# Distance sensor
		framedist = 'distance: ' + distance + 'cm'
		cv2.putText(frame, str(framedist) ,(40,20), font, 0.5,(255,255,255),1,cv2.LINE_AA)
		
		# Track face
		if len(faces) > 0:
			cv2.putText(frame,'Face Detected',(40,60), font, 0.5,(255,255,255),1,cv2.LINE_AA)
			laser.laser(0) # turn laser off
			print(targetX, targetY)
			
			# Center face along horizontal bisector
			if targetY < ((resH/2)-tor):
				error = ((resH/2)-targetY)/20
				outv = int(error)
				move.look_up(outv)
				Y_lock = 0
			elif targetY > ((resH/2)+tor):
				error = (targetY-(resH/2))/20
				outv = int(error)
				move.look_down(outv)
				Y_lock = 0
			else:
				Y_lock = 1
				
			# Center face along vertical bisector
			if targetX < ((resW/2)-tor):
				error_X = ((resW/2)-targetX)/20
				outv_X = int(error_X)
				move.look_left(outv_X)
				X_lock = 0
			elif targetX > ((resW/2)+tor):
				error_X = (targetX-(resW/2))/20
				outv_X = int(error_X)
				move.look_right(outv_X)
				X_lock = 0
			else:
				X_lock = 1
			
		else:
			cv2.putText(frame,'Searching for Face...',(40,60), font, 0.5,(0,0,255),1,cv2.LINE_AA)
			laser.laser(1) # turn laser on
			
		# Display frame
		cv2.imshow('Face Tracker', frame)
		
			
while True:
	
	if framecounter % 30 == 0:
		dist = ultrasonic.ultdist()
		if dist < 150.0:
			distance = str(dist)
		else:
			distance = 'ERROR... '
		loop()
	
	else:
		loop()
		
	framecounter +=1
	
	if cv2.waitKey(1) & 0xFF == ord('q'):
			RL.pause()
			laser.destroy()			
			break
	
			
# Release capture
video_cap.release()
cv2.destroyAllWindows()

	
