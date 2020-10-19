import cv2
import threading
import numpy as np
import sys
import os
import time

#import zmq
#import base64
#import picamera
#from picamera.array import PiRGBArray
#import argparse
#import imutils
#from collections import deque
#import psutil
#import datetime
from rpi_ws281x import *

import robotLight
import move

import laser
import ultrasonic

import utils

headMode = 'Default'
headMoveSensitivity = 5

cascPath = '/home/pi/adeept_raspclaws/server/haarcascade_frontalface_default.xml'
faceCascade = cv2.CascadeClassifier(cascPath)

font = cv2.FONT_HERSHEY_SIMPLEX

frameCounter = 1
frameZeroTime = time.time()
fps = 0


Y_lock = 0		# Target is centered in Y direction
X_lock = 0		# Target is centered in X direction

resW = 480		# Resolution width and
resH = 360		# Height
tor = 15			# Error tolerance for centering camera

video_cap = cv2.VideoCapture(0)
video_cap.set(3,resW)
video_cap.set(4,resH)
video_cap.set(15,2)

# Face Tracking function

minFaceSize = (30,30)
neigbors = 5

# Shape Classification function
minShapeArea = 500

RL=robotLight.RobotLight()						# Set up LEDs
RL.start()
RL.breath(127,127,255)

laser.setup()									# Set up laser emitter
	
ultrasonic.setup()								# Set up ultrasonic sensor
distance = str(round(ultrasonic.ultdist(),2))			# Reads initial distance


def facetrack(frame):

	# Convert to grayscale
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	# Apply HAAR cascade for face detection
	faces = faceCascade.detectMultiScale(
		gray,
		scaleFactor=1.1,
		minNeighbors=neigbors,
		minSize=minFaceSize,
		flags=cv2.CASCADE_SCALE_IMAGE
	)

	# Draw rectangle around face
	for (x, y, w, h) in faces:
		cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
		targetX = x + (w // 2)
		targetY = y + (h // 2)
		print(len(faces))

	# Track face if any is detected
	if len(faces) > 0:
		cv2.putText(frame, 'Face Detected', (40, 60), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
		laser.laser(0)  # turn laser off
		print(targetX, targetY)
		print(f' Head position: up/down: {move.Up_Down_input} left/right {move.Left_Right_input}')

		# Center face along horizontal bisector
		if targetY < ((resH // 2) - tor):
			error = ((resH // 2) - targetY) / 15
			outv = error
			move.look_up(outv)
			Y_lock = 0
		elif targetY > ((resH // 2) + tor):
			error = (targetY - (resH // 2)) / 15
			outv = error
			move.look_down(outv)
			Y_lock = 0
		else:
			Y_lock = 1

		# Center face along vertical bisector
		if targetX < ((resW // 2) - tor):
			error_X = ((resW // 2) - targetX) / 15
			outv_X = error_X
			move.look_left(outv_X)
			X_lock = 0
		elif targetX > ((resW // 2) + tor):
			error_X = (targetX - (resW // 2)) / 15
			outv_X = error_X
			move.look_right(outv_X)
			X_lock = 0
		else:
			X_lock = 1

	else:
		cv2.putText(frame, 'Searching for Face...', (40, 60), font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
		laser.laser(1)  # turn laser on

def shapeclassify(frame):
		gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
		blur = cv2.GaussianBlur(gray,(7,7),1)
		canny = cv2.Canny(blur,50,50)
		
		contours,hierarchy = cv2.findContours(canny,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[-2:]
    
		for cnt in contours:
			area = cv2.contourArea(cnt)
			print(f'Area of shape:{area}')
			if area > minShapeArea:
				cv2.drawContours(frame, cnt, -1, (255, 0, 0), 2)
				peri = cv2.arcLength(cnt,True)
				
				approx = cv2.approxPolyDP(cnt,0.02*peri,True)
				#print(len(approx))
				objCor = len(approx)
				x, y, w, h = cv2.boundingRect(approx)
 
				if objCor ==3: objectType ="Triangle"
				
				elif objCor == 4:
					aspRatio = w/float(h)
					if aspRatio > 0.95 and aspRatio < 1.05: objectType= "Square"
					else: objectType = "Rectangle"
				
				elif objCor == 5: objectType = "Pentagon"
				elif objCor == 6: objectType = "Hexagon"
				elif objCor == 7: objectType = "Heptagon"
				elif objCor == 8: objectType = "Octagon"
				else:objectType = "Ellipse"
				
				cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),1)
				cv2.putText(frame,objectType, (x+(w//2)-10,y+(h//2)-10),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,255),1, cv2.LINE_AA)
					
def loop():
		# Capture frame for each loop
		ret, frame = video_cap.read()
		if headMode == 'Face Tracking':
			facetrack(frame)
		elif headMode == 'Shape Classifying':
			shapeclassify(frame)
		elif headMode == 'Edge Detection':
			frame, contours = utils.get_contours(frame, showEdge = True, minArea = 2500,
																				filter = 4, draw = False)
			print(f'Objects detected: {len(contours)}')
			
			if len(contours) > 0:
				utils.obs_width(frame, contours[0][3], distance, draw = True)
				print(contours[0][3])
		
		elif headMode == 'Measuring Obstacle':
			frame, contours = utils.get_contours(frame, showEdge = True, minArea = 2500,
																				filter = 0, draw = True)
			print(f'Objects detected: {len(contours)}')
			
			if len(contours) > 0:
				utils.obs_width(frame, contours[0][3], int(float((distance))), draw = True, cm = True)
				print(contours[0][3])

		# Display distance from ultrasonic sensor
		frameDist = 'distance: ' + distance + 'cm'
		cv2.putText(frame, str(frameDist) ,(resW-165,15), font, 0.5,(255,255,255),1,cv2.LINE_AA)

		# Display FPS
		cv2.putText(frame, 'FPS: ' + str(fps), (1, 10), font, 0.3, (0, 255, 0), 1, cv2.LINE_AA)
		
		# Display Head mode
		cv2.putText(frame, 'Mode: ' + str(headMode), (1, resH-20), font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
		
		# Draw crosshair
		cv2.line(frame, (resW//2 - 10, resH//2), (resW//2 + 10, resH//2),
										(0,255,0), 1)
		cv2.line(frame, (resW//2, resH//2 - 10), (resW//2, resH//2 + 10),
										(0,255,0), 1)
		
		
		# Display frame
		frame = cv2.resize(frame,(640, 480))
		cv2.imshow('Hexapod FPV - Sharome Burton', frame)
			
while True:
	
	if frameCounter % 30 == 0:
		dist = ultrasonic.ultdist()		# Estimate distance every 30th frame
		frameCounter = 1							# Reset frame counter
		frameThirtyTime = time.time()	# Store time when 30th frame is processed
		
		# calculates frames/sec using time to process 30 frames
		fps = round(30/(frameThirtyTime-frameZeroTime),2)	
		print(f'FPS: {fps}')																				
		frameZeroTime = frameThirtyTime		
		
		# print current mode
		print(headMode)				

		if dist < 150.0:								# Ignore distances over 1.5m
			distance = str(round(dist,2))	# This is the value passed to the frame loop
		else:
			distance = '9999'				# This is passed when the distance is over 1.5m
		loop()
	
	else:		
		loop()
		
	frameCounter +=1
	
	# Keyboard input
	keyPressed = cv2.waitKey(1) & 0xFF
	
	if keyPressed != 0xFF:
					
		if keyPressed == ord('d'):
				headMode = 'default'
				RL.breath(127,127,255)
				print(headMode)

		elif keyPressed == ord('f'):
				headMode = 'Face Tracking'
				RL.pause()
				RL.setColor(255,255,255)
				RL.both_on()
				print(headMode)

		elif keyPressed == ord('s'):
				headMode = 'Shape Classifying'
				RL.pause()
				RL.setColor(255,255,255)
				RL.both_on()
				print(headMode)
				
		elif keyPressed == ord('e'):
				headMode = 'Edge Detection'
				RL.pause()
				RL.setColor(255,255,255)
				RL.both_on()
				print(headMode)	
				
		elif keyPressed == ord('m'):
				headMode = 'Measuring Obstacle'
				RL.pause()
				RL.setColor(255,255,255)
				RL.both_on()
				print(headMode)	
				
		# Manual head movement
				
		elif keyPressed == 82:
				move.look_up(headMoveSensitivity)
				print('Head Up')		
				
		elif keyPressed == 84:
				move.look_down(headMoveSensitivity)
				print('Head Down')		
				
		elif keyPressed == 81:
				move.look_left(headMoveSensitivity)
				print('Head Left')		
				
		elif keyPressed == 83:
				move.look_right(headMoveSensitivity)
				print('Head Right')									
		
		# Manual body movement
		
		elif keyPressed == ord('i'):
				move.commandInput('forward')
				print('Moving Forward')
		
		elif keyPressed == ord('k'):
				move.commandInput('backward')
				print('Moving Backward')
				
		elif keyPressed == ord('j'):
				move.commandInput('left')
				print('Moving Left')
				
		elif keyPressed == ord('l'):
				move.commandInput('right')
				print('Moving Right')
				
		elif keyPressed == ord('p'):
				move.commandInput('stand')
				print('Stopped Movement')
		
		# Quit
		
		elif keyPressed == ord('q'):
				move.commandInput('stand')
				time.sleep(0.1)
				move.init_all()
				RL.pause()
				laser.destroy()			
				break
	
			
# Release capture
video_cap.release()
cv2.destroyAllWindows()

	