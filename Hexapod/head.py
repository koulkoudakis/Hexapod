import cv2
import threading
import numpy as np
import sys
import os
import time

from rpi_ws281x import *

import robotLight		# LED control

import body			# Servo control, locomotion

import peripherals	# Inertial measurement units, ultrasonic distance
			# sensor, laser emitter, etc.
										
import utils		# functions for computer vision, logic


headMode = 'Default'
headMoveSensitivity = 5	# Manual head movement

# Movement variables
global direction, angle, destination
global smooth
smooth = 0 	# Smooth body servo mode
global checkMove
checkMove = 0	# Check movement cycle
global moveCycles
moveCycles = 0	# Complete movement cycles

# Face classifier cascade file
cascPath = 'haarcascade_frontalface_default.xml'
faceCascade = cv2.CascadeClassifier(cascPath)

font = cv2.FONT_HERSHEY_SIMPLEX

resW = 480		# Resolution width and
resH = (resW//4)*3	# Height	(aspect ratio must be 4:3)

frameCounter = 1
frameZeroTime = time.time()
fps = 0		# Initial frames/second feedback

# Face Tracking function
minFaceSize = (30,30)	
neigbors = 5	# Minimum nearest neighbors for positive identification

Y_lock = 0		# Target is centered in Y direction
X_lock = 0		# Target is centered in X direction
tor = 15			# Error tolerance for centering camera

# Obstacle measurement function
widthScale = resW/480.0

'''
Instantiation
'''
# Video capture object
video_cap = cv2.VideoCapture(0)
video_cap.set(3,resW)
video_cap.set(4,resH)
video_cap.set(15,2)

# LED object
RL = robotLight.RobotLight()
RL.start()
RL.breath(127,127,255)	# RGB - light blue

# Internal inertial measurement unit object
imu = peripherals.AccGyro('IMU', 1)
imu.read()
imuData = str('X=%f,Y=%f,Z=%f' % (imu.accel['x'], imu.accel['y'],
                                  imu.accel['z']))
                                  
# External inertial measurement unit object
compass = peripherals.exIMU('Compass', 1)
heading = str(compass.heading) # 0-360 degrees

# Laser emitter object
laser = peripherals.Laser('Laser Emitter', 1)	
	
# Ultrasonic sensor object
ultra = peripherals.Ultrasonic('Ultrasonic Sensor', 1)
distance = str(round(ultra.ultdist(),2))	# Reads initial distance

# Minimap object
minimap = utils.Map(400,400,initHeading=compass.heading)
minimap.draw_map()

def check_move():
	global checkMove
	global moveCycles
	
	if body.Hexapod.step_set !=1:
		checkMove = 1
			
	if checkMove:
		if body.Hexapod.step_set == 1:
			checkMove = 0
			moveCycles += 1

'''
Hexapod rotates at 15 degrees per move cycle
'''
def rotate(direction, angle, rest=1):
	global moveCycles
	moveCycles = 0
	global smooth
	smooth = 1
	
	angle = abs(angle)
	lowerAngle = (angle//15)*15
	upperAngle = ((angle//15)*15) + 15
	rotateCycles = (angle//15)
	
	body.commandInput('automatic',body.Hexapod)
	
	time.sleep(0.01)
	if direction:	# Rotate right
		print(f'Rotating right...{rotateCycles} cycles')
		print(f'lower: {lowerAngle} upper: {upperAngle}')
		
		if angle >= lowerAngle and angle < upperAngle:
			body.commandInput('right',body.Hexapod)
			while moveCycles != rotateCycles:
				check_move()
				time.sleep(0.01)
			print(f'Rotation: {moveCycles*15} degrees right')
			body.commandInput('stand',body.Hexapod)
			moveCycles = 0
			
	else:	# Rotate left
		print(f'Rotating left...{rotateCycles} cycles')
		
		if angle >= lowerAngle and angle < upperAngle:
			body.commandInput('left',body.Hexapod)
			while moveCycles != rotateCycles:
				check_move()
				time.sleep(0.01)
			print(f'Rotation: {moveCycles*15} degrees left')
			body.commandInput('stand',body.Hexapod)
			moveCycles = 0
			
	time.sleep(rest)


'''
Hexapod moves forward at 4 cm per move cycle
'''	
def move_distance(direction, destination, rest=1):
	global moveCycles
	moveCycles = 0
	smooth = 1
	body.commandInput('automatic',body.Hexapod)
	steps = destination // 4
	time.sleep(0.01)
	
	if direction:
		print(f'Moving forward...{steps} steps')
		body.commandInput('forward',body.Hexapod)
		while moveCycles < steps:
			time.sleep(0.01)
			check_move()
		body.commandInput('stand',body.Hexapod)
		print(f'Distance moved: {moveCycles*4} cm')
		moveCycles = 0
	
	else:
		print(f'Moving backward...{steps} steps')
		body.commandInput('backward',body.Hexapod)	
		while moveCycles < steps:
			time.sleep(0.01)
			check_move()
		body.commandInput('stand',body.Hexapod)
		distMoved = moveCycles*4
		print(f'Distance moved: {distMoved} cm')
		moveCycles = 0
	time.sleep(rest)
	
def loop():
		global direction, angle, destination
		# Capture frame for each loop, raw frame can be used outside
		ret, frame = video_cap.read()
		
		# Check current mode of first-person-view
		if headMode == 'Face Tracking':
			utils.face_track(frame,body.Hexapod,laser,resW,resH)
			
		elif headMode == 'Shape Classifying':
			utils.shapeclassify(frame, showEdge = True)
			
		elif headMode == 'Edge Detection':
			frame, contours = utils.get_contours(
				frame,
				showEdge = True,
				minArea = 2500,
				filter = 4,
				draw = False
				)
			print(f'Objects detected: {len(contours)}')
			
			if len(contours) > 0:
				utils.obs_width(frame,
				contours[0][3],
				int(float((distance))),
				widthScale,
				draw = True
				)
				#print(contours[0][3])
		
		elif headMode == 'Measuring Obstacle':
			frame, contours = utils.get_contours(
				frame,
				showEdge = True,
				minArea = 2500,
				filter = 0,
				draw = True
				)
			print(f'Objects detected: {len(contours)}')
			
			if len(contours) > 0:
				utils.obs_width(
					frame,
					contours[0][3],
					int(float((distance))),
					widthScale,
					draw = True,
					cm = True
					)
				#print(contours[0][3])
				
		elif headMode == 'Navigating':
			frame, contours = utils.get_contours(
					frame,
					showEdge = True,
					minArea = 2500,
					filter = 0,
					draw = True
					)
			print(f'Objects detected: {len(contours)}')
			
			if len(contours) > 0:
				cmWidth = utils.obs_width(
					frame,
					contours[0][3],
					int(float((distance))),
					widthScale,
					draw = True, cm = True
					)[1]
				#print(contours[0][3])
				direction, angle, destination = utils.find_way(
					frame,
					contours[0][-2],
					int(float(distance)),
					cmWidth,
					resW//2,
					resW,
					resH,
					show=True
					)
				if direction: dirLR = 'right'
				else: dirLR = 'left'
				
				#print(f'Rotate {angle} degrees {dirLR} then move {destination} cm forward')

		# Display distance from ultrasonic sensor
		frameDist = 'distance: ' + distance + 'cm'
		cv2.putText(frame, str(frameDist) ,(resW-165,15), font, 0.5,
		(255,255,255),1,cv2.LINE_AA)

		# Display FPS
		cv2.putText(frame, 'FPS: ' + str(fps), (1, 10), font, 0.3,
		 (0, 255, 0), 1, cv2.LINE_AA)
		
		# Display Head mode
		cv2.putText(frame, 'Mode: ' + str(headMode), (1, resH-20),
		 font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
		
		# Draw crosshair
		cv2.line(
			frame,
			(resW//2 - 10, resH//2),
			(resW//2 + 10, resH//2),
			(0,255,0),
			1
			)
		cv2.line(
			frame,
			(resW//2, resH//2 - 10),
			(resW//2, resH//2 + 10),
			(0,255,0),
			1
			)
		
		# Add blank area to FPV window
		blank = np.zeros((resH//4,resW,3), dtype=np.uint8)
		frame = np.vstack((frame, blank))
		
		# Display IMU data
		cv2.putText(frame, imuData, (1, resH+(resH//4)-20), font, 0.5,
		 (255, 255, 255), 1, cv2.LINE_AA)
		 
		# Display heading
		cv2.putText(frame, 'Heading: '+ heading, (1, resH+(resH//4)-40),
		 font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
		
		# Display frame
		frame = cv2.resize(frame,(640, 640))
		cv2.imshow('Hexapod FPV - Sharome Burton', frame)
		
		
while True:
	
	if frameCounter % 30 == 0:
		dist = ultra.ultdist()	# Estimate distance every 30th frame
		frameCounter = 1	# Reset frame counter
		frameThirtyTime = time.time()	# Store time when 30th frame is processed
		
		# calculates frames/sec using time to process 30 frames
		fps = round(30/(frameThirtyTime-frameZeroTime),2)
		print(f'FPS: {fps}')
		frameZeroTime = frameThirtyTime
		
		# Read IMU
		imu.read()
		imuData = str('X=%f,Y=%f,Z=%f' % (imu.accel['x'], imu.accel['y'],
                                  imu.accel['z']))
		compass.read() # Read compass data
		heading = str(compass.heading)
				
		#minimap.update_map([200,200])
		#minimap.draw_map()
		
		if dist < 150.0: # Ignore distances over 1.5m
			distance = str(round(dist,2))	# This is the value passed to the frame loop
		else:
			distance = '9999'# This is passed when the distance is over 1.5m
		loop()
	
	else:		
		loop()
		
	frameCounter +=1
	#print(body.Hexapod.step_set)
	
	# Check movement cycle
	
	if body.Hexapod.step_set !=1:
		checkMove = 1
			
	if checkMove:
		if body.Hexapod.step_set == 1:
			checkMove = 0
			moveCycles += 1
			print(f'Movement cycles: {moveCycles}, Distance covered: {moveCycles * 4} cm')	
	
	
	# Keyboard input
	# Stores only last 8 bits (ASCII) of 32-bit key input
	keyPressed = cv2.waitKey(1) & 0xFF  
																		
	
	if keyPressed != 0xFF:
					
		if keyPressed == ord('d'):
				headMode = 'default'
				RL.breath(127,127,255)
				print(headMode)
				
		elif keyPressed == ord('b'):
				headMode = 'default'
				RL.pause()
				RL.setColor(255,255,255)
				RL.both_on()
				print('default - breathing off')

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
				
		elif keyPressed == ord('n'):
				headMode = 'Navigating'
				RL.pause()
				RL.setColor(255,255,255)
				RL.both_on()
				print(headMode)
				
		elif keyPressed == ord('t'):
				print('Looking for text...')
				# Input high res photo for text recognition
				img = utils.photo(
					video_cap,
					1920,1440,
					resW,
					resH,
					show = False
					)
				text = utils.text_read(
					img,
					img.shape[1],
					img.shape[0],
					prep = 't'
					)
				if text != None:
					print('Text shown above was read')
					
				else:
					print('Text not detected')
				
		elif keyPressed == ord('u'):
			if headMode == 'Navigating':
				print('Testing movement')
				print(f'Rotate {angle} degrees {direction} then move {destination} cm forward')
				angle=((((angle-1)//15)+2)*15)  
				rotate(direction,angle)
				time.sleep(0.1)
				move_distance(1,destination+8)
				minimap.update_map(
					minimap.find_delta(direction,angle,destination),
					obj=True,
					objDist=dist
					)
				minimap.draw_map()
				
				time.sleep(0.1)
				rotate((direction-1)*-1,angle)
				time.sleep(0.1)
				move_distance(1,32)
				minimap.update_map(
					minimap.find_delta((direction-1)*-1,angle,32),
					obj=False,
					)
				minimap.draw_map()
				
				time.sleep(0.1)
				rotate((direction-1)*-1,90)
				time.sleep(0.1)
				move_distance(1,32)
				minimap.update_map(
					minimap.find_delta((direction-1)*-1,89,32),
					obj=False,
					)
				minimap.draw_map()
				
				time.sleep(0.1)
				rotate((direction-1)*-1,88)
				time.sleep(0.1)
				move_distance(1,32)
				minimap.update_map(
					minimap.find_delta((direction-1)*-1,88,32),
					obj=False,
					)
				minimap.draw_map()
				
			else:
				rotate(0,90)
				time.sleep(0.1)
				move_distance(1,8)
				minimap.update_map(minimap.find_delta(0,88,8))
				minimap.draw_map()
				print('Testing movement')
				
		# Manual head movement
				
		elif keyPressed == 82:
				body.Hexapod.look_up(headMoveSensitivity)
				print('Head Up')		
				
		elif keyPressed == 84:
				body.Hexapod.look_down(headMoveSensitivity)
				print('Head Down')		
				
		elif keyPressed == 81:
				body.Hexapod.look_left(headMoveSensitivity)
				print('Head Left')		
				
		elif keyPressed == 83:
				body.Hexapod.look_right(headMoveSensitivity)
				print('Head Right')									
		
		# Manual body movement
		
		elif keyPressed == ord('i'):
				body.commandInput('forward',body.Hexapod)
				print('Moving Forward')
		
		elif keyPressed == ord('k'):
				body.commandInput('backward',body.Hexapod)
				print('Moving Backward')
				
		elif keyPressed == ord('j'):
				body.commandInput('left',body.Hexapod)
				print('Moving Left')
				
		elif keyPressed == ord('l'):
				body.commandInput('right',body.Hexapod)
				print('Moving Right')
				
		elif keyPressed == ord('o'):
				if smooth:
					body.commandInput('automaticOff',body.Hexapod)
					smooth = 0
					print('Smooth Mode off')
				else:
					smooth = 1
					body.commandInput('automatic',body.Hexapod)
					print('Smooth Mode on')
				
		elif keyPressed == ord('p'):
				body.commandInput('stand',body.Hexapod)
				moveCycles = 0
				print('Stopped Movement')
		
		# Quit
		
		elif keyPressed == ord('q'):
				body.commandInput('stand',body.Hexapod)
				time.sleep(0.1)
				body.Hexapod.init_all()
				RL.pause()		
				break
	
			
# Free resources
video_cap.release()
ultra.setup(0)
laser.setup(0)
imu.setup(0)
compass.setup(0)
cv2.destroyAllWindows()
