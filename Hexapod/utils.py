import math
import cv2
import numpy as np
import time

import pytesseract # optical text recognition

font = cv2.FONT_HERSHEY_COMPLEX

# Face Tracking function

cascPath = 'haarcascade_frontalface_default.xml'
faceCascade = cv2.CascadeClassifier(cascPath)

minFaceSize = (30,30)	
neigbors = 5	# Minimum nearest neighbors for positive identification
Y_lock = 0	# Target is centered in Y direction
X_lock = 0	# Target is centered in X direction
tor = 15	# Error tolerance for centering camera

dps = 2 # Distance per step (cm)
aps = 6 # angle per step (deg)

# Table for estimating object width (distance: pixels per cm)

pixelToCm = {   "9":  65.00,
                "10": 56.00,
                "11": 49.06,
                "12": 43.12,
                "13": 36.56,
                "14": 33.50,
                "15": 31.15,
                "16": 29.71,
                "17": 28.00,
                "18": 26.03,
                "19": 24.52,
                "20": 23.77,
                "21": 22.95,
                "22": 22.13,
                "23": 21.31,
                "24": 20.48,
                "25": 19.67,
                "26": 18.92,
                "27": 18.16,
                "28": 17.41,
                "29": 16.65,
                "30": 15.90,
                "31": 15.54,
                "32": 15.18,
                "33": 14.82,
                "34": 14.46,
                "35": 14.10,
                "36": 13.73,
                "37": 13.37,
                "38": 13.01,
                "39": 12.65,
                "40": 12.29,
                "41": 11.85,
                "42": 11.41,
                "43": 10.96,
                "44": 10.52,
                "45": 10.08,
                "46": 9.64,
                "47": 9.20,
                "48": 8.75,
                "49": 8.31,
                "50": 7.87,
                "51": 7.42
            
}

class Map:
	#scaleFactor =  # every centimetre correlates to 5 pixels on map
	
	def __init__(self,resW,resH,scaleFactor=2.0,initHeading=0):
		self.resW = resW
		self.resH = resH
		self.map = np.zeros((resH,resW,3), dtype=np.uint8)
		self.coords = [[resW//2,resH//2],[resW//2,resH//2]]
		self.lastCenter = [resW//2,resH//2]
		self.newCenter = [resW//2,resH//2]
		# every centimetre correlates to X pixels on map
		self.scaleFactor = scaleFactor
		# add 90 degrees to make North correlate to 0 degrees
		# in polar coordinates
		self.heading=initHeading
		self.arrowHead=math.radians(self.heading)
		self.arrow=[
			[self.resW//2,self.resH//2],
			[int(20*math.cos(self.arrowHead-((math.pi)/2))+self.resW//2),
			int(20*math.sin(self.arrowHead-((math.pi)/2))+self.resH//2)]]
		self.objCoords=[] # obstacles detected positions
		self.wayp=[] # waypoints
		self.wayDist = 0.0 # Distance(cm) to waypoint
		self.wayHeading = 0.0 # heading (0-359 deg)
		self.goal=[] # ultimate destination
		self.goalHeading = 0.0 # heading (0-359 deg)
		self.goalDist = 0.0 # Distance(cm) to ultimate destination
		
		print(f'Minimap initialized, initial heading:{initHeading}')
		

	def draw_map(self,colorC=(0,0,255),colorL=(255,0,0),colorO=(255,255,255)):
		self.map = np.zeros((self.resH,self.resW,3), dtype=np.uint8)
		i=0 # Path line counter
		
		deltaX = self.lastCenter[0] - self.newCenter[0]
		deltaY = self.lastCenter[1] - self.newCenter[1]
		
		# Body is at central point
		cv2.circle(self.map,
			tuple((self.newCenter[0]+deltaX,self.newCenter[1]+deltaY)),
			5,
			(255,0,255),
			-1)
		# Arrow points to direction body is facing
		cv2.line(self.map,
			tuple(self.arrow[0]),
			tuple(self.arrow[1]),
			(0,255,255),
			2)
		print(f'Current heading: {math.degrees(self.arrowHead)}')
		
		# Draws waypoints
		if len(self.wayp) != 0:
			waypoint = 1
			for w in self.wayp:
				w[0]+=deltaX
				w[1]+=deltaY
				if w != [self.resW//2,self.resH//2]:
					cv2.circle(self.map,
						tuple(w),
						5,
						(0,255,0),
						-1
						)
					cv2.putText(self.map,
						str(waypoint),
						(w[0]+1,w[1]-1),
						font,
						0.5,
						(50,50,255),
						2
						)
					waypoint+=1
					
			# Update distance, bearing to next waypoint
			self.wayDist,self.wayHeading = calc_bearing(
					[self.resW//2,self.resH//2],
					self.wayp[0])
			print(f'Bearing to next waypoint:{self.wayHeading}')
			self.wayDist = self.wayDist/self.scaleFactor
			print(f'Distance to next waypoint:{self.wayDist}')
			
			# Removes waypoint if reached
			xTol = abs(self.wayp[0][0]-self.resW//2)
			yTol = abs(self.wayp[0][1]-self.resH//2)
			if xTol <= 3.5*self.scaleFactor and yTol <= 3.5*self.scaleFactor:
				self.wayp.pop(0)
				print('WAYPOINT REACHED')
				
		# Draws ultimate destination
		if len(self.goal) != 0:
			self.goal[0]+=deltaX
			self.goal[1]+=deltaY
			cv2.circle(self.map,tuple(self.goal),8,(0,255,0),2)
			
			# Update distance, bearing to destination
			self.goalDist,self.goalHeading = calc_bearing(
					[self.resW//2,self.resH//2],
					self.goal)
			print(f'Bearing to destination:{self.goalHeading}')
			self.goalDist = self.goalDist/self.scaleFactor
			print(f'Distance to destination:{self.goalDist}')
		
		# Draws objects detected along the way
		if len(self.objCoords) != 0:
			for o in self.objCoords:
				o[0]+=deltaX
				o[1]+=deltaY
				cv2.circle(self.map, tuple(o), 5, colorO, -1)
				
		# Draws path as red points connected by white lines
		for c in self.coords:
			#print(tuple(c))
			c[0]+=deltaX
			c[1]+=deltaY
			
			# Points to draw linear path
			if i !=0: pointA = self.coords[i-1]
			else: pointA = self.coords[i]
			pointB = c
			#print(pointA,pointB)
			
			i+=1
			
			if c[0]>=0 and c[0]<=self.resW and c[1]>=0 and c[1]<=self.resH:
				cv2.line(self.map,
					tuple(pointA),
					tuple(pointB),
					(255,255,255),
					1)
				cv2.circle(self.map, tuple(c), 3, colorC, -1)
				
		cv2.imshow('Minimap', self.map)
		#print(self.coords)
	
	def update_map(self,coords,obj=False,objDist=20.0):
		self.lastCenter = self.coords[0]
		self.newCenter = coords
		self.coords.insert(0,self.newCenter)
		if obj:
			self.objCoords.append([int(objDist*self.scaleFactor*math.cos(self.arrowHead-(math.pi)/2)+self.resW//2),
			int(objDist*self.scaleFactor*math.sin(self.arrowHead-(math.pi)/2)+self.resH//2)]
			)
		
	def find_delta(self,direction,rotation,distance):
		coords = []
		distance=float(distance)*self.scaleFactor
		#rotation=(((rotation-1)//15)+1)*15
		
		if direction:
			angle=rotation*1
		else: 
			angle=rotation*-1
		
		bearing=float(self.heading-90+angle)
		self.heading+=angle # New heading for body
		self.arrowHead+=math.radians(angle)
		self.arrow[1]=[int(20*math.cos(self.arrowHead-((math.pi)/2))+self.resW//2),
			int(20*math.sin(self.arrowHead-((math.pi)/2))+self.resH//2)]
		
		deltaX=int(distance*math.cos(math.radians(bearing)))
		print(f'deltaX:{deltaX}')
		deltaY=int(distance*math.sin(math.radians(bearing)))
		print(f'deltaY:{deltaY}')
		
		coords=[int((self.resW//2)+deltaX),int((self.resH//2)+deltaY)]
		
		return coords
		

# Face tracking using head servos
def face_track(frame,body,laser,resW,resH):

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
		print(f'Faces detected: {len(faces)}')

	# Track face if any is detected
	if len(faces) > 0:
		cv2.putText(frame, 'Face Detected', (40, 60), font, 0.5,
		 (255, 255, 255), 1, cv2.LINE_AA)
		laser.laser(0)  # turn laser off
		print(targetX, targetY)
		print(f' Head position: up/down: {body.Up_Down_input} left/right {body.Left_Right_input}')

		# Center face along horizontal bisector
		if targetY < ((resH // 2) - tor):
			error = ((resH // 2) - targetY) / 15
			outv = error
			body.look_up(outv)
			Y_lock = 0
		elif targetY > ((resH // 2) + tor):
			error = (targetY - (resH // 2)) / 15
			outv = error
			body.look_down(outv)
			Y_lock = 0
		else:
			Y_lock = 1

		# Center face along vertical bisector
		if targetX < ((resW // 2) - tor):
			error_X = ((resW // 2) - targetX) / 15
			outv_X = error_X
			body.look_left(outv_X)
			X_lock = 0
		elif targetX > ((resW // 2) + tor):
			error_X = (targetX - (resW // 2)) / 15
			outv_X = error_X
			body.look_right(outv_X)
			X_lock = 0
		else:
			X_lock = 1

	else:
		cv2.putText(frame, 'Searching for Face...', (40, 60), font, 0.5,
		 (0, 0, 255), 1, cv2.LINE_AA)
		laser.laser(1)  # turn laser on

# Returns closed contours with a specified number of corners and a minimum area
def get_contours(frame, cannyThr = [100,100], showEdge = False,
                    minArea = 5000, filter = 0, draw = False):
            
    frameGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frameBlur = cv2.GaussianBlur(frameGray,(5,5),1)
    frameCanny = cv2.Canny(frameBlur,cannyThr[0],cannyThr[1])
    kernel = np.ones((5,5))
    frameDilate = cv2.dilate(frameCanny, kernel, iterations = 2)
    frameThreshold = cv2.erode(frameDilate, kernel, iterations= 2)
    if showEdge: cv2.imshow('Edge detection', frameThreshold)

    contours, hierarchy = cv2.findContours(frameThreshold,
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]
        
    finalContours = []
    
    if len(contours) > 0:        
        for i in contours:
            area = cv2.contourArea(i)
            if area > minArea:
                peri = cv2.arcLength(i, True)   # perimeter
                corners = cv2.approxPolyDP(i, 0.02*peri, True)  # corner points
                bounds = cv2.boundingRect(corners)  # bounding box
                if filter > 0:
                    if len(corners) == filter:
                        finalContours.append([len(corners), area, corners, bounds, i])
                else:
                    finalContours.append([len(corners), area, corners, bounds, i])

        # Sorts closed contours according to size (biggest first)
        finalContours = sorted(finalContours, key = lambda x:x[1], reverse = True)

        # Draws contours onto image
        if draw:
            for contours in finalContours:
                cv2.drawContours(frame, contours[4], -1, (0,0,255), 3)

        return frame, finalContours
    
    else: 
        return frame, finalContours

def pixel_width_hor(frame, contours, draw = False):
    if len(contours) > 0:
        #print(contours)
        xMin = contours[0][0][0]
        xMinCoord = contours[0]
        #print(xMin)
        xMax = contours[0][0][0]
        xMaxCoord = contours[0]
        #print(xMax)
        
        for cnt in contours:
            # print(cnt)
            if cnt[0][0] < xMin:
                xMin = cnt[0][0]
                xMinCoord = cnt[0]
            elif cnt[0][0] > xMax:
                xMax = cnt[0][0]
                xMaxCoord = cnt[0]
            
        width = xMax - xMin
        
        xMinCoord = tuple(xMinCoord)    # Leftmost pixel
        xMaxCoord = tuple(xMaxCoord)    # Rightmost pixel
        coords = (xMinCoord,xMaxCoord)
        
        # Draws points in question
        if draw:
            if len(xMinCoord) == 2 and len(xMinCoord) == 2: 
                cv2.circle(frame, xMinCoord, 5, (255,0,255), -1)
                cv2.circle(frame, xMaxCoord, 10, (255,0,255), -1)
        
        return frame, width, coords
        
    else:
        return frame
    
def obs_width(frame, bounds, dist, widthScale, draw = False, cm = False):
    if len(bounds) > 0:
        
        xMin = bounds[0]
        xMinCoord = (bounds[0], bounds[1])
        
        xMax = xMin + bounds[2] 
        xMaxCoord = (xMax, bounds[1])
        
        width = xMax - xMin
        cmWidth = 1.0
        
        coords = (xMinCoord,xMaxCoord)
        
        # Draws line connecting points in question
        if draw:
            if len(xMinCoord) == 2 and len(xMinCoord) == 2:
                cv2.line(frame, xMinCoord, xMaxCoord, (255,0,255), 3)
                if cm:
                        if dist >=10 and dist <=50:
                                distance = dist        # Distance to object
                                pixelRatio = pixelToCm.get(str(distance))
                                cmWidth = round(width * widthScale/pixelRatio, 1)
                        
                                cv2.putText(frame, 'width(cm): ' + str(cmWidth), (bounds[0], bounds[1] - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                        
                        else:
                                cv2.putText(frame, 'Target out of range', (bounds[0], bounds[1] - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                else:
                        cv2.putText(frame, 'width(p): ' + str(width), (bounds[0], bounds[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        
        return frame, cmWidth, coords
        
    else:
        return frame
 
def photo(cap,pResW,pResH,oResW,oResH,show=True,save=False):
	cap.set(3,pResW)
	cap.set(4,pResH)
	ret, img = cap.read()
	if ret:
		if show:
                        display = cv2.resize(img,(640, 480))
                        cv2.imshow('Screen Cap', display)
		if save:
			pass
                
                # Reset video capture to original resolution
		cap.set(3,oResW)
		cap.set(4,oResH)
		
		return img
                
	else:
		cap.set(3,oResW)
		cap.set(4,oResH)
		return None

def find_way(frame,bounds,distance,obsWidth,midpoint,resW,resH,show=False):
	print(bounds)
	pixWidth = bounds[2]
	if pixWidth == 0: pixWidth = 1
	cmLeftDist = ((float(midpoint - bounds[0]))/pixWidth)*obsWidth
	cmRightDist = ((float(bounds[0] + bounds[2] - midpoint))/pixWidth)*obsWidth
	
	
	
	if (cmLeftDist < cmRightDist):
		direction = 0
		pheta = math.atan(cmLeftDist/distance)
		if pheta == 0: pheta = 0.001
		if pheta > math.pi: pheta = (math.pi / 2)-0.001
		destination = int(4 + (cmLeftDist/math.sin(pheta)))
		angle = int(math.degrees(pheta))
		
	elif (cmRightDist <= cmLeftDist):
		direction = 1
		pheta = math.atan(cmRightDist/distance)
		if pheta == 0: pheta = 0.001
		if pheta > math.pi: pheta = (math.pi / 2)-0.001
		destination = int(4 + (cmRightDist/math.sin(pheta)))
		angle = int(math.degrees(pheta))
		
	if show:
		if direction:
			endX = int((resW)*math.sin(pheta))
			endY = int((resH/4)*math.cos(pheta))
		
		else:	
			endX = int((resW)*math.sin(pheta*(-1)))
			endY = int((resH/4)*math.cos(pheta))
			
		cv2.line(
			frame,
			(resW//2, resH),
			((resW//2)+endX,(resH//2)-endY),
			(0,255,255),
			2
			)
		
	return direction, angle, destination
	
def calc_bearing(center, destination):
	deltaX = float(destination[0] - center[0])
	deltaY = float(center[1] - destination[1])
	wayDist = math.sqrt((deltaY ** 2)+(deltaX ** 2))
	
	pheta = math.atan2(deltaY,deltaX)
	bearing = (math.degrees(pheta) - 90.0)*-1
	
	if bearing < 0:
		return wayDist,bearing + 360 # 'corrects' negative angles
	else:
		return wayDist,bearing
	

def shapeclassify(frame, minShapeArea = 2500, showEdge = False):
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray,(7,7),1)
        canny = cv2.Canny(blur,50,50)
        if showEdge: cv2.imshow('Shape Edge Detection', canny)
        
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
                                if aspRatio > 0.95 and aspRatio < 1.05: 
                                        objectType= "Square"
                                else: objectType = "Rectangle"
                        
                        elif objCor == 5: objectType = "Pentagon"
                        elif objCor == 6: objectType = "Hexagon"
                        elif objCor == 7: objectType = "Heptagon"
                        elif objCor == 8: objectType = "Octagon"
                        else:objectType = "Ellipse"
                        
                        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),1)
                        cv2.putText(frame,objectType, (x+(w//2)-10,y+(h//2)-10),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,255),1, cv2.LINE_AA)

def text_read(frame, resW, resH, prep = 't', prt = True, show = True):
        #print(frame.shape)
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        if prep == 't':
                processed = cv2.threshold(gray, 0, 255,
                        cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
        elif prep == 'b': 
                processed = cv2.medianBlur(gray, 3)
                
        read_string = pytesseract.image_to_string(frame)
        
        if prt and read_string != None:
                print(f'\n {read_string} \n')
                
        if show and read_string != None:
                data = pytesseract.image_to_data(processed)
                print(data)
                for i,d in enumerate(data.splitlines()):
                        if i!=0:
                                d = d.split()    
                                if len(d) == 12:
                                        x,y,w,h = int(d[6]),int(d[7]),int(d[8]),int(d[9])
                                        cv2.rectangle(frame,(x,y),(x+w,h+y),(0,0,255),2)
                                        cv2.putText(frame,d[11],(x,y),font,1,(50,50,255),2)

                frame = cv2.resize(frame,(640,480))
                processed = cv2.resize(processed,(640,480))
                cv2.imshow('Text Detection',frame)
                cv2.imshow('Thresholded Image',processed)

        return read_string
	
if __name__ == "__main__":
	MAP = Map(400,400)

	while True:
		MAP.update_map(MAP.find_delta(1,90,8))
		#MAP.update_map([190,190])
		MAP.draw_map()
		#MAP.update_map([225,175])
		MAP.update_map(MAP.find_delta(0,0,0))
		MAP.draw_map()
		
		time.sleep(1)
		cv2.waitKey(0)
