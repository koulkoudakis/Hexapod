import cv2
import sys
import time

cascPath = '/home/pi/adeept_raspclaws/server/haarcascade_frontalface_default.xml'
faceCascade = cv2.CascadeClassifier(cascPath)

video_cap = cv2.VideoCapture(0)
time.sleep(1)

while True:
		# Capture frame for each loop
		ret, frame = video_cap.read()
		#print(frame)
		#print(ret)
		#print(video_cap.isOpened())
		
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		
		faces = faceCascade.detectMultiScale(
			frame,
			scaleFactor=1.1,
			minNeighbors=5,
			minSize=(30, 30),
			flags=cv2.CASCADE_SCALE_IMAGE
		)
		
		# Draw rectangle
		for (x, y, w, h) in faces:
			cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
			
		# Display frame
		cv2.imshow('Video', frame)
		
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
			
			
# Release capture
video_cap.release()
cv2.destroyAllWindows()

	
