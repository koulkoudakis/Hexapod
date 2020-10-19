import cv2
import numpy as np

# Table for estimating distance

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
    
def obs_width(frame, bounds, dist, draw = False, cm = False):
    if len(bounds) > 0:
        
        xMin = bounds[0]
        xMinCoord = (bounds[0], bounds[1])
        
        xMax = xMin + bounds[2] 
        xMaxCoord = (xMax, bounds[1])
        
        width = xMax - xMin
        
        coords = (xMinCoord,xMaxCoord)
        
        # Draws line connecting points in question
        if draw:
            if len(xMinCoord) == 2 and len(xMinCoord) == 2:
                cv2.line(frame, xMinCoord, xMaxCoord, (255,0,255), 3)
                if cm:
                    if dist >=10 and dist <=50:
                        distance = dist        # Distance to object
                        pixelRatio = pixelToCm.get(str(distance))
                        cmWidth = round(width * 1/pixelRatio, 1)
                        
                        cv2.putText(frame, 'width(cm): ' + str(cmWidth), (bounds[0], bounds[1] - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                else:
                    cv2.putText(frame, 'Target out of range', (bounds[0], bounds[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
            else:
                cv2.putText(frame, 'width(p): ' + str(width), (bounds[0], bounds[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        
        return frame, width, coords
        
    else:
        return frame
