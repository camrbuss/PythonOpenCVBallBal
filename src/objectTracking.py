import cv2
import numpy as np 
import matplotlib.pyplot as plt
import time
import serial 

ser = serial.Serial('/dev/tty.usbmodem131', 9600)


cap = cv2.VideoCapture(1)

color = (255,0,255)
line_width = 2
radius = 5
point = (0,0)

setpoint = 300
errorAdjPrev = 0.0
integ = 0.0
integPrev = float(0.0)

cx = 300;



while(True):

	ret, frame = cap.read()
	croppedFrame = frame[100:479-250, 0:639]
	gray = cv2.cvtColor(croppedFrame, cv2.COLOR_RGB2GRAY)

	'''hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	h = hsv[:,:,0]
	s = hsv[:,:,1]
	v = hsv[:,:,2]
	ret, thresh = cv2.threshold(s, 20, 255, cv2.THRESH_BINARY)
	'''

	thresh = 140
	_, thresh = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)

	'''
	thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, \
		cv2.THRESH_BINARY, 101, 1)
	'''

	_, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	#_, numshapes, _, = hierarchy.shape
	#print("Number of contours",numshapes)


	for c in contours:
		area = cv2.contourArea(c)

		#print("Area:",area)

		if 1500 < area < 4800: # Golf ball is 2500-4800
			M = cv2.moments(c)
			#print(M)
			cx = int( M['m10']/M['m00'] )
			cy = int( M['m01']/M['m00'] )
			#print("MX",M['m10'])  potential to analyze circularity of object
			#print("MY",M['m01'])
			#if  < area < 3800:
			point = (cx,cy+100)
			#print("POINT", point)
			break

	raw = cv2.resize(frame, (0,0), fx=1,fy=1)
	cv2.circle(raw, point, radius, color, line_width)

	cv2.imshow("Original",raw)
	cv2.imshow("Processed",thresh)


	error = (setpoint - cx)
	errorAdj = float(error) / 100.0
	
	# P = 13 I = 0.25 D = 13
	prop = 19.0 * errorAdj 

	integ = (0.3 * (error*0.033))+  integPrev 
	print(error)
	print(integPrev)
	deriv = 13.0 * ((errorAdj - errorAdjPrev) / 0.033)

	desAngle = prop + integ + deriv

	desAngleAbs = 79 + int(desAngle)

	if desAngleAbs > 126:
		desAngleAbs = 126
	if desAngleAbs < 32:
		desAngleAbs = 32

	errorAdjPrev = errorAdj
	integPrev = integ

	ser.write(str(unichr(desAngleAbs)))
	print('Set: {0:3d} Mes: {1:3d} DesA: {2:.2f}  Prop: {3:.2f}  Integ: {4:.2f}  Deriv: {5:.2f}' \
		.format(setpoint, cx, desAngle, prop, integ, deriv))


	ch = cv2.waitKey(1)
	if ch & 0xFF == ord('q'): # quit if q is pressed
		break
	if ch & 0xFF == 53: # 5 on the keyboard
		setpoint = 300
	if ch & 0xFF == 49: # 1 on the keyboard
		setpoint = 200
	if ch & 0xFF == 57: # 9 on the keyboard
		setpoint = 400

cap.release()
#cv2.waitKey(0)

cv2.destroyAllWindows() 


#ser.write(str(unichr(92)))
#time.sleep(1)

