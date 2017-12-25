import cv2
import numpy as np 
import matplotlib.pyplot as plt
import time
import serial 

ser = serial.Serial('/dev/tty.usbmodem131', 9600) # Arduino on serial port change to com for windows


cap = cv2.VideoCapture(1) # attach to web cam. Argument could be 0

color = (0,0,255) #Red circle to identify ball. 
line_width = 2
radius = 5
point = (0,0)

setpoint = 300
errorAdjPrev = 0.0
integ = 0.0
integPrev = float(0.0)

cx = 300;
cy = 0;

Kp = 17.5
Ki = 0.07
Kd = 11.0



while(True):

	ret, frame = cap.read()
	croppedFrame = frame[100:479-250, 0:639] # cropping the frame to help computation
	gray = cv2.cvtColor(croppedFrame, cv2.COLOR_RGB2GRAY)


	thresh = 140
	_, thresh = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)

	'''
	thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, \
		cv2.THRESH_BINARY, 101, 1)
	'''

	_, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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
	cv2.circle(raw, point, radius, color, line_width) # object
	cv2.circle(raw, (setpoint,cy+100), 20, (0,255,0), 3) # setpoint
	cv2.imshow("Original",raw)
	cv2.imshow("Processed",thresh)


	error = (setpoint - cx)
	
	errorAdj = float(error) / 100.0
	
	prop = 1.0 * errorAdj # Forcing float

	integ = (error*0.033) +  integPrev # Forcing float

	deriv = 1.0 * ((errorAdj - errorAdjPrev) / 0.033) # Forcing float

	desAngle = Kp * prop + Ki * integ + Kd * deriv

	desAngleAbs = 79 + int(desAngle)

	if desAngleAbs > 126:
		desAngleAbs = 126
	if desAngleAbs < 32:
		desAngleAbs = 32

	errorAdjPrev = errorAdj
	integPrev = integ

	ser.write(str(unichr(desAngleAbs)))
	print('Set: {0:3d} Mes: {1:3d} DesA: {2:.2f}  Prop: {3:.2f}  Integ: {4:.2f}  Deriv: {5:.2f}' \
		.format(setpoint, cx, desAngle, Kp * prop, Ki * integ, Kd * deriv))


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


