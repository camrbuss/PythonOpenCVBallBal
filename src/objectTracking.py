import cv2
import numpy as np 
import matplotlib.pyplot as plt

cap = cv2.VideoCapture(1)

color = (255,0,255)
line_width = 2
radius = 5




while(True):

	ret, frame = cap.read()
	
	gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

	'''hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	h = hsv[:,:,0]
	s = hsv[:,:,1]
	v = hsv[:,:,2]
	ret, thresh = cv2.threshold(s, 20, 255, cv2.THRESH_BINARY)
	'''

	thresh = 80
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

		if 10000 < area < 15000:
			M = cv2.moments(c)
			#print(M)
			cx = int( M['m10']/M['m00'] )
			cy = int( M['m01']/M['m00'] )


			point = (cx,cy)
			print("POINT", point)
			break

	raw = cv2.resize(frame, (0,0), fx=1,fy=1)
	cv2.circle(raw, point, radius, color, line_width)

	cv2.imshow("Original",raw)
	cv2.imshow("Processed",thresh)



	ch = cv2.waitKey(1)
	if ch & 0xFF == ord('q'):
		break

cap.release()
#cv2.waitKey(0)

cv2.destroyAllWindows() 

