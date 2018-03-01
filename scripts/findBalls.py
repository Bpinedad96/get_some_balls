# import the necessary packages
from collections import deque
import numpy as np
import argparse
import imutils
import math
import cv2
 

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
 
# initialize zed
camera = cv2.VideoCapture(0)

# keep looping
while True:
	# grab the current frame
	(grabbed, frame) = camera.read()
 
	# resize the frame, blur it, and convert it to the HSV
	# color space
	frame = imutils.resize(frame, width=1000)
	#blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	cv2.imshow("Frame2", mask)

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE) [1]
	center = None

	contours_area = []
	# calculate area and filter into new array
	for con in cnts:
	    area = cv2.contourArea(con)
	    contours_area.append(con)

	contours_circles = []

	# check if contour is of circular shape
	for con in contours_area:
	    perimeter = cv2.arcLength(con, True)
	    area = cv2.contourArea(con)
	    if perimeter == 0:
		break
	    circularity = 4*math.pi*(area/(perimeter*perimeter))
	    if 0.7 < circularity < 1.2:
		contours_circles.append(con)
	 
		# only proceed if at least one contour was found
		if len(cnts) > 0:
			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid
			((x, y), radius) = cv2.minEnclosingCircle(con)
			M = cv2.moments(con)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	 
			# only proceed if the radius meets a minimum size
				# draw the circle and centroid on the frame,
				# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)
 
	# show the frame to our screen
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
 
	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break
 
	print ("Area "+str(contours_area))
	print ("Circles "+str(contours_circles))
# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()


#16 a 125   7.81
#24 a92      3.8





