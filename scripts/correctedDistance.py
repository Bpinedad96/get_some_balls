#!/usr/bin/env python2
# import the necessary packages
from collections import deque
import numpy as np
import argparse
import imutils
import math
import cv2
import rospy

from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)

#ROS information
publisher = rospy.Publisher('visualization_marker', Marker)

rospy.init_node('balls')

profundity=0
camera=cv2.VideoCapture('rtsp://admin:123qweasd@192.168.1.88:554/11')
#75 px a 30cm
#150 a 30cm
#F=PD/W
F=697.67
W=6.45

mtx= np.array([[1307.664136, 0.000000, 953.434477], [0.000000, 1304.310096, 568.717493], [0.000000, 0.000000, 1.000000]])

dist=np.array([-0.387577, 0.121987, -0.001898, -0.002118, 0.000000])

# keep looping
while True:
	# grab the current frame
	try:
		#rospy.Subscriber("webcam/image_raw",Image,callback)
		#frame = cv_image
		(passed, fm)=camera.read()
		h,  w = fm.shape[:2]
		newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

		# undistort
		frame = cv2.undistort(fm, mtx, dist, None, mtx)
		#frame=fm
		# crop the image
		x,y,w,h = roi
		frame = frame[y:y+h, x:x+w]

		# resize the frame, blur it, and convert it to the HSV
		# color space
		frame = imutils.resize(frame, width=1000)
		#blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	 
		# construct a mask for the color "green", then perform
		# a series of dilations and erosions to remove any small
		# blobs left in the mask
		mask = cv2.inRange(hsv, greenLower, greenUpper)
		mask = cv2.erode(mask, None, iterations=6)
		mask = cv2.dilate(mask, None, iterations=6)
		mask = cv2.dilate(mask, None, iterations=4)
		mask = cv2.erode(mask, None, iterations=4)
		cv2.imshow("Frame2", mask)

		# find contours in the mask and initialize the current
		# (x, y) center of the ball
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE) [1]
		center = None

		contours_area = []
		contours_circles = []

		# check if contour is of circular shape	
		for con in cnts:
			perimeter = cv2.arcLength(con, True)
			area = cv2.contourArea(con)
			contours_area.append(con)
			if perimeter == 0:
				break
			circularity = 4*math.pi*(area/(perimeter*perimeter))
			if 0.65 < circularity < 1.2 and area>50:
				contours_circles.append(con)
				contours_area.append(con)

				((x, y), radius) = cv2.minEnclosingCircle(con)
			
				M = cv2.moments(con)
				center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

				# only proceed if the radius meets a minimum size
				# draw the circle and centroid on the frame,
				# then update the list of tracked points
				cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)

				#profundity=1103.9*(radius)**(-1.131) #eq obtaine by excel
				profundity=(W*F)/(radius*2)
				cv2.circle(frame, center, 5, (0, 0, 255), -1)
				cv2.putText(frame, "%.1f cm" % profundity, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

				marker = Marker()
				marker.header.frame_id = "/camera"
				marker.type = marker.SPHERE
				marker.action = marker.ADD
				marker.scale.x = 0.062
				marker.scale.y = 0.062
				marker.scale.z = 0.062
				marker.color.a = 1.0
				marker.color.g = 1.0
				marker.pose.orientation.w = 1.0
				marker.pose.position.x = profundity/100.00
				marker.pose.position.y = ((w/4)-x)*(6.2/(radius*2))/100.00
				marker.pose.position.z = ((h/2)-y)*(6.2/(radius*2))/100.00
				marker.lifetime.secs = 0.01
			else:
				marker.action=marker.DELETE

		publisher.publish(marker)


		# show the frame to our screen
		cv2.imshow("Frame", frame)
		key = cv2.waitKey(1) & 0xFF
		# if the 'q' key is pressed, stop the loop
		if key == ord("q"):
			break

	except Exception as e:
		print(e) 
# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()

