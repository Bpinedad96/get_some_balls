#!/usr/bin/env python2
# import the necessary packages
from collections import deque
import numpy as np
import argparse
import imutils
import math
import cv2
import rospy
import serial, time

from visualization_msgs.msg import Marker
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from collections import deque

#Opens port
#ser = serial.Serial('/dev/ttyACM0', 115200)

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (26, 43, 93) #26 51 85 
greenUpper = (42, 131, 255) #39 125 255

#ROS information
publisher  = rospy.Publisher('visualization_marker', Marker)
publisher2  = rospy.Publisher('servo_move', Int8MultiArray)
publisher3  = rospy.Publisher('ball_pos', Float32MultiArray)

rospy.init_node('balls')

profundity=0

#200 px a 30cm
#160 px a 40cm
#F=PD/W

camera=cv2.VideoCapture(0)

F=925
W=6.45
x=0
y=0

marker = Marker()
#servo= ""
#servoS= ""
#servoU= ""
#servoD= ""
servo=Int8MultiArray()
servo.data.append(0)
servo.data.append(0)

ball=Float32MultiArray()
ball.data.append(0)


area=0
area_before=0
#x_before=0
#y_before=0
#start=0
result=0
results = deque([0])
samples=24
thresh=15
error=100
#request=0

# keep looping
while True:
	# grab the current frame
	try:
		#rospy.Subscriber("webcam/image_raw",Image,callback)
		#frame = cv_image

		passed, frame=camera.read()
		h,  w = frame.shape[:2]
		frame = imutils.resize(frame, width=1000)
		

		blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		blurred = cv2.medianBlur(blurred,5)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	 
		# construct a mask for the color "green", then perform
		# a series of dilations and erosions to remove any small
		# blobs left in the mask
		mask = cv2.inRange(hsv, greenLower, greenUpper)#6 to 4
		cv2.imshow("Frame2", mask)

		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=6)
		mask = cv2.erode(mask, None, iterations=6)

		# find contours in the mask and initialize the current
		# (x, y) center of the ball
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE) [1]
		center = None

		contours_area = []
		contours_circles = []
		i=0
		i_max=0
		circularity_max=0
		area_max=0
					

		# check if contour is of circular shape	
		for con in cnts:
			perimeter = cv2.arcLength(con, True)
			area = cv2.contourArea(con)
			contours_area.append(con)
			if perimeter == 0:
				break
			circularity = 4*math.pi*(area/(perimeter*perimeter))
			if area>area_max:
				area_max=area
				circularity_max=circularity
				i_max=i
			i=i+1
		result=0

		if 0.6 < circularity_max:
			result=1
			
		if 0.6 < circularity_max and results.count(1)>thresh:
			contours_circles.append(cnts[i_max])
			contours_area.append(cnts[i_max])

			((x, y), radius) = cv2.minEnclosingCircle(cnts[i_max])
		
			M = cv2.moments(cnts[i_max])
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
			
			# only proceed if the radius meets a minimum size
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)

			#profundity=1103.9*(radius)**(-1.131) #eq obtaine by excel
			profundity=(W*F)/(radius*2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)
			cv2.putText(frame, "%.1f cm" % profundity, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

			'''if x<(w/2)-error:
				servoS="l"
			elif x>(w/2)+error:
				servoS="r"
			else:
				servoS="n"
			
			if y<(h/2)-error:
				servoU="u"
			elif y>(h/2)+error:
				servoU="d"
			else:
				servoU="n"'''
			
			if x<(w/2)-error:
				servo.data[0]=1
			elif x>(w/2)+error:
				servo.data[0]=-1
			else:
				servo.data[0]=0
			
			if y<(h/2)-error:
				servo.data[1]=-1
			elif y>(h/2)+error:
				servo.data[1]=1
			else:
				servo.data[1]=0
			
			#servoD=str(profundity)
			ball.data[0]=profundity

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

			area_before=area

		elif (results.count(1)<thresh):
			marker.action=marker.DELETE
			servo.data[0]=2
			servo.data[1]=0
			#servoS="s"
			#servoU="n"		
			#servoD="n"
		
		else:
			servo.data[0]=2
			#servoS="s"	
				
				
		#Since it seems to be the ball a comparison against 100 frames is done
		if len(results)<samples:
			results.append(result)
		else:
			results.rotate(1)
			results[0]=result
				

		publisher.publish(marker)
		publisher2.publish(servo)
		publisher3.publish(ball)
		#servo=servoS+servoU+servoD
		#request=ser.readline()
		'''if (request=='1'):
			request=0
			print(servo)
			ser.write(servo)'''

		# show the frame to our screen
		cv2.imshow("Frame", frame)
		key = cv2.waitKey(1) & 0xFF
		# if the 'q' key is pressed, stop the loop
		if key == ord("q"):
			break

	except Exception as e:
		print(e) 
		pass
# cleanup the camera and close any open windows
camera.release()
#ser.close()
cv2.destroyAllWindows()

