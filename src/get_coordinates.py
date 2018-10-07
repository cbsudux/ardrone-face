#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ardrone_face.msg import Coords
from cv_bridge import CvBridge, CvBridgeError
import dlib
import numpy as np
from math import pow, atan2, sqrt

face_cascade = cv2.CascadeClassifier('/home/cbsudux/catkin_ws/src/ardrone_face/src/haarcascade_frontalface_default.xml')
tracker = dlib.correlation_tracker()
rectangleColor = (0,165,255)

# Class that represents the node
class image_converter:

	def __init__(self):
		self.coordinates_publisher = rospy.Publisher('/ardrone/all_coordinates', Coords, queue_size = 10)
		self.tracking_face = 0
		self.bridge = CvBridge()
		# Subscriber that subscribes from drone's camera feed
		self.image_sub = rospy.Subscriber("/ardrone/image_raw",Image,self.callback)


	# Bulk of logic executed in callback. Called by rospin again and again
	def callback(self,data):

		# ros img -> cv_img conversion
		coordinates_msg = Coords()

		try:
			img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		height = np.size(img, 0)
		width = np.size(img, 1)

		# Detetction and tracking starts here	
		if not self.tracking_face:
			print("Trying to detect face!")
			faces = face_cascade.detectMultiScale(img,1.3,5)

			max_area,x,y,w,h = 0,0,0,0,0

			for (_x,_y,_w,_h) in faces:
				if _w*_h > max_area:
					x = int(_x)
					y = int(_y)
					w = int(_w)
					h = int(_h)
					max_area = w*h

			if max_area:
				# Track a rectangle of (x,y,x+w,y+h) given by the haar detector ( Rectangle will be drawn in the nxt if statement)
				#Note you may have to change x to x-20 etc
				tracker.start_track(img, dlib.rectangle(x,y,x+w,y+h))
				self.tracking_face = 1

		if self.tracking_face:
			tracking_quality = tracker.update(img)

			if tracking_quality > 8.75:
				tracked_position = 	tracker.get_position()
				t_x = int(tracked_position.left())
				t_y = int(tracked_position.top())
				t_w = int(tracked_position.width())
				t_h = int(tracked_position.height())
				cv2.rectangle(img,(t_x,t_y),(t_x + t_w, t_y+t_h), rectangleColor,2)
				
				coordinates_msg.x_b = t_x + t_w/2
				coordinates_msg.y_b = t_y + t_h/2
				coordinates_msg.x_c = width/2
				coordinates_msg.y_c = height/2

				cv2.line(img,(t_x + t_w/2,0),(t_x + t_w/2,height),(0,255,0))
				cv2.line(img,(0,t_y+t_h/2),(width,t_y +t_h/2),(0,255,0))

			else :
				self.tracking_face = 0

		coordinates_msg.track = self.tracking_face	
		self.coordinates_publisher.publish(coordinates_msg)

		kp_angular_z = 0.1
		error_max = 370
		# normalizer = 1/(kp_angular_z*error_max)

		
		if coordinates_msg.x_b - coordinates_msg.x_c > 0:
			z = -kp_angular_z*sqrt(pow((coordinates_msg.x_b - coordinates_msg.x_c),2) + pow((coordinates_msg.y_b - coordinates_msg.y_c),2))
			print(1/(1+np.exp(-z/10))*2 - 1)


			# Turn left
		elif coordinates_msg.x_b - coordinates_msg.x_c < 0:
			z = kp_angular_z*sqrt(pow((coordinates_msg.x_b - coordinates_msg.x_c),2) + pow((coordinates_msg.y_b - coordinates_msg.y_c),2))
			print(1/(1+np.exp(-z/10))*2 - 1)

	
		cv2.line(img,(width/2,0),(width/2,height),(255,0,0))
		cv2.line(img,(0,height/2),(width,height/2),(255,0,0))
		cv2.imshow("Image window", img)
		cv2.waitKey(3)




if __name__ == '__main__':
	ic = image_converter()
	rospy.init_node('Camera_init', anonymous=True)
	# Where else can spin be placed? ( Inside class? )
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()
