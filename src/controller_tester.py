#!/usr/bin/env python

# your normalization --> modifies PID lol
# Add a limit to yaw  ( Stop rotating and hover if yaw exceed a limit)
# Rewrite code with a better structure --> Separate e and u calculation 

import rospy
from geometry_msgs.msg import Twist # msg type for cmd_vel
from ardrone_project.msg import Coords
from math import pow, atan2, sqrt
import numpy as np

kp_angular_z  = 0.2

class ardrone_pid():


	def __init__(self):
		rospy.init_node('ardrone_controller',anonymous = True)
		self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
		self.coords_subscriber = rospy.Subscriber('/ardrone/all_coordinates', Coords, self.callback)
		self.coords = Coords()
		self.rate = rospy.Rate(10)

	def callback(self,data):
		self.coords = data

	def pid_controller(self):	
		
		vel_msg = Twist()

		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.linear.x = 0
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0

		while True:

			# Turn right
			if self.coords.x_b - self.coords.x_c > 0:
				variable = -kp_angular_z*(self.coords.x_b - self.coords.x_c)
				vel_msg.angular.z = 1/(1+np.exp(-variable/10))*2 - 1
				print(variable)

			# Turn left
			elif self.coords.x_b - self.coords.x_c < 0:
				temp_var = kp_angular_z*(self.coords.x_b - self.coords.x_c)
				vel_msg.angular.z = 1/(1+np.exp(-temp_var/10))*2 - 1
				print(temp_var)

			# If tracking is lost, enter auto-hover mode ( Test this out separately)
			if self.coords.track == False:
				vel_msg.angular.z = 0

			self.velocity_publisher.publish(vel_msg)
			self.rate.sleep()

		rospy.spin()


if __name__ == '__main__':
	try:
		x = ardrone_pid()
		x.pid_controller()

	except rospy.ROSInterruptException: pass