#!/usr/bin/env python

# your normalization --> modifies PID lol
# Add a limit to yaw  ( Stop rotating and hover if yaw exceed a limit)
# Rewrite code with a better structure --> Separate e and u calculation 

import rospy
from geometry_msgs.msg import Twist # msg type for cmd_vel
from ardrone_project.msg import Coords
from math import pow, atan2, sqrt



class ardrone_pid():


	def __init__(self):
		rospy.init_node('ardrone_controller',anonymous = True)
		self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
		self.coords_subscriber = rospy.Subscriber('/ardrone/all_coordinates', Coords, self.callback)
		self.coords = Coords()
		self.rate = rospy.Rate(10)
		print('Hi_2')

	def callback(self,data):
		self.coords = data
		# print(self.coords.x_b)


	def pid_controller(self):
		
		# # 100 is more lineant
		# tolerance = 75

		# vel_msg = Twist()

		# vel_msg.angular.x = 0
		# vel_msg.angular.y = 0
		# vel_msg.linear.x = 0
		# vel_msg.linear.y = 0
		# vel_msg.linear.z = 0
		print('Hi pid')

		integral = 0
		time_0 = 0

		
		while sqrt( pow((self.coords.x_b - self.coords.x_c),2) + pow((self.coords.y_b - self.coords.y_c),2)) > tolerance:
			
			time_1 = time.gettime()
			dt = time_1 - time_0
			integral = integral + error*dt
			# Turn right
			if self.coords.x_b - self.coords.x_c > 0:
				variable = -kp_angular_z*sqrt(pow((self.coords.x_b - self.coords.x_c),2) + pow((self.coords.y_b - self.coords.y_c),2))
				vel_msg.angular.z = 1/(1+np.exp(-variable/10))*2 - 1

			# Turn left
			elif self.coords.x_b - self.coords.x_c < 0:
				temp_var = kp_angular_z*sqrt(pow((self.coords.x_b - self.coords.x_c),2) + pow((self.coords.y_b - self.coords.y_c),2))	
				vel_msg.angular.z = 1/(1+np.exp(-temp_var/10))*2 - 1
		
			# If tracking is lost, enter auto-hover mode ( Test this out separately)
			if coords.track == False:
				vel_msg.angular.z = 0

			print(vel_msg.angular.z)
			self.velocity_publisher.publish(vel_msg)
			self.rate.sleep()

		vel_msg.angular.z = 0
		self.velocity_publisher.publish(vel_msg)

		rospy.spin()


if __name__ == '__main__':
	try:
		x = ardrone_pid()
		x.pid_controller()

	except rospy.ROSInterruptException: pass