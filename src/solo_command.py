#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist # msg type for cmd_vel

def takeoff():
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10 )
        rospy.init_node('takeoff', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        vel_msg = Twist()
        
        vel_msg.angular.z =0.5
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        while not rospy.is_shutdown():
          pub.publish(vel_msg)
          rate.sleep()

if __name__ == '__main__':
        try:
          takeoff()
        except rospy.ROSInterruptException:
          pass