#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def test_publisher():
	pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
	rospy.init_node('test_publisher', anonymous=True)
	rate = rospy.Rate(50)
	wait = rospy.Rate(0.5)
	while not rospy.is_shutdown():
		for duty in range(0, 91, 1):
			if not rospy.is_shutdown():
				cmd = Twist()
				cmd.linear.x = duty/100
				rospy.loginfo(cmd)
				pub.publish(cmd)
				rate.sleep()
		wait.sleep()
		for duty in range(0, 31, 1):
			if not rospy.is_shutdown():
				cmd = Twist()
				cmd.angular.z = duty/10.0
				cmd.linear.x = 1
				rospy.loginfo(cmd)
				pub.publish(cmd)
				rate.sleep()
		wait.sleep()
		for duty in range(31,-31,-1):
			if not rospy.is_shutdown():
				cmd = Twist()
				cmd.angular.z = duty/10.0
				cmd.linear.x = 1
				rospy.loginfo(cmd)
				pub.publish(cmd)
				rate.sleep()
		wait.sleep()
		for duty in range(-31, 0, 1):
			if not rospy.is_shutdown():
				cmd = Twist()
				cmd.angular.z = duty/10.0
				cmd.linear.x = 1
				rospy.loginfo(cmd)
				pub.publish(cmd)
				rate.sleep()
		wait.sleep()
		for duty in range(91, -1, -1):
			if not rospy.is_shutdown():
				cmd = Twist()
				cmd.linear.x = duty/100
				rospy.loginfo(cmd)
				pub.publish(cmd)
				rate.sleep()
		wait.sleep()

if __name__  == '__main__':
	try:
		test_publisher()
	except rospy.ROSInterruptException:
		pass
