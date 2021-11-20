#!/usr/bin/env/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import gpiozero as gpio

class test_controller():
	def __init__(self):
		rospy.init_node('test_controller', anonymous=True)
		rospy.Subscriber("cmd_vel", Twist, self.twistCallback)

		self.leftVal = 0
		self.rightVal = 0
		self.leftPwm = gpio.PWMLED(13)
		self.rightPwm = gpio.PWMLED(12)
		self.leftDir = gpio.LED(6)
		self.rightDir = gpio.LED(5)
		self.n = True
		self.xVal = 0
		self.thetaVal = 0
		self.rate = rospy.Rate(50)

	def twistCallback(self,msg):
		self.xVal = msg.linear.x;
		self.thetaVal = msg.angular.z;

	def spin(self):
		while not rospy.is_shutdown():
			self.spinOnce()
			self.rate.sleep()

	def spinOnce(self):
		throttle = self.xVal * 100
		if throttle > 100:
			throttle = 100
			rospy.loginfo("throttle max: 1.0 m/s")
		steerRight = self.thetaVal * 25
		if steerRight < -75:
			steerRight = -75
			rospy.loginfo("angular max: 3.0 rad/s")
		if steerRight > 75:
			steerRight = 75
			rospy.loginfo("angular max: 3.0 rad/s")
		left = throttle + steerRight
		right = throttle - steerRight
		if left > 100:
			left = 100
		if right > 100:
			right = 100
		
		if left < 0:
			self.leftDir.off()
			left = -left
		else:
			self.leftDir.on()
		if right < 0:
			self.rightDir.off()
			right = -right
		else:
			self.rightDir.on()

		self.leftVal = round(0.7*self.leftVal + 0.3*left)
		self.rightVal = round(0.7*self.rightVal + 0.3*right)
		if self.leftVal <= 2:
			self.leftVal = 0
		if self.rightVal <= 2:
			self.rightVal = 0

		self.leftPwm.value = self.leftVal/100.00
		self.rightPwm.value = self.rightVal/100.00
		logString = String(str(self.xVal) + ", " + str(self.thetaVal) + ": " + str(self.leftVal) + ", " + str(self.rightVal))
		rospy.loginfo(logString)


if __name__ == '__main__':
	try:
		test_controller = test_controller()
		test_controller.spin()
	except rospy.ROSInterruptException:
		pass
