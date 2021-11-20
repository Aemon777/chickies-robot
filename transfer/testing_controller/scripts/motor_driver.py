#!/usr/bin/env/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import gpiozero as gpio

class motor_driver():
	def __init__(self):
		rospy.init_node('motor_driver', anonymous=True)
		rospy.Subscriber("motor_vel", Twist, self.twistCallback)
		rospy.Subscriber("motor_control", String, self.controlCallback)

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
		self.for_rev = True
		self.maxPwm = 100

	def twistCallback(self,msg):
		self.xVal = msg.linear.x
		self.thetaVal = msg.angular.z
	
	def controlCallback(self,msg):
		info = msg.data.split(',')
		self.reverse = info[0] == "FOR"
		self.maxPwm = int(info[1])

	def spin(self):
		while not rospy.is_shutdown():
			self.spinOnce()
			self.rate.sleep()

	def spinOnce(self):
		throttle = self.xVal * 100 / 1.35
		if throttle > 100:
			throttle = 100
			rospy.loginfo(String("throttle max: 1.35 m/s"))
		if throttle < -100:
			throttle = -100
			rospy.loginfo(String("throttle max: 1.35 m/s"))
		steerLeft = self.thetaVal * 25
		if steerLeft < -75:
			steerLeft = -75
			rospy.loginfo(String("angular max: 3.0 rad/s"))
		if steerLeft > 75:
			steerLeft = 75
			rospy.loginfo(String("angular max: 3.0 rad/s"))
		if not self.for_rev:
			throttle = -throttle

		left = throttle + steerLeft
		right = throttle - steerLeft

		if left > 100:
			left = 100
		if left < -100:
			left = -100
		if right > 100:
			right = 100
		if left < -100:
			left = -100
		
		
		left = left*self.maxPwm/100
		right = right*self.maxPwm/100

		self.leftVal = round(0.75*self.leftVal + 0.25*left)
		self.rightVal = round(0.75*self.rightVal + 0.25*right)

		if self.leftVal <= 2 and self.leftVal >= -2:
			self.leftVal = 0
		if self.rightVal <= 2 and self.rightVal >= -2:
			self.rightVal = 0

		if self.leftVal < 0 and self.leftDir.is_active:
			self.leftDir.off()
		elif self.leftVal > 0 and not self.leftDir.is_active:
			self.leftDir.on()
		if self.rightVal < 0 and self.rightDir.is_active:
			self.self.rightDir.off()
		elif self.rightVal > 0 and not self.rightDir.is_active:
			self.rightDir.on()

		if self.leftPwm.value != abs(self.leftVal/100.00):
			self.leftPwm.value = abs(self.leftVal/100.00)
		if self.rightPwm.value != abs(self.rightVal/100.00):
			self.rightPwm.value = abs(self.rightVal/100.00)

		logString = String(str(self.xVal) + ", " + str(self.thetaVal) + ": " + str(self.leftVal) + ", " + str(self.rightVal))
		rospy.loginfo(logString)


if __name__ == '__main__':
	try:
		motor_driver = motor_driver()
		motor_driver.spin()
	except rospy.ROSInterruptException:
		pass
