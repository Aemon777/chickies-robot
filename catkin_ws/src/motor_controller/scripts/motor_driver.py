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
		self.rate = rospy.Rate(40)
		self.maxPwm = 100

	def twistCallback(self,msg):
		self.xVal = msg.linear.x
		self.thetaVal = msg.angular.z
	
	def controlCallback(self,msg):
		self.maxPwm = int(msg.data)

	def spin(self):
		while not rospy.is_shutdown():
			self.spinOnce()
			self.rate.sleep()
			
	def limitThrottle(throttle):
		if throttle > 100:
			throttle = 100
			rospy.loginfo(String("throttle max: 1.35 m/s"))
		elif throttle < -100:
			throttle = -100
			rospy.loginfo(String("throttle max: 1.35 m/s"))
		return throttle
	
	def limitTurn(turn):
		if turn < -75:
			turn = -75
			rospy.loginfo(String("angular max: 3.0 rad/s"))
		elif turn > 75:
			turn = 75
			rospy.loginfo(String("angular max: 3.0 rad/s"))
		return turn
	
	def limitValsAfterTurn(val, opVal):
		shift = 0
		if val > 100:
			shift = val - 100
			opVal = opVal - shift
			val = 100
		elif val < -100:
			shift = abs(val) - 100
			opVal = opVal + shift
			val = -100
		return val, opVal, shift
	
	def limitRoundedVal(val):
		if val <= 2 and val >= -2:
			val = 0
		elif val > 100:
			val = 100
		elif self.leftVal < -100:
			val = -100
		return val
	
# Motor movement goals:
#	Wheels should always be abs(steerLeft)*2 apart from each other
#	Other than that, they should be moving at close to 
	def spinOnce(self):
		
		throttle = limitThrottle(self.xVal * 100 / 1.35)
		
		steerLeft = limitTurn(self.thetaVal * 25)

		left = throttle + steerLeft
		right = throttle - steerLeft

		left, right, rightShift = limitValsAfterTurn(left, right)
		right, left, leftShift = limitValsAfterTurn(right, left)
		
		left = left*self.maxPwm/100
		right = right*self.maxPwm/100

		self.leftVal = round(0.75*self.leftVal + 0.25*left)
		self.rightVal = round(0.75*self.rightVal + 0.25*right)
		
		self.leftVal = limitRoundedVal(self.leftVal)
		self.rightVal = limitRoundedVal(self.rightVal)

		if self.leftVal < 0 and self.leftDir.is_active:
			self.leftDir.off()
			print("left wheel back")
		elif self.leftVal > 0 and not self.leftDir.is_active:
			self.leftDir.on()
			print("left wheel forward")
		if self.rightVal < 0 and self.rightDir.is_active:
			self.rightDir.off()
			print("right wheel backward")
		elif self.rightVal > 0 and not self.rightDir.is_active:
			self.rightDir.on()
			print("right wheel forward")

		if self.leftPwm.value != abs(self.leftVal/100.00):
			self.leftPwm.value = abs(self.leftVal/100.00)
			print("left pwm updated to " + str(self.leftPwm.value))
		if self.rightPwm.value != abs(self.rightVal/100.00):
			self.rightPwm.value = abs(self.rightVal/100.00)
			print("right pwm updated to " + str(self.rightPwm.value))

		logString = String(str(self.xVal) + ", " + str(self.thetaVal) + ": " + str(self.leftVal) + ", " + str(self.rightVal))
		rospy.loginfo(logString)


if __name__ == '__main__':
	try:
		motor_driver = motor_driver()
		motor_driver.spin()
	except rospy.ROSInterruptException:
		pass
