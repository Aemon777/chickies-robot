#!/usr/bin/env/python

from difflib import context_diff
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import yaml
import serial

class motor_driver():
	def __init__(self):
		rospy.init_node('motor_driver', anonymous=True)
		rospy.Subscriber("motor_vel", Twist, self.twistCallback)
		rospy.Subscriber("motor_control", String, self.controlCallback)

		self.serialPort = ""
		with open("/home/ubuntu/chickies-robot/resources/comports.yaml", "r") as stream:
			try:
				data = yaml.safe_load(stream)
				self.serialPort=serial.Serial(data['TEENSY_PORT'],data['TEENSY_BAUDRATE'])
			except yaml.YAMLError as exc:
				print(exc)

		self.leftVel = 0
		self.rightVel = 0
		self.xVal = 0
		self.thetaVal = 0
		self.rate = rospy.Rate(10)
		self.maxVel = 1250 #mm/sec
		self.maxRads = 3

	def twistCallback(self,msg):
		self.xVal = round(msg.linear.x * 1000)
		self.thetaVal = msg.angular.z
	
	def controlCallback(self,msg):
		self.maxVel = int(msg.data)

	def spin(self):
		while not rospy.is_shutdown():
			self.spinOnce()
			self.rate.sleep()
			
	def limitThrottle(self, throttle, max):
		if throttle > max:
			throttle = max
			rospy.loginfo(String("throttle max: " + max + " m/s"))
		elif throttle < -max:
			throttle = -max
			rospy.loginfo(String("throttle min: -" + max + " m/s"))
		return throttle
	
	def limitTurn(self, turn, max):
		if turn > max:
			turn = max
			rospy.loginfo(String("angular max: " + max + " rad/s"))
		elif turn < -max:
			turn = -max
			rospy.loginfo(String("angular min: -" + max + " rad/s"))
		return turn
	
	def limitValsAfterTurn(self, val, opVal, max):
		shift = 0
		if val > max:
			shift = val - max
			opVal = opVal - shift
			val = max
		elif val < -max:
			shift = abs(val) - max
			opVal = opVal + shift
			val = -max
		return val, opVal
	
	def limitRoundedVal(self, val, max):
		if val <= 30 and val >= -30:
			val = 0
		elif val > 30 and val < 50:
			val = 50
		elif val < -30 and val > -50:
			val = -50
		elif val > max:
			val = max
		elif val < -max:
			val = -max
		return val
	
# Motor movement goals:
#	Wheels should always be abs(steerLeft)*2 apart from each other
#	Other than that, they should be moving at close to 
	def spinOnce(self):
		
		throttle = self.limitThrottle(self.xVal, self.maxVel)
		
		turn = self.limitTurn(self.thetaVal, self.maxRads)
		# recall that turn, here, is directly proportional to right velocity and inversely so to left velocity
		# Ccw and all that
		left = throttle - turn/3.6*self.maxVel
		right = throttle + turn/3.6*self.maxVel

		left, right = self.limitValsAfterTurn(left, right, self.maxVel)
		right, left = self.limitValsAfterTurn(right, left, self.maxVel)

		self.leftVel = round(0.75*self.leftVel + 0.25*left)
		self.rightVel = round(0.75*self.rightVel + 0.25*right)
		
		self.leftVel = self.limitRoundedVal(self.leftVel, self.maxVel)
		self.rightVel = self.limitRoundedVal(self.rightVel, self.maxVel)

		serialString = "V," + str(self.leftVel) + "," + str(self.rightVel) + ",**"
		self.serialPort.write(bytes(serialString, "utf-8"))
		
		logString = String(str(self.xVal) + ", " + str(self.thetaVal) + ": " + str(self.leftVel) + ", " + str(self.rightVel))
		rospy.loginfo(logString)


if __name__ == '__main__':
	try:
		motor_driver = motor_driver()
		motor_driver.spin()
	except rospy.ROSInterruptException:
		pass
