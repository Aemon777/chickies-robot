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
		rospy.Subscriber("cmd_vel", Twist, self.twistCallback)

		self.serialPort = ""
		with open("/home/ubuntu/chickies-robot/resources/comports.yaml", "r") as stream:
			try:
				data = yaml.safe_load(stream)
				self.serialPort=serial.Serial(data['TEENSY_MOTORS_PORT'],data['TEENSY_MOTORS_BAUDRATE'])
			except yaml.YAMLError as exc:
				print(exc)

		self.leftVel = 0
		self.rightVel = 0
		self.xVal = 0
		self.thetaVal = 0
		self.rate = rospy.Rate(10)
		self.maxVel = 1.0#1.0 #m/sec
		self.velUnits = 1.0 #m/sec
		self.maxRads = 2.4#2.4

	def twistCallback(self,msg):
		self.xVal = msg.linear.x * self.velUnits
		self.thetaVal = msg.angular.z - 0.006

	def spin(self):
		while not rospy.is_shutdown():
			self.spinOnce()
			self.rate.sleep()
			
	def limitThrottle(self, throttle, max):
		if throttle > max:
			throttle = max
			rospy.loginfo(String("throttle max: " + str(max) + " m/s"))
		elif throttle < -max:
			throttle = -max
			rospy.loginfo(String("throttle min: -" + str(max) + " m/s"))
		return throttle
	
	def limitTurn(self, turn, max):
		if turn > max:
			turn = max
			rospy.loginfo(String("angular max: " + str(max) + " rad/s"))
		elif turn < -max:
			turn = -max
			rospy.loginfo(String("angular min: -" + str(max) + " rad/s"))
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
		if val <= 0.03*self.velUnits and val >= -0.03*self.velUnits:
			val = 0
		elif val > 0.03*self.velUnits and val < 0.05*self.velUnits:
			val = 0.05*self.velUnits
		elif val < -0.03*self.velUnits and val > -0.05*self.velUnits:
			val = -0.05*self.velUnits
		elif val > max:
			val = max
		elif val < -max:
			val = -max
		return val
	
	def limitChange(self, target, current):
		tar = target + 2.0
		cur = current + 2.0
		maxChange = self.calculateMaxChange(current)
		if (tar - cur) > maxChange:
			tar = cur + maxChange
		elif (cur - tar) > maxChange:
			tar = cur - maxChange
		return tar - 2.0
	
	def calculateMaxChange(self, current):
		return 0.1 + abs(current)*0.1
	
# Motor movement goals:
#	Wheels should always be abs(steerLeft)*2 apart from each other
#	Other than that, they should be moving at close to 
	def spinOnce(self):
		
		throttle = self.limitThrottle(self.xVal, self.maxVel)
		
		turn = self.limitTurn(self.thetaVal, self.maxRads)
		# recall that turn, here, is directly proportional to right velocity and inversely so to left velocity
		# Ccw and all that
		left = throttle - turn/self.maxRads*self.maxVel
		right = throttle + turn/self.maxRads*self.maxVel

		left, right = self.limitValsAfterTurn(left, right, self.maxVel)
		right, left = self.limitValsAfterTurn(right, left, self.maxVel)
		
		#left = self.limitChange(left, self.leftVel)
		#right = self.limitChange(right, self.rightVel)
		
		left = round(left, 3)
		right = round(right, 3)

		self.leftVel = self.limitRoundedVal(left, self.maxVel)
		self.rightVel = self.limitRoundedVal(right, self.maxVel)

		serialString = "V," + str(self.leftVel) + "," + str(self.rightVel) + ",**"
		
		logString = String(str(self.xVal) + ", " + str(self.thetaVal) + ": " + str(self.leftVel) + ", " + str(self.rightVel))
		rospy.loginfo(logString)
		self.serialPort.write(bytes(serialString, "utf-8"))

if __name__ == '__main__':
	try:
		motor_driver = motor_driver()
		motor_driver.spin()
	except rospy.ROSInterruptException:
		pass
