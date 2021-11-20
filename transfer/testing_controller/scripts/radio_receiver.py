#!/usr/bin/env/python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial
import yaml

class radio_receiver():
    def __init__(self):
        with open("comports.yaml", "r") as stream:
            try:
                data = yaml.safe_load(stream)
                self.serialPort = serial.Serial(data['RADIO_PORT'],data['RADIO_BAUDRATE'])
            except yaml.YAMLError as exc:
                print(exc)
        self.rate = rospy.Rate(20)
        self.pub_vel = rospy.Publisher('motor_vel', Twist, queue_size = 10)
        self.pub_control = rospy.Publisher('motor_control', String, queue_size = 10)
        rospy.init_node('radio_receiver', anonymous=True)

    def spin(self):
        while not rospy.is_shutdown():
            self.spinOnce()
            self.rate.sleep()
    
    def spinOnce(self):
        #read from port
        read = self.serialPort.readline().decode('utf-8')
        data = read.split(',')

        #twist/velocity message
        throttle = int(data[2])
        steer = int(data[0])
        steerLeft = steer*1.5 - 75
        vel = Twist()
        vel.linear.x = throttle*1.35/100
        vel.angular.z = steerLeft/3
        rospy.loginfo(vel)
        self.pub_vel.publish(vel)

        #string/control message
        for_rev = (int(data[4]) < 50)
        maxPwm = 100 - int(data(5))
        control = String()
        if for_rev:
            control.data += "FOR, "
        else:
            control.data += "REV, "
        control.data += str(maxPwm)
        rospy.loginfo(control)
        self.pub_control.publish(control)

if __name__ == '__main__':
    try:
        radio_receiver = radio_receiver()
        radio_receiver.spin()
    except rospy.ROSInterruptException:
        pass