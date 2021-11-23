import rospy
from std_msgs.msg import Int64
import serial
import gpiozero as gpio
from time import sleep
import yaml

def talker():
    with open('/home/ubuntu/chickies-robot/playground/comports.yaml', "r") as stream:
        try:
            params = yaml.safe_load(stream)
            serialPort=serial.Serial(params['TEENSY_PORT'],params['TEENSY_BAUDRATE'])
        except yaml.YAMLError as exc:
            print(exc)
    leftEncoder = rospy.Publisher('Odometry/leftTicks', Int64, queue_size=10)
    rightEncoder = rospy.Publisher('Odometry/rightTicks', Int64, queue_size=10)
    rospy.init_node('odometry', anonymous=True)
    rate = rospy.Rate(params['TEENSY_UPDATE_RATE'])
    while not rospy.is_shutdown():
        read = serialPort.readline().decode('utf-8')
        data = read.split(',')
        leftTicks = int(data[1])
        rightTicks = int(data[0])
        #print(leftTicks)
        rospy.loginfo(leftTicks)
        rospy.loginfo(rightTicks)
        leftEncoder.publish(leftTicks)
        rightEncoder.publish(rightTicks)
        rate.sleep()
    serialPort.close()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass