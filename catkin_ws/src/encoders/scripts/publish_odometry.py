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
        try:
            read = serialPort.readline().decode('utf-8')
            data = read.split(',')
            leftTicks = int(data[1])
            rightTicks = int(data[0])
            print(leftTicks,rightTicks)
            #rospy.loginfo(leftTicks)
            #rospy.loginfo(rightTicks)
            leftEncoder.publish(leftTicks)
            rightEncoder.publish(rightTicks)
            rate.sleep()
        except serial.serialutil.SerialException:
            print('why is this happening to me????')
            sleep(0.5)
        except Exception as ex:
            template = "An exception of type {0} occurred. Arguments:\n{1!r}"
            message = template.format(type(ex).__name__, ex.args)
            print(message)
            sleep(0.5)
    serialPort.close()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
