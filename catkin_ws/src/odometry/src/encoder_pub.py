import rospy
from std_msgs.msg import Int64
import serial
from time import sleep
import yaml
from std_msgs.msg import String

class encoder_pub():
    def __init__(self):
        self.serialPort = ""
        with open('/home/ubuntu/chickies-robot/resources/comports.yaml', "r") as stream:
            try:
                params = yaml.safe_load(stream)
                self.serialPort=serial.Serial(params['TEENSY_PORT'],params['TEENSY_BAUDRATE'])
            except yaml.YAMLError as exc:
                print(exc)
        rospy.init_node('odometry', anonymous=True)
        rospy.Subscriber('motor_vel', String, self.velCallback)
        self.leftEncoder = rospy.Publisher('left_ticks', Int64, queue_size=10)
        self.rightEncoder = rospy.Publisher('right_ticks', Int64, queue_size=10)
        self.rate = rospy.Rate(params['TEENSY_UPDATE_RATE'])

    def velCallback(self, msg):
        try:
            self.serialPort.write(bytes(msg.data, 'utf-8'))
        except:
            try:
                print()
                print()
                self.serialPort.close()
                with open('/home/ubuntu/chickies-robot/resources/comports.yaml', "r") as stream:
                    try:
                        print("Restarting port")
                        params = yaml.safe_load(stream)
                        self.serialPort=serial.Serial(params['TEENSY_PORT'],params['TEENSY_BAUDRATE'])
                    except yaml.YAMLError as exc:
                        print(exc)
            except:
                print("Failed to open port...")
                sleep(0.5)

    def spin(self):
        while not rospy.is_shutdown():
            try:
                read = self.serialPort.readline().decode('utf-8')
                data = read.split(',')
                if(data[0] == 'E'):
                    leftTicks = int(data[1])
                    rightTicks = int(data[2])
                    #print(leftTicks,rightTicks)
                    rospy.loginfo(leftTicks)
                    rospy.loginfo(rightTicks)
                    self.leftEncoder.publish(leftTicks)
                    self.rightEncoder.publish(rightTicks)
                self.rate.sleep()
            except serial.serialutil.SerialException:
                self.serialPort.close()
                with open('/home/ubuntu/chickies-robot/resources/comports.yaml', "r") as stream:
                    try:
                        params = yaml.safe_load(stream)
                        self.serialPort=serial.Serial(params['TEENSY_PORT'],params['TEENSY_BAUDRATE'])
                    except yaml.YAMLError as exc:
                        print(exc)
                sleep(0.5)
            except Exception as ex:
                template = "An exception of type {0} occurred. Arguments:\n{1!r}"
                message = template.format(type(ex).__name__, ex.args)
                self.serialPort.close()
                with open('/home/ubuntu/chickies-robot/resources/comports.yaml', "r") as stream:
                    try:
                        print("Restarting port")
                        params = yaml.safe_load(stream)
                        self.serialPort=serial.Serial(params['TEENSY_PORT'],params['TEENSY_BAUDRATE'])
                    except yaml.YAMLError as exc:
                        print(exc)
                print(message)
                sleep(0.5) 
        self.serialPort.close()

if __name__ == '__main__':
    try:
        encoder_pub = encoder_pub()
        encoder_pub.spin()
    except rospy.ROSInterruptException:
        pass
