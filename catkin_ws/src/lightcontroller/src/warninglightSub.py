import gpiozero as gpio
from time import sleep
import rospy
from std_msgs.msg import Bool

led = gpio.LED(5)

def cbWarninglights(data):
    rospy.loginfo('Warning light state: ' + str(data.data))
    if(data.data):
        led.off()
    else:
        led.on()
    
def listener():
    rospy.init_node('warning_state_reader', anonymous=True)
    rospy.Subscriber('Lights/warninglight', Bool, cbWarninglights)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    led.on()
    listener()
