import gpiozero as gpio
from time import sleep
import rospy
from std_msgs.msg import Bool

led = gpio.LED(27)

def cbHeadlights(data):
    rospy.loginfo('Headlight state: ' + str(data.data))
    if(data.data):
        led.off()
    else:
        led.on()
    
def listener():
    rospy.init_node('light_state_reader', anonymous=True)
    rospy.Subscriber('Lights/headlights', Bool, cbHeadlights)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
