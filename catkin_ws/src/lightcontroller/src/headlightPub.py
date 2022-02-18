import rospy
from std_msgs.msg import Bool
from time import sleep
import sys

headlights = rospy.Publisher('headlights', Bool, queue_size=10)
sleep(0.5)
rospy.init_node('headlights', anonymous=True)
sleep(0.5)
lightState = (sys.argv[1] == "True")
rospy.loginfo(lightState)
headlights.publish(lightState)
