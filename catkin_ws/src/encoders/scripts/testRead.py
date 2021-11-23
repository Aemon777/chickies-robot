import rospy
from std_msgs.msg import Int64

def callbackLeft(data):
    rospy.loginfo('Left Ticks: ' + str(data.data))
def callbackRight(data):
    rospy.loginfo('Right Ticks: ' + str(data.data))
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('encoder_reader', anonymous=True)

    rospy.Subscriber('Odometry/leftTicks', Int64, callbackLeft)
    rospy.Subscriber('Odometry/rightTicks', Int64, callbackRight)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()