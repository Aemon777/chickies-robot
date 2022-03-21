#!/usr/bin/env/python

import rospy
from nav_msgs import Odometry
from geometry_msgs import PoseWithCovarianceStamped
from geometry_msgs import TwistWithCovarianceStamped
from std_msgs import Header

class amcl_pose_to_odom():
    def __init__(self):
        rospy.init_node('amcl_pose_to_odom', anonymous=False)
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.poseCallback)
        self.pub_odom = rospy.Publisher('vo_data', Odometry, queue_size=10)
        self.rate = rospy.Rate(100)

    def poseCallback(self, msg):
        pub_msg = Odometry()

        pub_msg.Header.frame_id = msg.Header.frame_id
        pub_msg.Header.stamp = msg.Header.stamp
        pub_msg.Header.seq = msg.Header.seq + 1

        pub_msg.pose = msg.pose

        twist = TwistWithCovarianceStamped()
        twist.covariance = [25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25]

        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        pub_msg.twist = twist

        pub_msg.child_frame_id = "base_footprint"

        rospy.logInfo(pub_msg)
        self.pub_odom.publish(pub_msg)


    def spin(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        amcl_pose_to_odom = amcl_pose_to_odom()
        amcl_pose_to_odom.spin()
    except rospy.ROSInterruptException:
        pass
