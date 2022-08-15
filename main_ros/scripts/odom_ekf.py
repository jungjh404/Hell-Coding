#! /usr/bin/env python

import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class OdomEKF():
    def __init__(self):
        rospy.init_node("odom_ekf", anonymous=True)

        self.ekf_pub = rospy.Publisher("ekf_odom", Odometry, queue_size=1)
        rospy.wait_for_message("odom_combined", PoseWithCovarianceStamped)
        rospy.Subscriber("odom_combined", PoseWithCovarianceStamped, self.ekf_odom_cb)
        rospy.loginfo("Publishing combined odometry")

    def ekf_odom_cb(self, msg):
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose = msg.pose

        self.ekf_pub.publish(odom)

if __name__ == "__main__":
    try:
        OdomEKF()
        rospy.spin()
    except:
        pass