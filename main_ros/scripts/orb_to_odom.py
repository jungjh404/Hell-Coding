#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry


class ORB_ODOM:
    def __init__(self):
        rospy.init_node("orb_2_odom", anonymous=True)

        rospy.wait_for_message("/orb_slam2_mono/pose", PoseStamped)
        rospy.Subscriber("/orb_slam2_mono/pose", PoseStamped, self.orb_cb)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_cb)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)

        self.cur_twist = Twist()
        self.cur_pose = PoseStamped()

        self.rate = rospy.Rate(10)

        self.run()


    def run(self):
        while not rospy.is_shutdown():
            msg = Odometry()

            msg.header.stamp = rospy.Time.now()

            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_link"

            msg.pose.pose = self.cur_pose.pose
            msg.twist.twist = self.cur_twist

            self.odom_pub.publish(msg)
            self.rate.sleep()


    def cmd_cb(self, msg: Twist):
        self.cur_twist = msg


    def orb_cb(self, msg: PoseStamped):
        self.cur_pose = msg

if __name__ == "__main__":
    a = ORB_ODOM()