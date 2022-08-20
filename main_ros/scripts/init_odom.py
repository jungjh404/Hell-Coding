#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist

class Init_Odom:
    def __init__(self):
        rospy.init_node("init_odom", anonymous=True)

        self.init_odom = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.rate = rospy.Rate(10)

        self.run()


    def run(self):
        while self.init_odom.get_num_connections() < 1:
            msg = Twist()
            print(msg)
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 0

        self.init_odom.publish(msg)


if __name__ == "__main__":
    a = Init_Odom()