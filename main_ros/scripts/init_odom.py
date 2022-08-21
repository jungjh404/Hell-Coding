#!/usr/bin/env python3


import rospy
from std_msgs.msg import Float64


class Init_Odom:
    def __init__(self):
        rospy.init_node("init_odom", anonymous=True)
        self.init_odom = rospy.Publisher("/sensors/servo_position_command", Float64, queue_size=1)

        self.run()


    def run(self):
        while self.init_odom.get_num_connections() < 1:
            msg = Float64()

            msg.data = 0.0

        self.init_odom.publish(msg)


if __name__ == "__main__":
    a = Init_Odom()