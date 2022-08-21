#!/usr/bin/env python


import rospy
from ackermann_msgs.msg import AckermannDriveStamped

class Init_Odom_ackr:
    def __init__(self):
        rospy.init_node("init_odom_acker", anonymous=True)

        self.init_odom_ackr = rospy.Publisher("/xycar_motor", AckermannDriveStamped, queue_size=1)


    def run(self):
    
        msg = AckermannDriveStamped()
        
        msg.drive.steering_angle = 0
        msg.drive.steering_angle_velocity = 0
        msg.drive.speed = 0
        msg.drive.acceleration = 0
        msg.drive.jerk = 0

        self.init_odom_ackr.publish(msg)


if __name__ == "__main__":
    a = Init_Odom_ackr()
    while a.init_odom_ackr.get_num_connections() < 1:
        a.run()
        break