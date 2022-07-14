#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

imu_data = open('imu_data.txt', 'w')
cnt = 0
def callback(msg):
    global imu_data
    global cnt

    if cnt < 100000:
        cnt+=1
        line = f'{msg.orientation.x},{msg.orientation.y},{msg.orientation.z},{msg.angular_velocity.x},{msg.orientation.y},{msg.orientation.z},{msg.linear_acceleration.x},{msg.linear_acceleration.y},{msg.linear_acceleration.z}\n'
        imu_data.write(line)
    
    else:
        imu_data.close()
        exit(0)

rospy.init_node("imu_parser", anonymous=True)
rospy.Subscriber("imu", Imu, callback)
rospy.spin()