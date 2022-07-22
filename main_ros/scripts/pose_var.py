#!/usr/bin/env python3


import rospy
import math
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class Pose:
    def __init__(self):
        rospy.init_node("pose_var_parser", anonymous=True)

        self.cnt = 1

        self.x = 0
        self.prev_x = 0
        
        self.y = 0
        self.prev_y = 0

        self.r = 0
        self.prev_r = 0

        self.yaw = 0

        self.avg_x = 0
        self.avg_y = 0
        self.avg_r = 0

        self.d_avg_x = 0
        self.d_avg_y = 0
        self.d_avg_r = 0

        self.var_x = 0
        self.var_y = 0
        self.var_r = 0

        self.twist_x = 0
        self.prev_twist_x = 0
        
        self.twist_y = 0
        self.prev_twist_y = 0

        self.ang_twist_z = 0
        self.prev_ang_twist_z = 0
        
        self.avg_twist_x = 0
        self.avg_twist_y = 0
        self.avg_ang_twist_z = 0

        self.d_avg_twist_x = 0
        self.d_avg_twist_y = 0
        self.d_avg_ang_twist_z = 0

        self.var_twist_x = 0
        self.var_twist_y = 0
        self.var_ang_twist_z = 0

        self.dif_x = 0
        self.dif_y = 0

        self.dt = 0.1
        
        rospy.wait_for_message("/orb_slam2_mono/pose", PoseStamped)
        rospy.loginfo("Message Received Sucessfully")
        rospy.Subscriber("/orb_slam2_mono/pose", PoseStamped, self.callback)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initcallback)
        self.pub = rospy.Publisher("/odom", Odometry, queue_size=1)
        rospy.spin()

    def initcallback(self, msg):
        self.dif_x = msg.pose.pose.position.x
        self.dif_y = msg.pose.pose.position.y

        r = rospy.Rate(1/self.dt)
        r.sleep()

    def avg(self):
        self.avg_x = ((self.avg_x * (self.cnt-1)) + self.x) / self.cnt
        self.avg_y = ((self.avg_y * (self.cnt-1)) + self.y) / self.cnt
        self.avg_r = ((self.avg_r * (self.cnt-1)) + self.r) / self.cnt


    def double_avg(self):
        self.d_avg_x = ((self.d_avg_x * (self.cnt-1)) + self.x ** 2) / self.cnt
        self.d_avg_y = ((self.d_avg_y * (self.cnt-1)) + self.y ** 2) / self.cnt
        self.d_avg_r = ((self.d_avg_r * (self.cnt-1)) + self.r ** 2) / self.cnt


    def var(self):
        self.var_x = self.d_avg_x - self.avg_x ** 2
        self.var_y = self.d_avg_y - self.avg_y ** 2
        self.var_r = self.d_avg_r - self.avg_r ** 2


    def twist_avg(self):
        self.avg_twist_x = ((self.avg_twist_x * (self.cnt-1)) + self.twist_x) / self.cnt
        self.avg_twist_y = ((self.avg_twist_y * (self.cnt-1)) + self.twist_y) / self.cnt
        self.avg_ang_twist_z = ((self.avg_ang_twist_z * (self.cnt-1)) + self.ang_twist_z) / self.cnt


    def twist_double_avg(self):
        self.d_avg_twist_x = ((self.d_avg_twist_x * (self.cnt-1)) + self.twist_x ** 2) / self.cnt
        self.d_avg_twist_y = ((self.d_avg_twist_y * (self.cnt-1)) + self.twist_y ** 2) / self.cnt
        self.d_avg_ang_twist_z = ((self.d_avg_ang_twist_z * (self.cnt-1)) + self.ang_twist_z ** 2) / self.cnt


    def twist_var(self):
        self.var_twist_x = self.d_avg_twist_x - self.avg_twist_x ** 2
        self.var_twist_y = self.d_avg_twist_y - self.avg_twist_y ** 2
        self.var_ang_twist_z = self.d_avg_ang_twist_z - self.avg_ang_twist_z ** 2
    

    def callback(self, msg):
        q_x = msg.pose.orientation.x
        q_y = msg.pose.orientation.y
        q_z = msg.pose.orientation.z
        q_w = msg.pose.orientation.w

        self.yaw = math.atan2(2.0*(q_y*q_z + q_w*q_x), q_w*q_w - q_x*q_x - q_y*q_y + q_z*q_z)

        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_r = self.r

        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.r = self.yaw

        self.prev_twist_x = self.twist_x
        self.prev_twist_y = self.twist_y
        self.prev_ang_twist_z = self.ang_twist_z

        vel_x = (self.x - self.prev_x) / self.dt
        vel_y = (self.y - self.prev_y) / self.dt

        self.twist_x = math.cos(self.yaw) * vel_x + math.sin(self.yaw) * vel_y
        self.twist_y = math.cos(self.yaw) * vel_y - math.sin(self.yaw) * vel_x
        self.ang_twist_z = (self.r - self.prev_r) / self.dt

        self.avg()
        self.double_avg()
        self.var()

        self.twist_avg()
        self.twist_double_avg()
        self.twist_var()
        
        self.cnt += 1

        self.publish()
        
        r = rospy.Rate(1/self.dt)
        r.sleep()

    
    def publish(self):
        msg = Odometry()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.child_frame_id = "base_link"
        
        msg.pose.pose.position.x = self.x + self.dif_x
        msg.pose.pose.position.y = self.y + self.dif_y
        
        msg.pose.covariance[0] = self.var_x
        msg.pose.covariance[7] = self.var_y
        msg.pose.covariance[14] = 99999
        msg.pose.covariance[21] = 99999
        msg.pose.covariance[28] = 99999
        msg.pose.covariance[35] = self.var_r

        msg.twist.covariance[0] = self.var_twist_x
        msg.twist.covariance[7] = self.var_twist_y
        msg.twist.covariance[14] = 99999
        msg.twist.covariance[21] = 99999
        msg.twist.covariance[28] = 99999
        msg.twist.covariance[35] = self.var_ang_twist_z

        msg.twist.twist.linear.x = self.twist_x
        msg.twist.twist.linear.y = self.twist_y
        msg.twist.twist.angular.z = self.ang_twist_z
        
        self.pub.publish(msg)
    

if __name__ == "__main__":
    _ = Pose()