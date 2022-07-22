import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped
from nav_msg.msg import Odometry

class Pose :
    def __init__(self):
        rospy.init_node("pose_var_parser", anonymous=True)

        self.cnt = 0

        self.x = 0
        self.y = 0
        self.r = 0

        self.avg_x = 0
        self.avg_y = 0
        self.avg_r = 0

        self.d_avg_x = 0
        self.d_avg_y = 0
        self.d_avg_r = 0

        self.var_x = 0
        self.var_y = 0
        self.var_r = 0

        
        rospy.wait_for_message("/orb_slam/pose", PoseStamped)
        rospy.Subscriber("/orb_slam/pose", PoseStamped, self.callback)
        self.pub = rospy.Publisher("odom", Odometry, queue_size=1)


    def avg(self) :
        self.avg_x = ((self.avg_x * (self.cnt-1)) + self.x) / self.cnt
        self.avg_y = ((self.avg_y * (self.cnt-1)) + self.y) / self.cnt
        self.avg_r = ((self.avg_r * (self.cnt-1)) + self.r) / self.cnt


    def double_avg(self) :
        self.d_avg_x = ((self.d_avg_x * (self.cnt-1)) + self.x ** 2) / self.cnt
        self.d_avg_y = ((self.d_avg_y * (self.cnt-1)) + self.y ** 2) / self.cnt
        self.d_avg_r = ((self.d_avg_r * (self.cnt-1)) + self.r ** 2) / self.cnt


    def var(self) :
        self.var_x = self.d_avg_x - self.avg_x ** 2
        self.var_y = self.d_avg_y - self.avg_y ** 2
        self.var_r = self.d_avg_r - self.avg_r ** 2
    

    def callback(self, msg):
        q_x = msg.pose.orientation.x
        q_y = msg.pose.orientation.y
        q_z = msg.pose.orientation.z
        q_w = msg.pose.orientation.w

        yaw = np.atan2(2.0*(q_y*q_z + q_w*q_x), q_w*q_w - q_x*q_x - q_y*q_y + q_z*q_z)

        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.r = yaw
        self.cnt += 1

    
    def publish(self):
        msg = Odometry()
        msg[0] = self.var_x
        msg[7] = self.var_y
        msg[14] = 99999
        msg[21] = 99999
        msg[28] = 99999
        msg[35] = self.var_r
        self.pub.publish(msg)

# msg.pose.covariance = {cov_x, 0, 0, 0, 0, 0,
# 0, cov_y, 0, 0, 0, 0,
# 0, 0, 99999, 0, 0, 0,
# 0, 0, 0, 99999, 0, 0,
# 0, 0, 0, 0, 99999, 0,
# 0, 0, 0, 0, 0, rcov_z}