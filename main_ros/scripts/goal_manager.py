#!/usr/bin/env python


import rospy
import math
import actionlib
import tf
import threading
import sys
from goal import Goal
from stopline import Stopline
from ar_marker_pose import ar_marker_pose
from lane_scan import LaneScan

from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseAction, MoveBaseActionGoal, MoveBaseActionFeedback
from nav_msgs.msg import Path
from std_srvs.srv import Empty
from dynamic_reconfigure.msg import Config, DoubleParameter
from dynamic_reconfigure.srv import Reconfigure
# from actionlib_msgs.msg import GoalID


class GoalManager:
    def __init__(self, idx):
        self.goal_cnt = idx

        ##### M I S S I O N  G O A L #####
        # Goal(x, y, yaw, inflation_rad=0.2, stop=False)
        # Goal(x=14.858,  y=4.281,     yaw=103.631,    inflation_rad=0.05),           
        # self.goal_list = [
        #                 Goal(x=8.045,   y=-0.398,   yaw=-5.105,  inflation_rad=0.05),               #0 before p-parking
        #                 Goal(x=6.654,   y=-1.281,   yaw=-3.019, inflation_rad=0.05),                #1 p-parking complete
        #                 Goal(x=8.698,   y=-0.424,     yaw=-4.874, inflation_rad=0.05),              #2 after p-parking
        #                 Goal(x=14.631,  y=0.974,     yaw=84.169, inflation_rad=0.05),               #3 after tunnel
        #                 Goal(x=14.858,  y=4.281,     yaw=84.169,    inflation_rad=0.05),            #4 before t-parking
        #                 Goal(x=16.086,  y=3.659,    yaw=174.035, inflation_rad=0.0),                #5 t-parking complete
        #                 Goal(x=13.750,    y=8.025,     yaw=170.188, inflation_rad=0.05),            #7 before_obstacle
        #                 Goal(x=9.068,    y=8.523,     yaw=173.320, inflation_rad=0.05),             #8 after_obstacle
        #                 Goal(x=1.802,    y=9.381,     yaw=174.721, inflation_rad=0.05, stop=True),  #9 stop_line
        #                 Goal(x=1.310,    y=0.269,     yaw=-6.357, inflation_rad=0.05)               #10 s-curve
        #                 ]
        
        self.goal_list = [
                        Goal(x=14.851,  y=3.382,     yaw=82.142,    inflation_rad=0.05),            #0 up
                        Goal(x=3.396,    y=9.266,     yaw=175.133, inflation_rad=0.05),             #1 before s-curve
                        Goal(x=1.310,    y=0.269,     yaw=-6.357, inflation_rad=0.05),              #2 lap 1
                        Goal(x=14.851,  y=3.382,     yaw=82.142,    inflation_rad=0.05),            #3 up
                        Goal(x=3.396,    y=9.266,     yaw=175.133, inflation_rad=0.05),             #4 before s-curve
                        Goal(x=1.310,    y=0.269,     yaw=-6.357, inflation_rad=0.05),              #5 lap 2
                        Goal(x=14.851,  y=3.382,     yaw=82.142,    inflation_rad=0.05),            #6 up
                        Goal(x=3.396,    y=9.266,     yaw=175.133, inflation_rad=0.05),             #7 before s-curve
                        Goal(x=1.310,    y=0.269,     yaw=-6.357, inflation_rad=0.05)               #8 lap 3
                        ]


        self.goal_num = len(self.goal_list)
        # self.marker_to_goal_dict = {1:1, 2:3, 8:7} # Goal: goal_idx
        self.rate = rospy.Rate(10)
        self.proximity_radius = 0.4 ##
        self.via_point_update_flag = False
        
        self.stop_node = Stopline()
        # self.lane_node = LaneScan()

        self.goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1)
        # self.feedback_sub = rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.proximity_cb)
        # self.via_pub = rospy.Publisher('/move_base/TebLocalPlannerROS/via_points', Path, queue_size=1)
        # self.ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_cb)
        self.tf_listener = tf.TransformListener()
        self.via_point_thread = None

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        
        rospy.wait_for_service('/move_base/local_costmap/local_inflation_layer/set_parameters')

        rospy.loginfo("Goal Manager Initialized.")

        # if self.goal_list[self.goal_cnt].via_points is not None:

        #     self.via_point_thread = threading.Thread(target=self.viapoint_update_pub, args=(self.goal_list[self.goal_cnt].via_points, ))
        #     self.via_point_thread.start()
        #     rospy.loginfo("Thread Initialized!")

        self.res_sub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.reach_cb)

        self.inflation_rad_call()
        # self.lane_scan_pub()
        self.goal_pub.publish(self.goal_msg_generate())
        # rospy.spin()


    def viapoint_update_pub(self, via_points): ##
        r = rospy.Rate(5)

        # descrete publication
        while not self.via_point_update_flag:
            via_points_msg = Path() 
            via_points_msg.header.stamp = rospy.Time.now()
            via_points_msg.header.frame_id = "map" # CHANGE HERE: odom/map
            via_points_msg.poses = []

            for via_point in via_points:
                next_viapoint = PoseStamped()
                next_viapoint.pose.position.x = via_point[0]
                next_viapoint.pose.position.y = via_point[1]
            
                via_points_msg.poses.append(next_viapoint)

            self.via_pub.publish(via_points_msg)
            
            rospy.loginfo("Via Point Pub")
            r.sleep()

        rospy.loginfo("Via Point Thread OFF!")


    def goal_msg_generate(self):
        goal = MoveBaseActionGoal()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.header.seq = self.goal_cnt
        goal.goal_id.id = str(self.goal_cnt)

        goal.goal.target_pose.header.frame_id = "map"
        goal.goal.target_pose.header.stamp.secs = goal.header.stamp.secs
        goal.goal.target_pose.header.stamp.nsecs = goal.header.stamp.nsecs
        goal.goal.target_pose.header.seq = self.goal_cnt

        target = self.goal_list[self.goal_cnt]

        goal.goal.target_pose.pose.position = target.xy_to_point()
        goal.goal.target_pose.pose.orientation = target.yaw_to_quaternion()

        # self.viapoint_update_pub(target.via_points) ##
        
        rospy.loginfo("#%d: %f %f %f", self.goal_cnt, target.x+target.x_diff, target.y+target.y_diff, target.yaw)

        return goal

    def inflation_rad_call(self):
        srv = rospy.ServiceProxy('/move_base/local_costmap/local_inflation_layer/set_parameters', Reconfigure)
        msg = Config()

        param = DoubleParameter()
        param.name = 'inflation_radius'
        param.value = self.goal_list[self.goal_cnt].inflation_radius

        msg.doubles.append(param)
        res = srv(msg)

    def lane_scan_pub(self):
        self.lane_node.lane_pub_flag = self.goal_list[self.goal_cnt].lane_flag

    
    def reach_cb(self, msg):
        rospy.sleep(1.)
        if msg.status.text == "Goal reached." and int(msg.status.goal_id.id) == self.goal_cnt:
            self.goal_cnt += 1

            if self.goal_cnt >= self.goal_num:
                rospy.loginfo("Mission Finished.")
                sys.exit(0)
            
            else:
                self.inflation_rad_call()
                # self.lane_scan_pub()
                self.goal_pub.publish(self.goal_msg_generate())

                # Threading when via_point is available
        else:
            self.goal_pub.publish(self.goal_msg_generate())


    def ar_cb(self, msg):
        for marker in msg.markers:
            if marker.id not in self.marker_to_goal_dict.keys():
                continue

            target = self.marker_to_goal_dict[marker.id]
            trans = ar_marker_pose(self.tf_listener, marker.id)

            if trans[0] is not None:
                self.goal_list[target].x = trans[0]
                self.goal_list[target].y = trans[1]

            # if marker.id == 1:
            #     self.goal_list[target].x = trans[0] + 1.700
            #     self.goal_list[target].y = trans[1] - 0.45
            #     self.goal_list[target + 1].x = trans[0] + 0.595
            #     self.goal_list[target + 1].y = trans[1] + 0.352
            
            # if marker.id == 2:
            #     self.goal_list[target].x = trans[0] - 0.905
            #     self.goal_list[target].y = trans[1]

            # if marker.id == 8:
            #     self.goal_list[target].x = trans[0] - 0.5
            #     self.goal_list[target].y = trans[1] + 2.3
            #     self.goal_list[target + 2].x = trans[0] + 0.65
            #     self.goal_list[target + 2].y = trans[1] + 1.5


    def proximity_cb(self, msg):
        if self.goal_cnt >= self.goal_num:
            return

        proximity_dist = math.sqrt(math.pow((msg.feedback.base_position.pose.position.x - self.goal_list[self.goal_cnt].x), 2) 
                                    + math.pow((msg.feedback.base_position.pose.position.y - self.goal_list[self.goal_cnt].y), 2))

        if proximity_dist < self.proximity_radius:
            self.via_point_update_flag = True


if __name__ == "__main__":
    try:
        rospy.init_node("goal_manager", anonymous=True)
        a = GoalManager(0)
    except KeyboardInterrupt:
        exit(0)
