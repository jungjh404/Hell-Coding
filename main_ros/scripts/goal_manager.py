#!/usr/bin/env python


import rospy
import math
import actionlib
import tf
import threading
from goal import Goal
from ar_marker_pose import ar_marker_pose

from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseAction, MoveBaseActionGoal, MoveBaseFeedback
from nav_msgs.msg import Path
from std_srvs.srv import Empty
# from actionlib_msgs.msg import GoalID


class GoalManager:
    def __init__(self):
        self.goal_cnt = 0
        self.goal_list = [
                        Goal(x=0.612760,    y=0.399775,     yaw=math.degrees(-0.1275164)),        # start point
                        Goal(x=4.953308,    y=0.020134,     yaw=math.degrees(-0.1056652)),        # p-parking tag found
                        Goal(x=6.787442,    y=-1.127816,    yaw=math.degrees(-0.0906591)),        # p-parking complete
                        Goal(x=8.261093,    y=-0.277736,    yaw=math.degrees(-0.1040991)),        # p-parking exit
                        Goal(x=14.227601,   y=-1.209821,    yaw=math.degrees(0.8419409)),         # 1st corner (in ternnel)
                        Goal(x=14.748132,   y=1.421688,     yaw=math.degrees(1.4447077)),         # t-parking tag found
                        Goal(x=15.346061,   y=4.123437,     yaw=math.degrees(1.4601398)),         # just before t-parking complete
                        Goal(x=15.168897,   y=4.699220,     yaw=math.degrees(1.8925488)),         # rotation for t-parking start
                        Goal(x=15.766825,   y=3.591946,     yaw=math.degrees(3.0090416)),         # t-parking complete
                        Goal(x=15.157824,   y=4.957056,     yaw=math.degrees(1.4670029)),         # t-parking exit
                        Goal(x=15.763900,   y=7.646256,     yaw=math.degrees(2.4558618)),         # 2nd corner
                        Goal(x=9.746145,    y=8.729162,     yaw=math.degrees(3.0850491)),         # just before stop line
                        Goal(x=1.110168,    y=9.689941,     yaw=math.degrees(-2.6658999)),        # 3rd corner (just before warigari)
                        Goal(x=0.757480,    y=7.121173,     yaw=math.degrees(-1.5707963)),        # warigari 1
                        Goal(x=0.066044,    y=5.416704,     yaw=math.degrees(-1.675673)),         # warigari 2
                        Goal(x=0.355482,    y=3.358477,     yaw=math.degrees(-1.6099938)),        # warigari 3
                        Goal(x=-0.802270,   y=1.863046,     yaw=math.degrees(-1.5880358)),        # warigari 4
                        Goal(x=-0.705791,   y=0.978651,     yaw=math.degrees(-0.8345376))         # 4th corner (just before start 
                        ]
        self.goal_num = len(self.goal_list)
        self.marker_to_goal_dict ={1:0, 2:1, 8:2} # Goal: goal_idx
        self.rate = rospy.Rate(10)
        self.proximity_radius = 1.0 ##
        self.via_point_update_flag = False
        

        # self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1)
        # self.feedback_sub = rospy.Publisher("/move_base/feedback", MoveBaseFeedback, self.proximity_cb)
        self.res_sub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.reach_cb)
        self.via_pub = rospy.Publisher('/move_base/TebLocalPlannerROS/via_points', Path, queue_size=1)
        self.ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_cb)
        self.stop_line = False
        self.tf_listener = tf.TransformListener()
        self.via_point_thread = None

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        rospy.loginfo("Goal Manager Initialized.")

        if type(self.goal_list[self.goal_cnt]) == list and self.goal_list[self.goal_cnt][3] is not None:
            rospy.loginfo("Thread Initialized!")
            # self.via_point_thread = threading.Thread(target=self.via_points_pub, args=(self.goal_list[self.goal_cnt].via_points,))
            # self.via_point_thread.start()

            self.via_point_thread = threading.Thread(target=self.viapoint_update_pub, args=(self.goal_list[self.goal_cnt].via_points,))
            self.via_point_thread.start()

        self.goal_pub.publish(self.goal_msg_generate())
        rospy.spin()

    def via_points_pub(self, via_points):
        via_points_msg = Path() 
        via_points_msg.header.stamp = rospy.Time.now()
        via_points_msg.header.frame_id = "odom" # CHANGE HERE: odom/map

        # Add via-points
        for point in via_points:
            point_msg = PoseStamped()
            point_msg.pose.position.x = point[0]
            point_msg.pose.position.y = point[1]
            via_points_msg.poses.append(point_msg)

        r = rospy.Rate(5) # 10hz
        t = 0.0

        while 1:
            self.via_pub.publish(via_points_msg)
            print("pub")
            r.sleep()

    def viapoint_update_pub(self, via_points): ##
        viapoint_idx = 0
        len_viapoints = len(via_points)
        r = rospy.Rate(5)

        via_points_msg = Path() 
        via_points_msg.header.stamp = rospy.Time.now()
        via_points_msg.header.frame_id = "odom" # CHANGE HERE: odom/map
        next_viapoint = PoseStamped()
        

        while viapoint_idx < len_viapoints:
            if self.via_point_update_flag:
                next_viapoint.pose.position.x = via_points[viapoint_idx][0]
                next_viapoint.pose.position.y = via_points[viapoint_idx][1]

                via_points_msg.poses = [next_viapoint]

                self.via_pub.publish(via_points_msg)
                rospy.loginfo("current viapoint: (%f, %f)", next_viapoint.pose.position.x , next_viapoint.pose.position.y)
                self.via_point_update_flag = False
                r.sleep()


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
        goal.goal.target_pose.pose.orientation = target.yaw_to_quaternion() ##########typeerror###########

        # self.viapoint_update_pub(target.via_points) ##
        
        rospy.loginfo("#%d: %f %f %f", self.goal_cnt, target.x+target.x_diff, target.y+target.y_diff, target.yaw)
        return goal


    def reach_cb(self, msg):
        if msg.status.text == "Goal reached." and int(msg.status.goal_id.id) == self.goal_cnt:
            self.goal_cnt += 1

            clearing = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            res = clearing()

            # Closing remaining thread
            if self.via_point_thread is not None:
                self.via_point_thread.join()
                self.via_point_thread = None

            if self.goal_cnt >= self.goal_num:
                rospy.loginfo("Mission Finished.")
                exit(0)
            
            else:
                # Threading when via_point is available
                if self.goal_list[self.goal_cnt].via_points is not None:
                    self.via_point_thread = threading.Thread(target=self.viapoint_update_pub, args=(self.via_points[self.goal_list[self.goal_cnt].via_points]))
                    self.via_point_thread.start()

                self.goal_pub.publish(self.goal_msg_generate())

        else:
            self.goal_pub.publish(self.goal_msg_generate())


    def ar_cb(self, msg):
        for marker in msg.markers:
            target = self.marker_to_goal_dict[marker.id]
            trans = ar_marker_pose(self.tf_listener, marker.id)
                
            target.x = trans[0]
            target.y = trans[1]

    def proximity_cb(self, msg):
        proximity_dist = math.sqrt(math.pow((msg.position.x - self.goal_list[self.goal_cnt].via_points[self.viapoint_cnt][0]), 2) 
                                    + math.pow((msg.position.y - self.goal_list[self.goal_cnt].via_points[self.viapoint_cnt][1]), 2))

        if proximity_dist < self.proximity_radius:
            self.via_point_update_flag = True

if __name__ == "__main__":
    try:
        rospy.init_node("goal_manager", anonymous=True)
        a = GoalManager()
    except KeyboardInterrupt:
        exit(0)