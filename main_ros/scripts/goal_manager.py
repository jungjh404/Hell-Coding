#!/usr/bin/env python


from telnetlib import DO
import rospy
import math
import actionlib
import tf
import threading
from goal import Goal
from ar_marker_pose import ar_marker_pose

from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseAction, MoveBaseActionGoal, MoveBaseActionFeedback
from nav_msgs.msg import Path
from std_srvs.srv import Empty
from dynamic_reconfigure.msg import Config, DoubleParameter
from dynamic_reconfigure.srv import Reconfigure
# from actionlib_msgs.msg import GoalID


class GoalManager:
    def __init__(self):
        self.goal_cnt = 0
        ## for mission driving
        # self.goal_list = [
        #                 Goal(x=0.612760,    y=0.299775,     yaw=math.degrees(-0.1275164)),        # start point
        #                 Goal(x=4.953308,    y=-0.18,        yaw=math.degrees(-0.1056652)),        # p-parking tag found
        #                 Goal(x=7.761093,    y=-0.477736,    yaw=math.degrees(-0.1040991)),        # p-parking exit
        #                 Goal(x=6.887442,    y=-1.127816,    yaw=math.degrees(-0.0906591)),        # p-parking complete
        #                 Goal(x=8.261093,    y=-0.477736,    yaw=math.degrees(-0.1040991)),        # p-parking exit
        #                 Goal(x=14.227601,   y=-1.209821,    yaw=math.degrees(0.8419409)),         # 1st corner (in ternnel)
        #                 Goal(x=14.748132,   y=1.421688,     yaw=math.degrees(1.4447077)),         # t-parking tag found
        #                 Goal(x=15.346061,   y=4.123437,     yaw=math.degrees(1.4601398)),         # just before t-parking complete
        #                 Goal(x=15.168897,   y=4.699220,     yaw=math.degrees(1.8925488)),         # rotation for t-parking start
        #                 Goal(x=15.766825,   y=3.591946,     yaw=math.degrees(3.0090416)),         # t-parking complete
        #                 Goal(x=15.157824,   y=4.957056,     yaw=math.degrees(1.4670029)),         # t-parking exit
        #                 Goal(x=15.763900,   y=7.646256,     yaw=math.degrees(2.4558618)),         # 2nd corner
        #                 Goal(x=9.746145,    y=8.729162,     yaw=math.degrees(3.0850491)),         # just before stop line
        #                 Goal(x=1.110168,    y=9.689941,     yaw=math.degrees(-2.6658999)),        # 3rd corner (just before warigari)
        #                 Goal(x=0.757480,    y=7.121173,     yaw=math.degrees(-1.5707963)),        # warigari 1
        #                 Goal(x=0.066044,    y=5.416704,     yaw=math.degrees(-1.675673)),         # warigari 2
        #                 Goal(x=0.355482,    y=3.358477,     yaw=math.degrees(-1.6099938)),        # warigari 3
        #                 Goal(x=-0.802270,   y=1.863046,     yaw=math.degrees(-1.5880358)),        # warigari 4
        #                 Goal(x=-0.705791,   y=0.978651,     yaw=math.degrees(-0.8345376))         # 4th corner (just before start 
        #                 ]

        # for speed driving
        # self.goal_list = [
        #                 Goal(x=0.612760,    y=0.299775,     yaw=math.degrees(-0.1275164)),        # start point
                    
        #                 Goal(x=14.527601,   y=-0.909821,    yaw=math.degrees(0.8419409)),         # 1st corner (in ternnel)
        #                 Goal(x=14.748132,   y=1.421688,     yaw=math.degrees(1.4447077)),         # t-parking tag found
        #                 Goal(x=15.763900,   y=7.646256,     yaw=math.degrees(2.4558618)),         # 2nd corner
        #                 Goal(x=0.310168,    y=9.689941,     yaw=math.degrees(-2.6658999)),        # 3rd corner (just before warigari)
        #                 Goal(x=0.757480,    y=7.121173,     yaw=math.degrees(-1.5707963)),        # warigari 1
        #                 Goal(x=0.066044,    y=5.416704,     yaw=math.degrees(-1.675673)),         # warigari 2
        #                 Goal(x=0.355482,    y=3.358477,     yaw=math.degrees(-1.6099938)),        # warigari 3
        #                 Goal(x=-0.802270,   y=1.863046,     yaw=math.degrees(-1.5880358)),        # warigari 4
        #                 Goal(x=-0.705791,   y=0.978651,     yaw=math.degrees(-0.8345376))         # 4th corner (just before start)
        #                 ]

        # for 0829
        # self.goal_list = [
        #                 Goal(x=14.748132,   y=1.421688,     yaw=math.degrees(1.4447077)),         # t-parking tag found
        #                 Goal(x=15.463900,   y=7.646256,     yaw=math.degrees(2.4558618)),         # 2nd corner
        #                 Goal(x=1.110168,    y=9.689941,     yaw=math.degrees(-2.6658999)),        # 3rd corner (just before warigari)
        #                 Goal(x=0.757480,    y=7.121173,     yaw=math.degrees(-1.5707963)),        # warigari 1
        #                 Goal(x=0.066044,    y=5.416704,     yaw=math.degrees(-1.675673)),         # warigari 2
        #                 Goal(x=0.355482,    y=3.358477,     yaw=math.degrees(-1.6099938)),        # warigari 3
        #                 Goal(x=-0.802270,   y=1.863046,     yaw=math.degrees(-1.5880358)),        # warigari 4
        #                 Goal(x=-0.705791,   y=0.978651,     yaw=math.degrees(-0.8345376))         # 4th corner (just before start)
        #                 ]

        ## for dong-bang
        self.goal_list=[
            Goal(x=11.0, y=0, yaw=180, via_points=[[10.2, 0.1], [10.4, 0.1]]),
            Goal(x=7.0, y=0.4, yaw=90, via_points=[[9, 0], [8, 0]])
        ]
        # [12.5, 0], [12, 0.3], [11.5, 0], [11, -0.3]
        self.goal_num = len(self.goal_list)
        self.marker_to_goal_dict ={1:0, 2:1, 8:2} # Goal: goal_idx
        self.rate = rospy.Rate(10)
        self.proximity_radius = 1.0 ##
        self.via_point_update_flag = False
        self.viapoint_cnt = 0
        

        # self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1)
        self.feedback_sub = rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.proximity_cb)
        self.via_pub = rospy.Publisher('/move_base/TebLocalPlannerROS/via_points', Path, queue_size=1)
        self.ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_cb)
        self.stop_line = False
        self.tf_listener = tf.TransformListener()
        self.via_point_thread = None

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        rospy.loginfo("Goal Manager Initialized.")

        if self.goal_list[self.goal_cnt].via_points is not None:
            # self.via_point_thread = threading.Thread(target=self.via_points_pub, args=(self.goal_list[self.goal_cnt].via_points,))
            # self.via_point_thread.start()

            self.via_point_thread = threading.Thread(target=self.viapoint_update_pub, args=(self.goal_list[self.goal_cnt].via_points, ))
            self.via_point_thread.start()
            rospy.loginfo("Thread Initialized!")

        self.res_sub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.reach_cb)

        self.goal_pub.publish(self.goal_msg_generate())
        rospy.spin()


    def viapoint_update_pub(self, via_points): ##
        r = rospy.Rate(5)

        via_points_msg = Path() 
        via_points_msg.header.stamp = rospy.Time.now()
        via_points_msg.header.frame_id = "map" # CHANGE HERE: odom/map
        via_points_msg.poses = []
        
        # descrete publication
        while not self.via_point_update_flag:
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
        srv = rospy.ServiceProxy('/move_base/local_costmap/inflation_layer/set_parameters', Reconfigure)
        msg = Config()

        param = DoubleParameter()
        param.name = 'inflation_radius'
        param.value = self.goal_list[self.goal_cnt].inflation_radius

        msg.doubles.append(param)
        res = srv(msg)


    def reach_cb(self, msg):
        if msg.status.text == "Goal reached." and int(msg.status.goal_id.id) == self.goal_cnt:
            self.goal_cnt += 1
            self.via_point_update_flag = True
            clearing = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            res = clearing()

            # # Closing remaining thread
            if self.via_point_thread is not None:
                self.via_point_thread.join()
                self.via_point_thread = None

            if self.goal_cnt >= self.goal_num:
                rospy.loginfo("Mission Finished.")
                exit(0)
            
            else:
                self.goal_pub.publish(self.goal_msg_generate())

                # Threading when via_point is available
                if self.goal_list[self.goal_cnt].via_points is not None:
                    self.via_point_thread = threading.Thread(target=self.viapoint_update_pub, args=(self.goal_list[self.goal_cnt].via_points))
                    self.via_point_thread.start()
        else:
            self.goal_pub.publish(self.goal_msg_generate())


    def ar_cb(self, msg):
        for marker in msg.markers:
            target = self.marker_to_goal_dict[marker.id]
            trans = ar_marker_pose(self.tf_listener, marker.id)
                
            target.x = trans[0]
            target.y = trans[1]


    def proximity_cb(self, msg):
        proximity_dist = math.sqrt(math.pow((msg.feedback.base_position.pose.position.x - self.goal_list[self.goal_cnt].x), 2) 
                                    + math.pow((msg.feedback.base_position.pose.position.y - self.goal_list[self.goal_cnt].y), 2))
        rospy.loginfo("Proximity: %f", proximity_dist)

        if proximity_dist < self.proximity_radius:
            self.via_point_update_flag = True
        rospy.loginfo(self.via_point_update_flag)


if __name__ == "__main__":
    try:
        rospy.init_node("goal_manager", anonymous=True)
        a = GoalManager()
    except KeyboardInterrupt:
        exit(0)
