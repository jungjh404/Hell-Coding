#!/usr/bin/env python3

import rospy
import math
from tf.transformations import quaternion_from_euler
# from ar_track_alvar_msgs.msg import AlvarMarker
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult


class GoalManager:
    def __init__(self):
        rospy.init_node("Goal_Manager", annoymous=True)

        self.goal_cnt = 0
        self.goal_list = [[1, 0, 0], [7.5, 0, 90]]

        self.goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.reach_cb)
        

    def reach_cb(self, msg:MoveBaseActionResult):
        if msg.status.text == "Goal reached.":
            self.goal_cnt += 1

            next_goal_msg = PoseStamped()

            next_goal_msg.header.seq = self.goal_cnt
            next_goal_msg.header.stamp = rospy.Time.now()
            next_goal_msg.header.frame_id = "map"

            target = self.goal_list[self.goal_cnt]

            if target is not None:
                next_goal_msg.pose.position.x = target[0]
                next_goal_msg.pose.position.y = target[1]

                x, y, z, w = quaternion_from_euler(0, 0, math.radians(target[2]))
                next_goal_msg.pose.orientation.x = x
                next_goal_msg.pose.orientation.y = y
                next_goal_msg.pose.orientation.z = z
                next_goal_msg.pose.orientation.w = w


            self.goal_pub.publish(next_goal_msg)


if __name__ == "__main__":
    a = GoalManager()