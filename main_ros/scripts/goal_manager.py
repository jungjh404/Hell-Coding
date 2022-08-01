#!/usr/bin/env python3

import rospy
from ar_track_alvar_msgs.msg import AlvarMarker
from geomtry_msgs.msg import PoseStamped

class GoalManager:
    def __init__(self):
        rospy.init_node("Goal_Manager", annoymous=True)
        rospy.Subscriber()
        self.goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)