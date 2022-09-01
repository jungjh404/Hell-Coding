#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

rospy.init_node("Clear_Costmap", anonymous=True)
r = rospy.Rate(0.5)
rospy.wait_for_service("/move_base/clear_costmaps")
while not rospy.is_shutdown():
    clearing = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
    res = clearing()
    r.sleep()
    
