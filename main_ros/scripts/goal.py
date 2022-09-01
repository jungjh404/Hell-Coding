#!/usr/bin/env python

import math
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Quaternion

class Goal:
    def __init__(self, x, y, yaw, via_points=None, x_diff=0, y_diff=0, inflation_rad=0.2, stop=False, lane=True):
        self.x = x
        self.y = y
        self.yaw = yaw

        self.via_points = via_points

        self.x_diff = x_diff
        self.y_diff = y_diff

        self.inflation_radius = inflation_rad

        self.stop_flag = stop
        self.lane_flag = lane
    
    def xy_to_point(self):
        return Point(self.x + self.x_diff, self.y + self.y_diff, 0)
    
    def yaw_to_quaternion(self):
        return Quaternion(*quaternion_from_euler(0, 0, math.radians(self.yaw)))
    
    