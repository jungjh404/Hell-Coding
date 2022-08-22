#!/usr/bin/env python

from tf.transformations import quaternion_from_euler

class Goal:
    def __init__(self, x, y, yaw, via_points=None, x_diff=None, y_diff=None):
        self.x = x
        self.y = y
        self.yaw = yaw

        self.via_points = via_points

        self.x_diff = None
        self.y_diff = None
    
    def 