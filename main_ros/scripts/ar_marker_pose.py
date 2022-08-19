#!/usr/bin/env python

import rospy
import math
from tf.transformations import euler_from_quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped


def ar_marker_pose(tf_listener, id):
    target_frame = "ar_marker_"+str(id)
    src_frame = "map"

    tf_listener.waitForTransform(src_frame, target_frame, rospy.Time(), rospy.Duration(1.0))
    try:
        now = rospy.Time.now()
        tf_listener.waitForTransform(target_frame, src_frame, now, rospy.Duration(1.0))
        trans, rot = tf_listener.lookupTransform(src_frame, target_frame, now)

    except Exception as E:
        rospy.logwarn(E)
        
    return trans

if __name__ == "__main__":
    import tf
    from ar_track_alvar_msgs.msg import AlvarMarkers
    
    rospy.init_node("ar_marker_pose", anonymous=True)
    alvar_msg = rospy.wait_for_message("/ar_pose_marker", AlvarMarkers, timeout=3)
    tf_listener = tf.TransformListener()

    trans = ar_marker_pose(tf_listener, alvar_msg.markers[0].id)

    # axes_dict = {
    # 'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    # 'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    # 'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    # 'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    # 'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    # 'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    # 'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    # 'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

    # for axes in axes_dict.keys():
    #     r,p,y = euler_from_quaternion(rot, axes=axes)
    #     print(axes, math.degrees(r), math.degrees(p), math.degrees(y))
    