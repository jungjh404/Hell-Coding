#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers


def ar_marker_pose(tf_listener, id):
    target_frame = "ar_marker_"+str(id)
    src_frame = "map"

    tf_listener.waitForTransform(src_frame, target_frame, rospy.Time(), rospy.Duration(1.0))
    tf_listener.waitForTransform(target_frame, src_frame, rospy.Time(), rospy.Duration(1.0))

    try:
        now = rospy.Time.now()
        tf_listener.waitForTransform(src_frame, target_frame, now, rospy.Duration(1.0))
        trans, rot = tf_listener.lookupTransform(src_frame, target_frame, now)
        print(trans, rot)

        tf_listener.waitForTransform(target_frame, src_frame, now, rospy.Duration(1.0))
        trans, rot = tf_listener.lookupTransform(target_frame, src_frame, now)
        print(trans, rot)

    except Exception as E:
        rospy.logwarn(E)
        
    return trans, rot

if __name__ == "__main__":
    import tf
    from ar_track_alvar_msgs.msg import AlvarMarkers
    
    rospy.init_node("ar_marker_pose", anonymous=True)
    alvar_msg = rospy.wait_for_message("/ar_pose_marker", AlvarMarkers, timeout=3)
    tf_listener = tf.TransformListener()

    ar_marker_pose(tf_listener, alvar_msg.markers[0].id)
    