#!/usr/bin/env python3

import rospy
import tf
import roslib

MAP_UNIT_MODIFIER = 1
X_MAP_OFFSET = 1.25 #1.75


roslib.load_manifest('robotics_challenge')

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((MAP_UNIT_MODIFIER * X_MAP_OFFSET, 0.0, 0.0),
                        (0.0, 0.0, 0.0, 1.0),
                        rospy.Time.now(),
                        "planner",
                        "map")
        rospy.sleep(0.5)
    rate.sleep()