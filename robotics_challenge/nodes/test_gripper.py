#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState


def callback(msg):
    if 0.004 < msg.position[0] < 0.006  :
        print('ball is picked !', msg.position[0])
    else:
        print('ball not picked ! ', msg.position[0])

rospy.init_node('test_gripper_gap')

sub = rospy.Subscriber('/joint_states', JointState, callback)

rospy.spin()