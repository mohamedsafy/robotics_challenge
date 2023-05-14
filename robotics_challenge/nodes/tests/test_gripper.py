#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState


def callback(msg):
    if 0.004 < msg.position[0] < 0.010  :
        print('ball is picked !', msg.position[0])
    else:
        print('ball not picked ! ', msg.position[0])

rospy.init_node('test_gripper_gap')

joint_state = rospy.wait_for_message('/joint_states', JointState, callback)
if 0.004 < joint_state.position[0] < 0.010  :
    print('ball is picked !', joint_state.position[0])
else:
    print('ball not picked ! ', joint_state.position[0])

rospy.spin()