#!/usr/bin/env python3
import rospy
import tf2_ros
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

rospy.init_node('test_angle_node')

x,y = -1,-0.5

robot_pos = rospy.wait_for_message('/odom', Odometry, rospy.Duration(1))
xo,yo = robot_pos.pose.pose.position.x, robot_pos.pose.pose.position.y
s= (y-yo)/(x-xo)

theta = math.atan(s)
print(theta)

if(theta < 0):
    theta = theta +math.pi
elif theta == 0:
    if x < xo:
        theta = theta + math.pi

if(y - yo < 0):
    theta = theta + math.pi

w = math.cos(theta/2)
z = math.sin(theta/2)


print(w,z)
