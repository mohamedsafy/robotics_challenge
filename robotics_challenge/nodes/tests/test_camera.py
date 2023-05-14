#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point,Pose
from tf2_geometry_msgs import PoseStamped
from image_geometry import PinholeCameraModel


rospy.init_node('test_camera_node')

def image_callback(img):
    image_rect_pub.publish(img)

def camera_info_callback(info):
    camera_info_pub.publish(info)

camera_info_pub = rospy.Publisher('/camera_info', CameraInfo, queue_size=14)
camera_info_sub = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, camera_info_callback)

image_rect_pub = rospy.Publisher('/image_raw', Image, queue_size=14)
image_raw_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)

rospy.spin()


