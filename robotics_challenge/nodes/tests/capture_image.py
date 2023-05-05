#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    input("capture image ?")
    cv2.imwrite('image.jpg', cv_image)
    

rospy.init_node('camera_capture_node')
rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)
rospy.spin()
