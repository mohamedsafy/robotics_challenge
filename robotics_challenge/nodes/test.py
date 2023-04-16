#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo

camera = PinholeCameraModel()
def callback(info):
	camera.fromCameraInfo(info)
	
	pixel = (181,325)
	print(pixel)
	point = camera.projectPixelTo3dRay(pixel)
	print(point)

def main():
	rospy.init_node('test_node')
	

	sub = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, callback)
	rospy.spin()

if __name__=='__main__':
	main()


