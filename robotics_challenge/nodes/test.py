#!/usr/bin/env python3

import rospy
import math
import numpy as np
import tf2_ros
from geometry_msgs.msg import Point, PointStamped
from opencv_apps.msg import CircleArrayStamped
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo

BALL_SIZE = 0.0275 # meters (example size)
camera = PinholeCameraModel()
pub = rospy.Publisher('/balls_location',Point)

def circle_callback(array):
	info = rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo)
	camera.fromCameraInfo(info)
	fx = info.K[0]
	circles = array.circles
	points = []
	for circle in circles:
		print(circle)
		ball_size_pixels = circle.radius # pixels (example size)
		distance = BALL_SIZE * fx / ball_size_pixels
		point = camera.projectPixelTo3dRay((circle.center.x,circle.center.y))
		print('before converstion')
		print(point)
		point = tuple(distance * axis for axis in point)
		print('before transform')
		print(point)
		dpoint = Point(x=point[0],y=point[2],z=point[1])
    # Transform the point from camera to base frame
		tf_buffer = tf2_ros.Buffer()
		tf_listener = tf2_ros.TransformListener(tf_buffer)
		transform = tf_buffer.lookup_transform('base_link', info.header.frame_id, rospy.Time())
		translation = np.array([transform.transform.translation.x,
								transform.transform.translation.y,
								transform.transform.translation.z])
		rotation = np.array([transform.transform.rotation.x,
							transform.transform.rotation.y,
							transform.transform.rotation.z,
							transform.transform.rotation.w])
		R = quaternion_matrix(rotation)[:3,:3]
		point_base = np.dot(R, point) + translation
	# Create a PointStamped message for the 3D world point
		world_point = PointStamped()
		world_point.header.frame_id = 'world'
		world_point.header.stamp = rospy.Time.now()
		world_point.point.x = point_base[0]
		world_point.point.y = point_base[1]
		world_point.point.z = point_base[2]
		print(world_point)

def quaternion_matrix(quaternion):
    q = np.array(quaternion[:4], dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < 0.0001:
        return np.identity(4)
    q *= np.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[2, 2]-q[3, 3], q[1, 2]-q[3, 0], q[1, 3]+q[2, 0], 0.0],
        [q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3], q[2, 3]-q[1, 0], 0.0],
        [q[1, 3]-q[2, 0], q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [0.0, 0.0, 0.0, 1.0]])


def main():
	rospy.init_node('test_node')

	sub_ball = rospy.Subscriber('/hough_circles/circles',CircleArrayStamped, circle_callback)
	rospy.spin()

if __name__=='__main__':
	main()


