#!/usr/bin/env python3

import rospy
import math
import numpy as np
import tf2_ros
from geometry_msgs.msg import Point, Pose
from opencv_apps.msg import CircleArrayStamped
from opencv_apps.msg import Circle
from image_geometry import PinholeCameraModel
import tf2_geometry_msgs 
from sensor_msgs.msg import CameraInfo

BALL_SIZE = 0.0275 # meters (example size)


rospy.init_node('test_node')

camera = PinholeCameraModel()
pub = rospy.Publisher('/balls_location',Point)
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)


def circle_callback(array):
	info = rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo)
	camera.fromCameraInfo(info)
	fx = info.K[0]
	fy = info.K[4]

	circles = array.circles
	for circle in circles:
		if circle.center.x < 325 and circle.center.x > 300 :
			print(circle)
			print("focal lenght 0.2f% , 0.2f%",fx,fy)
			
			ball_size_pixels = circle.radius # pixels (example size)
			distance_x = BALL_SIZE * fx / ball_size_pixels
			distance_y = BALL_SIZE * fy / ball_size_pixels
			print("distance % , %", distance_x,distance_y)
			point = camera.projectPixelTo3dRay((circle.center.x,circle.center.y))
			print('before converstion')
			print(point)
			#point = tuple(distance * axis for axis in point)
			point = [point[0]*distance_x,1*distance_y,-point[1]]
			print('before transform')
			print(point)
			dpoint = Point(x=point[0],y=point[1],z=point[2])
			pose = Pose()
			pose.position = dpoint
			pose_stamped = tf2_geometry_msgs.PoseStamped()
			pose_stamped.header.frame_id = 'camera_link'
			pose_stamped.header.stamp = rospy.Time.now()
			pose_stamped.pose = pose

			try:
				output_pose_stamped = tf_buffer.transform(pose_stamped, 'map', rospy.Duration(1))
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				print('Failed !')
			print(output_pose_stamped)

	

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
def transform_pose(input_pose, from_frame, to_frame):

    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time.now()

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

def main():


	sub_ball = rospy.Subscriber('/hough_circles/circles',CircleArrayStamped, circle_callback)
	circle = Circle()
	circle.center.x = 323
	circle.center.y = 281
	circle.radius = 17
    
	#circle_callback(circle)

	rospy.spin()

if __name__=='__main__':
	main()


