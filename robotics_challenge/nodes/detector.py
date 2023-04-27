#!/usr/bin/env python3
import rospy
import cv2
import tf2_ros
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import Point,Pose
from tf2_geometry_msgs import PoseStamped
from image_geometry import PinholeCameraModel

BALL_SIZE = 0.0275
# Initialize the ROS node
rospy.init_node("ball_detection_node")
#rospy.wait_for_service("spawn")
print('-------------------------------DETECTOR IS UP-------------------------------')
# Initialize the CvBridge object
bridge = CvBridge()

#Setup PinholeCamera model
camera_info = rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo)
camera = PinholeCameraModel()
camera.fromCameraInfo(camera_info)

#Frame transformation
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

ball_pub = rospy.Publisher('/balls_location', PoseStamped, queue_size=1)
ball_blob_pub = rospy.Publisher('/balls_location/blob', Point, queue_size=1)


# Define the callback function to process the image data
def process_image(image_msg):
    # Convert the ROS image message to a numpy array
    img = bridge.imgmsg_to_cv2(image_msg, "bgr8")

    # Convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    lower_red = np.array([160,140,50])
    upper_red = np.array([180,255,255])

    
    # Apply a Gaussian blur to the grayscale image
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    #gray = cv2.medianBlur(gray,5)

    # Apply the Hough Circle algorithm
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param1=30, param2=30, minRadius=0, maxRadius=0)
    #circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2,100)   
    # If circles are detected, draw them on the original image and return their coordinates
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            cv2.circle(img, (x, y), r, (0, 255, 0), 2)
            print("Ball detected at ({}, {}) with radius {}".format(x, y, r))
            #if x >= 300 and x <= 325:
                #print("Exact location : ", find_3d_point(x,y,r))
                #ball_pub.publish(find_3d_point(x,y,r))
        circle = max(circles, key=lambda x: x[2])
        print(circle)
        ball_blob_pub.publish(Point(circle[0],circle[1],circle[2]))
    else:
        ball_blob_pub.publish(Point(0,0,0))
        print("No balls detected.")

    # Display the resulting image
    cv2.imshow("Result", img)
    cv2.imshow("Maks", gray)
    cv2.waitKey(1)
def find_3d_point(px,py,ball_radius):
        #Preparing variables
        fx = camera_info.K[0]
        fy = camera_info.K[4]
        radius = ball_radius
        ball_size_pixels = radius # pixels (example size)
        distance_x = BALL_SIZE * fx / ball_size_pixels
        distance_y = BALL_SIZE * fy / ball_size_pixels

        #Convert to 3D ray
        point = camera.projectPixelTo3dRay((px,py))

        #Convert to 3D Point
        dpoint = Point(point[0]*distance_x,point[1],point[2]*distance_y)

        #Preparing variables for frame transform
        pose = Pose()
        pose.position = dpoint
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'camera_rgb_optical_frame'
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = pose

        #Preforming transform
        try:
            output_pose_stamped = tf_buffer.transform(pose_stamped, 'map', rospy.Duration(1))
            return output_pose_stamped
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('Failed !')
# Subscribe to the camera topic
rospy.Subscriber("/camera/rgb/image_raw", Image, process_image)
#rospy.Subscriber("dark")

# Spin the ROS node
rospy.spin()