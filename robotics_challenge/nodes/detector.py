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
from nav_msgs.msg import Odometry
from robotics_challenge.msg import Ball, SnapshotBalls

#Constants
BALL_SIZE = 0.0275
LINE_X_POSITION = -0.1 #To deal with the inacurracy of the coordinates retrieved from camera to 3d point


class BallDetector:

    def __init__(self):
        print('-------------------------------DETECTOR IS UP-------------------------------')
        # Initialize the CvBridge object
        self.bridge = CvBridge()

        #Odom
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.robot_pos_callback)
        self.robot_pos = Pose()


        #Setup PinholeCamera model/ Image topic
        self.sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.process_image)
        self.camera_info = rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo)
        self.camera = PinholeCameraModel()
        self.camera.fromCameraInfo(self.camera_info)

        #Frame transformation
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.ball_pub = rospy.Publisher('/balls_location', PoseStamped, queue_size=1)
        self.ball_blob_pub = rospy.Publisher('/balls_location/blob', SnapshotBalls, queue_size=1)

    #Odom Subscriber callback
    def robot_pos_callback(self, msg):
        self.robot_pos = msg.pose.pose


    # Define the callback function to process the image data
    def process_image(self, image_msg):
        # Convert the ROS image message to a numpy array
        img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        self.detect_ball(img)
        self.detect_close_ball(img)
        
    def detect_close_ball(self,img):
        #TODO
        return
        
    def detect_ball(self,img):
        # Convert the image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Apply a Gaussian blur to the grayscale image
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply the Hough Circle algorithm
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=100)

        # If circles are detected, draw them on the original image and return their coordinates
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            balls = SnapshotBalls()

            #Converting to Ball objects & drawing balls
            for (x, y, r) in circles:
                cv2.circle(img, (x, y), r, (0, 255, 0), 2)
                ball = Ball(position3d=self.find_3d_point(x,y,r),positionPixel=Point(x,y,r))

                #If ball on our side, add it to balls snapshot
                if ball.position3d.x < LINE_X_POSITION :
                    balls.list.append(ball)

            #Handle if no balls where found on our side
            if balls.list :
            #Rearranging nearest ball (target ball) to the front
                nearest_ball = max(balls.list, key=lambda ball : ball.positionPixel.z)
                balls.list.insert(0, balls.list.pop(balls.list.index(nearest_ball)))

            print(balls)
            self.ball_blob_pub.publish(balls)
        #Incase there is no balls, we still send an empty array
        else:
            self.ball_blob_pub.publish(SnapshotBalls())
            print("No balls detected.")

        # Display the resulting image
        cv2.imshow("Result", img)
        cv2.waitKey(1)

    def find_3d_point(self, px,py,ball_radius):
            #Preparing variables
            fx = self.camera_info.K[0]
            fy = self.camera_info.K[4]
            radius = ball_radius
            ball_size_pixels = radius # pixels (example size)
            distance_x = BALL_SIZE * fx / ball_size_pixels
            distance_y = BALL_SIZE * fy / ball_size_pixels

            #Convert to 3D ray
            point = self.camera.projectPixelTo3dRay((px,py))

            #Convert to 3D Point
            dpoint = Point(point[0]*distance_x/point[2],point[1]/point[2],1*distance_y)

            #Preparing variables for frame transform
            pose = Pose()
            pose.position = dpoint
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'camera_rgb_optical_frame'
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose = pose

            #Preforming transform
            try:
                output_pose_stamped = self.tf_buffer.transform(pose_stamped, 'map', rospy.Duration(1))
                return output_pose_stamped.pose.position
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print('Failed !')


def main():
    # Initialize the ROS node
    rospy.init_node("ball_detection_node")
    #rospy.wait_for_service("spawn")
    bDetector = BallDetector()

    rospy.spin()

if __name__=='__main__':
    main()