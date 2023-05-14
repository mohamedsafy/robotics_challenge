#!/usr/bin/env python3
import rospy
import math
import cv2
import tf2_ros
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import Point,Pose
from tf2_geometry_msgs import PoseStamped
from image_geometry import PinholeCameraModel
from nav_msgs.msg import Odometry, OccupancyGrid
from robotics_challenge.msg import Ball, SnapshotBalls
from tf.transformations import euler_from_quaternion, quaternion_from_euler


#Constants
BALL_SIZE = 0.0275
LINE_X_POSITION = 0.02 #To deal with the inacurracy of the coordinates retrieved from camera to 3d point

CAMERA_TOPIC = '/camera/rgb/image_raw'
CAMERA_INFO_TOPIC = '/camera/rgb/camera_info'


class BallDetector:

    def __init__(self):
        print('-------------------------------DETECTOR IS UP-------------------------------')
        # Initialize the CvBridge object
        self.bridge = CvBridge()

        #Odom
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.robot_pos_callback)
        self.robot_pos = Pose()


        #Setup PinholeCamera model/ Image topic
        self.sub = rospy.Subscriber(CAMERA_TOPIC, Image, self.process_image)
        self.camera_info = rospy.wait_for_message(CAMERA_INFO_TOPIC, CameraInfo)
        self.camera = PinholeCameraModel()
        self.camera.fromCameraInfo(self.camera_info)
        print(' Camera Connected Succesfully')

        #Frame transformation
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.ball_pub = rospy.Publisher('/balls_location', Point, queue_size=1)
        self.ball_blob_pub = rospy.Publisher('/balls_location/blob', SnapshotBalls, queue_size=1)
        self.output_mask_pub=rospy.Publisher('/output_mask', Image, queue_size=3)

        #Map
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.map_pub = rospy.Publisher('/modified_map', OccupancyGrid, queue_size=1)

    #Odom Subscriber callback
    def robot_pos_callback(self, msg):
        self.robot_pos = msg.pose.pose
    def map_callback(self, map):
        self.map = map

        #self.update_map()

    # Define the callback function to process the image data
    def process_image(self, image_msg):
        # Convert the ROS image message to a numpy array
        img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        print('Image Received')


        #self.gpt(img)
        self.alternate_ball(img)
        #self.detect_ball(img)
        #self.detect_close_ball(img)
        
    def detect_close_ball(self,img):
        #TODO
        return

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
            output_pose_stamped = self.tf_buffer.transform(pose_stamped, 'planner', rospy.Duration(1))
            return output_pose_stamped.pose.position
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('Failed !')
    def alternate_ball(self, img):
        hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsv_frame = cv2.resize(hsv_frame,(640,480))
        img = cv2.resize(img,(640,480))

        RED_low_H=0
        RED_low_S=52
        RED_low_V=0
        RED_high_H=0
        RED_high_S=255
        RED_high_V=255

        ANY_low_H=3
        ANY_low_S=83
        ANY_low_V=103
        ANY_high_H=179
        ANY_high_S=255
        ANY_high_V=255

        final_contours = self.find_contours(hsv_frame, RED_low_H,RED_low_S,RED_low_V,RED_high_H,RED_high_S,RED_high_V)
        #final_contours = self.find_contours(hsv_frame, ANY_low_H,ANY_low_S,ANY_low_V,ANY_high_H,ANY_high_S,ANY_high_V)


        for contour in final_contours:
            x, y, w, h = cv2.boundingRect(contour)
            dpoint = self.find_3d_point(x+(w/2),y+(h/2), w/2)
            if 315 <= (x+(w/2)) <= 325 :#and dpoint.x < LINE_X_POSITION 

            #publish point
                print(x+(w/2),y+(h/2), len(contour)/2)
                print(dpoint)
                self.ball_pub.publish(dpoint)

            #draw boundaries
            img = cv2.rectangle(img, (x, y),(x + w, y + h),(0, 0, 255), 2)

            #update map
            #self.update_map(dpoint)
            

        #cv2.imshow("mask",mask_frame)
        #cv2.imshow("image", img)
        cv2.waitKey(1)
    def find_contours(self, hsv_frame,low_H,low_S,low_V,high_H,high_S,high_V):
        mask_frame=cv2.inRange(hsv_frame, (low_H, low_S, low_V), (high_H, high_S, high_V))
        contours, hierarchy = cv2.findContours(mask_frame,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        final_contours=[]
        for contour in contours:
            if len(contour) >= 5:
                ellipse = cv2.fitEllipse(contour)
                (x, y), (major_axis, minor_axis), angle = ellipse

                # Calculate the aspect ratio of the ellipse
                aspect_ratio = major_axis / minor_axis

                # Define a threshold value for the aspect ratio to determine whether the ellipse is a circle
                threshold = 0.3

                # If the aspect ratio is close to 1.0, the ellipse is a circle
                if abs(aspect_ratio - 1.0) < threshold:
                    #print(x,y,len(contour)/2)
                    # Process the detected circle and publish the results
                    final_contours.append(contour)

        #output_mask = self.bridge.cv2_to_imgmsg(mask_frame, "passthrough")
        #self.output_mask_pub.publish(output_mask)
        cv2.imshow("mask",mask_frame)
        return final_contours

    def update_map(self):
        ldata = list(self.map.data)

        x1,y1,x2,y2 = self.calculate_vision()
        print(x1,x2,y1,y2)
        print("updating")
        #draw line
        for x in np.arange(x1,x2+0.1,0.01):
            print('hey')
            for y in np.arange(y1,y2+0.01,0.01):
                index = int((y- -10) / self.map.info.resolution * self.map.info.width + (x- -10) / self.map.info.resolution)
                print(index)
                ldata[index] = 100


        self.map.data = tuple(ldata)
        self.map_pub.publish(self.map)
    def calculate_vision(self):
        #This part works only if we set our size to be negative
        
        orientation_list = [self.robot_pos.orientation.x,self.robot_pos.orientation.y,self.robot_pos.orientation.z,self.robot_pos.orientation.w]
        orientation = euler_from_quaternion(orientation_list)
        print(orientation[2])

        line_end_x = (self.robot_pos.position.x + 0.5) * math.cos(orientation[2])
        line_end_y = (self.robot_pos.position.y + 0.5) * math.sin(orientation[2])


        return self.robot_pos.position.x, self.robot_pos.position.y, line_end_x, line_end_y

def main():
    # Initialize the ROS node
    rospy.init_node("ball_detection_node")
    #rospy.wait_for_service("spawn")
    bDetector = BallDetector()

    rospy.spin()

if __name__=='__main__':
    main()