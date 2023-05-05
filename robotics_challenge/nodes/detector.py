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
LINE_X_POSITION = 0.02 #To deal with the inacurracy of the coordinates retrieved from camera to 3d point


class BallDetector:

    def __init__(self):
        print('-------------------------------DETECTOR IS UP-------------------------------')
        # Initialize the CvBridge object
        self.bridge = CvBridge()

        #Odom
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.robot_pos_callback)
        self.robot_pos = Pose()


        #Setup PinholeCamera model/ Image topic
        self.sub = rospy.Subscriber('/camera/image', Image, self.process_image)
        self.camera_info = rospy.wait_for_message('/camera/camera_info', CameraInfo)
        self.camera = PinholeCameraModel()
        self.camera.fromCameraInfo(self.camera_info)
        print(' Camera Connected Succesfully')

        #Frame transformation
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.ball_pub = rospy.Publisher('/balls_location', Point, queue_size=1)
        self.ball_blob_pub = rospy.Publisher('/balls_location/blob', SnapshotBalls, queue_size=1)
        self.output_mask_pub=rospy.Publisher('/output_mask', Image, queue_size=3)


    #Odom Subscriber callback
    def robot_pos_callback(self, msg):
        self.robot_pos = msg.pose.pose


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
        
    def detect_ball(self,img):
        # Convert the image to grayscale
        #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = img[:,:,0]

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

        ANY_low_H=50
        ANY_low_S=102
        ANY_low_V=0
        ANY_high_H=179
        ANY_high_S=255
        ANY_high_V=255

        #final_contours,mask_frame = self.find_contours(hsv_frame, RED_low_H,RED_low_S,RED_low_V,RED_high_H,RED_high_S,RED_high_V)
        final_contours,mask_frame = self.find_contours(hsv_frame, ANY_low_H,ANY_low_S,ANY_low_V,ANY_high_H,ANY_high_S,ANY_high_V)


        for contour in final_contours:
            x, y, w, h = cv2.boundingRect(contour)
            dpoint = self.find_3d_point(x+(w/2),y+(h/2), w/2)
            #if 315 <= (x+(w/2)) <= 325 and dpoint.x < LINE_X_POSITION :
            print(x+(w/2),y+(h/2), len(contour)/2)
            print(dpoint)
            img = cv2.rectangle(img, (x, y),(x + w, y + h),(0, 0, 255), 2)
            #self.ball_pub.publish(dpoint)
        '''
        mask_frame=cv2.inRange(hsv_frame, (low_H, low_S, low_V), (high_H, high_S, high_V))

        contours, hierarchy = cv2.findContours(mask_frame,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if len(contour) >= 5:
                ellipse = cv2.fitEllipse(contour)
                (x, y), (major_axis, minor_axis), angle = ellipse

                # Calculate the aspect ratio of the ellipse
                aspect_ratio = major_axis / minor_axis

                # Define a threshold value for the aspect ratio to determine whether the ellipse is a circle
                threshold = 1

                # If the aspect ratio is close to 1.0, the ellipse is a circle
                if abs(aspect_ratio - 1.0) < threshold:
                    print(ellipse)
                    # Process the detected circle and publish the results
                    x, y, w, h = cv2.boundingRect(contour)
                    img = cv2.rectangle(img, (x, y),(x + w, y + h),(0, 0, 255), 2)
        '''        

        #cv2.imshow("mask",mask_frame)
        #cv2.imshow("image", img)

        output_mask = self.bridge.cv2_to_imgmsg(mask_frame, "passthrough")
        self.output_mask_pub.publish(output_mask)


        cv2.waitKey(1)
        return mask_frame
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
        cv2.imshow("mask",mask_frame)
        return final_contours, mask_frame

    def gpt(self, image):

        ANY_low_H=50
        ANY_low_S=102
        ANY_low_V=0
        ANY_high_H=179
        ANY_high_S=255
        ANY_high_V=255

        # Define threshold values for circularity check
        aspect_ratio_thresh = 0.9
        min_contour_area = 10

        # Define minimum distance between circle centers to consider them as part of the same cluster
        cluster_distance_thresh = 20

        # Convert the input image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply a mask to the grayscale image using the HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (ANY_low_H, ANY_low_S, ANY_low_V), (ANY_high_H, ANY_high_S, ANY_high_V))
        gray_masked = cv2.bitwise_and(gray, gray, mask=mask)

        # Find contours in the masked grayscale image
        contours, _ = cv2.findContours(gray_masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Group circular contours into clusters
        circle_clusters = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_contour_area:
                continue
            (x, y), radius = cv2.minEnclosingCircle(contour)
            aspect_ratio = min(radius / area, area / radius)
            if aspect_ratio > aspect_ratio_thresh:
                # Check if the contour belongs to an existing cluster
                added_to_cluster = False
                for cluster in circle_clusters:
                    if np.sqrt((x - cluster[0][0])**2 + (y - cluster[0][1])**2) < cluster_distance_thresh:
                        cluster.append(contour)
                        added_to_cluster = True
                        break
                if not added_to_cluster:
                    circle_clusters.append([[(x, y), radius, contour]])

        # Draw circles around the detected ball clusters
        for cluster in circle_clusters:
            x_sum = y_sum = r_sum = 0
            for circle in cluster:
                x_sum += circle[0][0]
                y_sum += circle[0][1]
                r_sum += circle[1]
            x_avg = int(x_sum / len(cluster))
            y_avg = int(y_sum / len(cluster))
            r_avg = int(r_sum / len(cluster))
            cv2.circle(image, (x_avg, y_avg), r_avg, (0, 255, 0), 2)

        # Display the resulting image
        cv2.imshow("Image", image)
        cv2.waitKey(0)


def main():
    # Initialize the ROS node
    rospy.init_node("ball_detection_node")
    #rospy.wait_for_service("spawn")
    bDetector = BallDetector()

    rospy.spin()

if __name__=='__main__':
    main()