#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from operator import attrgetter
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from opencv_apps.msg import Circle, CircleArrayStamped
from std_msgs.msg import Int32, Float32



class TurtleBot():
    def __init__(self):
        rospy.init_node('turtlebot_controller')
        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.process_image)
        self.hough_circle_sub= rospy.Subscriber('/hough_circles/circles',CircleArrayStamped, self.circle_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.camera_info = rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo)
        self.rate = rospy.Rate(10)
        self.BALL_RAD = 0.0275
        self.ball_x = None
        self.ball_y = None
        self.ball_radius = None
        self.state = 'search'
    
    def circle_callback(self, array):
        circles = array.circles
        print(circles)
        if circles:
            circle = max(circles, key=attrgetter('radius'))
            self.ball_x = circle.center.x
            self.ball_y = circle.center.y
            self.ball_radius = circle.radius
            self.ball_distance = self.calculate_distance()

    def calculate_distance(self):
        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        return (fx * self.BALL_RAD / self.ball_radius)
    '''
    def process_image(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_color = np.array([0, 100, 100])
        upper_color = np.array([20, 255, 255])
        mask = cv2.inRange(hsv_image, lower_color, upper_color)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.imshow('hsv image',hsv_image)
        print("ball positions :")
        if len(contours) > 0:
            contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]
            max_contour = max(contour_sizes, key=lambda x: x[0])[1]
            (x, y), radius = cv2.minEnclosingCircle(max_contour)
            self.ball_x = int(x)
            self.ball_y = int(y)
            self.ball_radius = int(radius)
            print("X: %, y: %",self.ball_x,self.ball_y)
    '''

    def spin(self):
        twist = Twist()
        twist.angular.z = 0.5
        self.cmd_pub.publish(twist)

    def stop(self):
        twist = Twist()
        twist.angular.z = 0
        twist.linear.x = 0
        self.cmd_pub.publish(twist)

    def move_forward(self, speed):
        twist = Twist()
        twist.linear.x = speed
        self.cmd_pub.publish(twist)

    def distance_callback(self, data):
        self.ball_distance = data.data

    def turn_towards_ball(self):
        if self.ball_x is not None:
            if self.ball_x < 320:
                self.turn(0.1)
            elif self.ball_x > 320:
                self.turn(-0.1)

    def turn(self, speed):
        twist = Twist()
        twist.angular.z = speed
        self.cmd_pub.publish(twist)

    def pick_ball(self):
        twist = Twist()
        twist.linear.x = -0.1
        self.cmd_pub.publish(twist)
        rospy.sleep(2)
        twist.linear.x = 0
        self.cmd_pub.publish(twist)

    def place_ball(self):
        twist = Twist()
        twist.linear.x = 0.2
        self.cmd_pub.publish(twist)
        rospy.sleep(2)
        twist.linear.x = 0
        self.cmd_pub.publish(twist)

    def run(self):
        while not rospy.is_shutdown():
            if self.state == 'search':
                print('Search State')
                self.spin()
                if self.ball_x is not None:
                    self.state = 'approach'
            elif self.state == 'approach':
                print('Approach State')
                if self.ball_radius < 80:
                    print('Moving to ball')
                    self.turn_towards_ball()
                    self.stop()
                    #rospy.sleep(0.5)
                    self.move_forward(0.1)
                    self.stop()
                if self.ball_radius > 80:
                    #self.state = 'pickup'
                    self.stop()
        self.rate.sleep()
if __name__ == '__main__':
    try:
        turtlebot = TurtleBot()
        turtlebot.run()
    except rospy.ROSInterruptException:
        pass