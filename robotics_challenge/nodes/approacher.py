#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import Twist, Point, Pose
from tf2_geometry_msgs import PoseStamped
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
from operator import attrgetter


BALL_SIZE = 0.0275

class Approacher():
    def __init__(self):
        #Ros specifics
        rospy.init_node('ball_approacher_node')
        #rospy.wait_for_service("spawn")
        print('-------------------------------APPROACHER IS UP-------------------------------')
        self.rate = rospy.Rate(10)

        #Setup PinholeCamera model
        self.camera_info = rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo)
        self.camera = PinholeCameraModel()
        self.camera.fromCameraInfo(self.camera_info)

        #Robot movement publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        #Ball position reciever
        self.ball_sub = rospy.Subscriber('/balls_location', Point, self.ball_callback)

        #State machine specifics
        
        self.BALL_RAD = 0.0275
        self.ball_x = None
        self.ball_y = None
        self.ball_radius = None
        self.state = 'search'

        #Frame transformation
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        

    #Check if ball is on the robot side
    def on_our_side(self,):

        #Convert to 3D point
        dpoint = self.find_3d_point()

        if not dpoint == None and dpoint.pose.position.x < -0.1 :
             print('point on our side')
             return True
        else:
             return False
        
    def find_3d_point(self):
        #Preparing variables
        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        px = self.ball_x
        py = self.ball_y
        radius = self.ball_radius
        ball_size_pixels = radius # pixels (example size)
        distance_x = BALL_SIZE * fx / ball_size_pixels
        distance_y = BALL_SIZE * fy / ball_size_pixels

        #Convert to 3D ray
        point = self.camera.projectPixelTo3dRay((px,py))

        #Convert to 3D Point
        dpoint = Point(point[0]*distance_x,1*distance_y,-point[1])

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
            return output_pose_stamped
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('Failed !')

    def ball_callback(self, ball):
        print(ball)
        self.ball_x = ball.x
        self.ball_y = ball.y
        self.ball_radius = ball.z

    def spin(self):
        twist = Twist()
        twist.angular.z = 0.3
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

    def run(self):
        while not rospy.is_shutdown():
            if self.state == 'search':
                print('Search State')
                self.spin()
                if self.ball_x is not None and self.on_our_side():
                    self.state = 'approach'
            elif self.state == 'approach':
                print('Approach State')
                if self.ball_radius < 80:
                    print('Moving to ball')
                    self.turn_towards_ball()
                    self.turn(0)
                    #self.stop()
                    #rospy.sleep(0.5)
                    self.move_forward(0.2)
                    self.stop()
                if self.ball_radius > 80:
                    self.state = 'pickup'
                    self.move_forward(0.2)
                    rospy.sleep(0.01)
                    self.move_forward(0.2)
                    rospy.sleep(0.01)
                    self.move_forward(0.2)
                    rospy.sleep(0.01)
                    self.move_forward(0.2)
                    rospy.sleep(0.01)
                    self.stop()
                    return
            #elif self.state == 'getback':
            #     self.move_forward(-0.2)
            #     self.stop()
                 
        self.rate.sleep()

def main():
    approacher = Approacher()

    approacher.run()

if __name__ == '__main__':
    main()