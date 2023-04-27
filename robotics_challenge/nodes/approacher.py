#!/usr/bin/env python3

import math
import rospy
import tf2_ros
import actionlib
from geometry_msgs.msg import Twist, Point, Pose
from tf2_geometry_msgs import PoseStamped
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseGoal, MoveBaseAction

BALL_SIZE = 0.0275

class Approacher():
    def __init__(self):
        #Ros specifics
        rospy.init_node('ball_approacher_node')
        #rospy.wait_for_service("spawn")
        print('-------------------------------APPROACHER IS UP-------------------------------')
        self.rate = rospy.Rate(10)

        print('getting camera info')
        #Setup PinholeCamera model
        self.camera_info = rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo)
        self.camera = PinholeCameraModel()
        self.camera.fromCameraInfo(self.camera_info)
        print('Done')
        
        #Robot movement publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        #Ball position reciever
        self.ball_sub = rospy.Subscriber('/balls_location/blob', Point, self.ball_callback)

        #State machine specifics
        
        self.BALL_RAD = 0.0275
        self.ball_x = None
        self.ball_y = None
        self.ball_radius = None
        self.state = 'search'
        self.finished_moving = False

        print('setting transform')
        #Frame transformation
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        print('Done')
        #Odom
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback)
        self.robot_pos = Pose()

        #Move base
        self.move_base = actionlib.SimpleActionClient('move_base', 	MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5.0))

    def callback(self, msg):
        self.robot_pos = msg.pose.pose

    #Check if ball is on the robot side
    def on_our_side(self):
        print('Is ball on our side ?')
        #Convert to 3D point
        dpoint = self.find_3d_point()

        if not dpoint == None and dpoint.pose.position.x < -0.1 :
             print(dpoint)
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
        ball_size_pixels = self.ball_radius # pixels (example size)
        distance_x = BALL_SIZE * fx / ball_size_pixels
        distance_y = BALL_SIZE * fy / ball_size_pixels

        #Convert to 3D ray
        point = self.camera.projectPixelTo3dRay((px,py))

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
            output_pose_stamped = self.tf_buffer.transform(pose_stamped, 'map', rospy.Duration(1))
            return output_pose_stamped
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('Failed !')

    def ball_callback(self, ball):
        #if self.ball_x == 0 :
        #    self.ball_x = None
        #    self.ball_y = None
        #    self.ball_radius = None
        #    return
        self.ball_x = ball.x
        self.ball_y = ball.y
        self.ball_radius = ball.z

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

    def turn_towards_ball(self):
        while self.ball_x != 320:
            if self.ball_x is not None:
                if self.ball_x < 320:
                    self.turn(0.005)
                elif self.ball_x > 320:
                    self.turn(-0.005)

    def turn(self, speed):
        twist = Twist()
        twist.angular.z = speed
        self.cmd_pub.publish(twist)

    def take_action(self):
        print("moving to ball : {},{}".format(self.ball_x, self.ball_radius))

        correction_x = 320 - self.ball_x
        if abs(correction_x) <= 5:
            angular_vel = 0
        else:
            angular_vel = correction_x/1000

        correction_radius = (70 - self.ball_radius)*0.5
        if  correction_radius <= 5 :
            linear_vel = 0

        else :
            linear_vel = correction_radius/100

        print("Speed : {},{}".format(linear_vel, angular_vel))
        return (linear_vel, angular_vel)
    def start(self):
        stop_spinning = False
        while not rospy.is_shutdown() :
            if self.ball_x is not None and self.ball_x != 0 and self.on_our_side():
                self.stop()
                linear_vel, angular_vel = self.take_action()
                linear_vel = min(0.2, linear_vel)
                #angular_vel = max(0.2, min(angular_vel, 0.5))
                if linear_vel == 0 and angular_vel ==0:
                    self.stop()
                    break
                twist = Twist()
                twist.angular.z = angular_vel
                twist.linear.x = linear_vel
                print("Speed : {},{}".format(linear_vel,angular_vel))
                self.cmd_pub.publish(twist)
            else:
                self.spin()
                rospy.sleep(0.05)
                print('done')
                self.stop()
    def go_to_line(self):

        #Face the line
        while not (0.99 <= abs(self.robot_pos.orientation.w) <= 1) :
            print(self.robot_pos.orientation)
            self.spin()
        self.stop()

        

        while self.robot_pos.position.x < -0.1:
            speed = abs(self.robot_pos.position.x)
            speed = max(0.2, min(speed, 5.0))
            self.move_forward(speed)
            
        self.stop()

    def goal_status(self, status, result):
        self.finished_moving = True
    def move_distance(self, target, speed):
        origin = self.robot_pos.position
        distance = math.dist([self.robot_pos.position.x,self.robot_pos.position.y],[origin.x, origin.y])
        while distance < target:
            distance = math.dist([self.robot_pos.position.x,self.robot_pos.position.y],[origin.x, origin.y])
            self.move_forward(speed)
            #print(distance)
        self.stop()
    
    def wait_finishing_movement(self):
        while not self.finished_moving:
            print('Still moving')
        self.finished_moving = False
        
def main():
    approacher = Approacher()
    
    while not rospy.is_shutdown():
        approacher.start()
        
        approacher.move_distance(0.11, 0.01)
        print('done')
        
        rospy.wait_for_service('ball_pickup')
        try:
            rospy.ServiceProxy('ball_pickup', Empty)()
            print('reached')
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        #input('Did you pick the ball ?')
        approacher.move_distance(-0.1, 0.05)

        '''
        while approacher.ball_x !=0 and approacher.ball_x is not None and approacher.on_our_side():
            approacher.start()

            rospy.wait_for_service('ball_pickup')
            try:
                rospy.ServiceProxy('ball_pickup', Empty)()
                print('reached')
                
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
            #input('Did you pick the ball ?')
            approacher.move_distance(-0.1, 0.05)
        '''
        
        approacher.go_to_line()
        #rospy.spin()
        
        #approacher.wait_finishing_movement()

        rospy.wait_for_service('ball_drop')
        try:
            rospy.ServiceProxy('ball_drop', Empty)()
            print('reached')
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
        rospy.sleep(1)

        approacher.move_distance(0.2, -0.1)

        rospy.sleep(1)
    rospy.spin()

    
    
if __name__ == '__main__':
    main()