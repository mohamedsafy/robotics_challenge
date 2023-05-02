#!/usr/bin/env python3

import math
import rospy
from robotics_challenge.msg import Ball
from geometry_msgs.msg import Twist, Point, Pose
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from robotics_challenge.msg import Ball, SnapshotBalls

ERROR_CODE = 1000

class BallCollector():
    def __init__(self):

        #local Fields
        self.target_ball = Ball()
        self.balls = []
        self.close_ball = Ball()
        
        #Publishers & Subscribers
        self.ball_sub = rospy.Subscriber('/balls_location/blob', SnapshotBalls, self.ball_callback)

        #TODO: Implement close ball 

    def ball_callback(self, msg):
        if msg.list:
            self.target_ball = msg.list.pop(0)
            self.balls = msg.list[:]
        else : 
            self.target_ball = None
            self.balls = []

class Approacher():
    def __init__(self):
        
        print('-------------------------------APPROACHER IS UP-------------------------------')
        self.rate = rospy.Rate(10)
        
        #Robot movement publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        #State machine specifics
        self.state = 'search' #not used
        self.finished_moving = False #not used
        self.collector = BallCollector()

        #Odom
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback)
        self.robot_pos = Pose()


    def callback(self, msg):
        self.robot_pos = msg.pose.pose

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

    def turn(self, speed):
        twist = Twist()
        twist.angular.z = speed
        self.cmd_pub.publish(twist)

    def take_action(self):
        if self.collector.target_ball is not None and self.collector.target_ball.positionPixel.x != 0:
        
            ball_x = self.collector.target_ball.positionPixel.x
            ball_radius = self.collector.target_ball.positionPixel.z

            print("moving to ball : {},{}".format(ball_x, ball_radius))

            correction_x = 320 - ball_x
            if -5 <= correction_x <= 5:
                angular_vel = 0
            else:
                angular_vel = correction_x/1000

            correction_radius = (75 - ball_radius)*0.5
            if  correction_radius <= 5 :
                linear_vel = 0

            else :
                linear_vel = correction_radius/100

            print("Speed : {},{}".format(linear_vel, angular_vel))
            return (linear_vel, angular_vel)
        else:
            return (ERROR_CODE,ERROR_CODE) 
    def start(self):
        stop_spinning = False

        #Implementation for the search and approach state. the rest of the states are scattered across the code
        while not rospy.is_shutdown() :
            if self.collector.target_ball is not None and self.collector.target_ball.positionPixel.x != 0:
                self.stop()
                linear_vel, angular_vel = self.take_action()
                linear_vel = min(0.1, linear_vel)
                #angular_vel = max(0.2, min(angular_vel, 0.5))
                if linear_vel == 0 and angular_vel ==0:
                    self.stop()
                    break
                if linear_vel != ERROR_CODE and angular_vel != ERROR_CODE:
                    twist = Twist()
                    twist.angular.z = angular_vel
                    twist.linear.x = linear_vel
                    self.cmd_pub.publish(twist)
            else:
                self.spin()
                rospy.sleep(0.05)
                print('Searching for balls')
                self.stop()
    def go_to_line(self):

        #Face the line
        while not (0.98 <= abs(self.robot_pos.orientation.w) <= 1) :
            print(self.robot_pos.orientation)
            self.spin()
        self.stop()

        #move till line is reached
        while self.robot_pos.position.x < -0.15:
            speed = abs(self.robot_pos.position.x)
            speed = max(0.2, min(speed, 5.0))
            self.move_forward(speed)
            
        self.stop()

    def move_distance(self, target, speed):
        origin = self.robot_pos.position
        distance = math.dist([self.robot_pos.position.x,self.robot_pos.position.y],[origin.x, origin.y])
        while distance < target:
            distance = math.dist([self.robot_pos.position.x,self.robot_pos.position.y],[origin.x, origin.y])
            self.move_forward(speed)
            #print(distance)
        self.stop()
        
def main():
    #Ros specifics
    rospy.init_node('ball_approacher_node')

    approacher = Approacher()
    

    #All of this needs to be contained in a controlled state machine rather than this step by step code
    while not rospy.is_shutdown():

        #search and approach state
        approacher.start()
        
        
        approacher.move_distance(0.1, 0.01)
        print('done')
        
        #pickup state
        rospy.wait_for_service('ball_pickup')
        try:
            rospy.ServiceProxy('ball_pickup', Empty)()
            print('reached')
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
        #making space for the robot to spin
        approacher.move_distance(-0.1, 0.05)


        #Atempt to make the robot able to detect if the ball is picked 
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
        
        #delivery state
        approacher.go_to_line()

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