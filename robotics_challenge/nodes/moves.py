#!/usr/bin/env python3

import rospy
import actionlib
import math
import moveit_commander
import moveit_msgs.msg
from robotics_challenge.msg import Ball
from geometry_msgs.msg import Pose, Point, PolygonStamped, Point32
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseGoal, MoveBaseAction


OPEN = 0.9
CLOSE = 0

class ArmController:
    
    def __init__(self):
        moveit_commander.roscpp_initialize([])
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.robot = moveit_commander.RobotCommander('robot_description')
        self.gripper = self.robot.get_joint('gripper')

    def set_pos(self, position):
        self.arm.set_position_target(position)
        self.arm.go(wait=True)
    def set_pos_name(self, name):
        self.arm.set_named_target(name)
        self.arm.go(wait=True)
    def open_gripper(self):
        self.gripper.move(self.gripper.max_bound() * OPEN, True)

    def close_gripper(self):
        self.gripper.move(self.gripper.max_bound() * CLOSE, True)

    def pick_ball(self, position):
        position_above_ball = [position[0], position[1], position[2] + 0.1]
        self.set_pos(position_above_ball)
        self.open_gripper()
        position_below_ball = [position[0] - 0.005, position[1], position[2]]
        self.set_pos(position_below_ball)
        #position_below_ball = [position[0] - 0.01, position[1], position[2]]
        #self.set_pos(position_below_ball)
        self.close_gripper()
        self.set_pos_name('home')
    def place_ball(self, position):
        self.set_pos(position)
        self.open_gripper()
        self.set_pos_name('home')
        

class MovementManager:

	def __init__(self):
		
		self.move_base = actionlib.SimpleActionClient('move_base', 	MoveBaseAction)
		self.move_base.wait_for_server(rospy.Duration(5.0))
		self.finished = False
		rospy.logdebug("move_base is ready")
		
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback)
		self.robot_pos = Pose()
		self.arm_controller = ArmController()
	def callback(self, msg):
		self.robot_pos = msg.pose.pose
	def move_to_target(self, pos):
		# Create goal:
		goal = MoveBaseGoal()
		
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = pos
		
		self.move_base.send_goal(goal, self.goal_status)
		
	def pickup_routine(self, pose):
		pose.position.x +=0.2
		pose.orientation.z=1
		self.move_to_target(pose)
		while not self.finished:
			print('still moving!')
		self.finished =False
		self.arm_controller.pick_ball([0.23, 0, 0.029])
	def delivery_routine(self, pose):
		pose.position.x = 0.0
		pose.orientation.z=0
		pose.orientation.w=1
		self.move_to_target(pose)
		self.finished =False
		while not self.finished:
			print('still moving!')
		self.finished =False
		rospy.sleep(1)
		self.arm_controller.place_ball([0.2, 0, 0.0275])
	def goal_status(self, status, result):
		""" Check the status of a goal - goal reached, aborted,
		or rejected.
		"""
		self.finished = True
	def create_obstacle_msg(self,x, y, radius):
		obstacle = ObstacleMsg()
		obstacle.polygon.points = [Point32(x=x+radius, y=y)]
		angle_step = 2 * math.pi / 16
		for i in range(16):
			angle = i * angle_step
			obstacle.polygon.points.append(Point32(x=x+radius*math.cos(angle), 				y=y+radius*math.sin(angle)))
		return obstacle
	def set_obstacle(self, balls):

		pub = rospy.Publisher('/obstacle_array', ObstacleArrayMsg, queue_size=10)
		obstacles = ObstacleArrayMsg()
		for ball in balls:
			obstacle = self.create_obstacle_msg(ball[0],ball[1],ball[2])
			obstacles.obstacles.append(obstacle)
		rospy.sleep(1)
		pub.publish(obstacles)


def main():

	rospy.init_node('movement_manager_node')
	
	mManager = MovementManager()
	

	ball2 = Ball(position= Pose(position= Point(x=-1.5,y=0.7,z=0)))
	ball1 = Ball(position= Pose(position= Point(x=-1.5,y=-0.7,z=0)))
	ball0 = Ball(position= Pose(position= Point(x=-1.5,y=0,z=0)))
	balls = [ball0,ball1,ball2]
	balls_obs = [[-1.5, 0, 0.0275], [-1.5, -0.7, 0.0275], [-1.5, 0.7, 0.0275]]
	'''
	while not mManager.robot_pos:
		print('didnot recieve position')
	
	result = mManager.orient_robot(ball.position)
	
	print(result)
	
	rospy.spin()
	'''
	mManager.set_obstacle(balls_obs)
	for ball in balls:
		mManager.pickup_routine(ball.position)
		rospy.sleep(1)
		mManager.delivery_routine(ball.position)
	

if __name__=='__main__':
	main()



