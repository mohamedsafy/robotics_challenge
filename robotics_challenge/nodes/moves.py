#!/usr/bin/env python3

import rospy
import actionlib
import math
import cv2
import moveit_commander
import moveit_msgs.msg
import tf2_ros
from tf2_geometry_msgs import PoseStamped
from cv_bridge import CvBridge
from operator import attrgetter
from robotics_challenge.msg import Ball
from geometry_msgs.msg import Pose, Point, PolygonStamped, Point32, Twist
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from opencv_apps.msg import Circle, CircleArrayStamped
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseGoal, MoveBaseAction

OPEN = 0.9
CLOSE = 0
BALL_SIZE = 0.0275

class Approacher():
	def __init__(self):
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
		self.ball_sub = rospy.Subscriber('/balls_location', PoseStamped, self.ball_callback)

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
		self.ball_x = ball.pose.position.x
		self.ball_y = ball.pose.position.y
		self.ball_radius = ball.pose.position.z

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
			if self.ball_radius < 80:
				print('Moving to ball')
				self.turn_towards_ball()
				self.turn(0)
				#self.stop()
				#rospy.sleep(0.5)
				self.move_forward(0.1)
				self.stop()
			if self.ball_radius > 80:
				self.state = 'pickup'
				self.stop()
				return
			#elif self.state == 'getback':
			#     self.move_forward(-0.2)
			#     self.stop()
				
		self.rate.sleep()

class BallsCollector:
	def __init__(self):
		self.sub = rospy.Subscriber('/balls_location', PoseStamped, self.add_ball)
		self.balls = []
	def add_ball(self, pose):
		print(pose)
		self.balls.append(pose)

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

		#Robot movement publisher
		self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback)
		self.robot_pos = Pose()
		self.arm_controller = ArmController()
		self.appr = Approacher()
	def callback(self, msg):
		self.robot_pos = msg.pose.pose
	def move_to_target(self, pos):
		# Create goal:
		goal = MoveBaseGoal()
		
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = pos
		
		self.move_base.send_goal(goal, self.goal_status)
		
	def pickup_routine(self,pose):
		pose.position.x +=0.2
		pose.position.z =0
		self.move_to_target(pose)
		while not self.finished:
			print('still moving!')
		self.finished= False
		rospy.sleep(1)
		#self.appr.run()
		self.arm_controller.pick_ball([0.25, 0, 0.029])
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
	def spin(self, speed):
		twist = Twist()
		twist.angular.z = speed
		self.cmd_pub.publish(twist)
		
	def goal_status(self, status, result):

		self.finished = True
	def create_obstacle_msg(self,x, y, radius):
		obstacle = ObstacleMsg()
		obstacle.polygon.points = [Point32(x=x+radius, y=y)]
		angle_step = 2 * math.pi / 16
		for i in range(16):
			angle = i * angle_step
			obstacle.polygon.points.append(Point32(x=x+radius*math.cos(angle),y=y+radius*math.sin(angle)))
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
	bCollector = BallsCollector()

	while len(bCollector.balls) < 3 :
		print(len(bCollector.balls))
		mManager.spin(0.3)
	mManager.spin(0)

	for ball in bCollector.balls :
		mManager.pickup_routine(ball.pose)
		rospy.sleep(1)
		mManager.delivery_routine(ball.pose)

	rospy.spin()

if __name__=='__main__':
	main()



