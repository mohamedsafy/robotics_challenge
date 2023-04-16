#!/usr/bin/env python3

import rospy
import actionlib
from robotics_challenge.msg import Ball
from geometry_msgs.msg import Pose, Point
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseGoal, MoveBaseAction
#x=-1.1, y=0

class MovementManager:

	def __init__(self, cManager):
	
		self.cManager = cManager
		
	def move_to_target(self, ball):
		# Create goal:
		goal = MoveBaseGoal()

		# Set random goal:
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = ball.position.position.x
		goal.target_pose.pose.position.y = ball.position.position.y
		goal.target_pose.pose.orientation.w = 1.0
		
		self.cManager.send_goal_move_base(goal)

class ConnectionManager:
	
	def __init__(self):
	
		self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.move_base.wait_for_server(rospy.Duration(5.0))
		rospy.logdebug("move_base is ready")
		
	def send_goal_move_base(self, goal):
		
		self.move_base.send_goal(goal, self.goal_status)
		
	
	def goal_status(self, status, result):
		""" Check the status of a goal - goal reached, aborted,
		or rejected.
		"""
		
		# Goal reached
		if status == 3:
			
			rospy.loginfo("Goal succeeded")

		# Goal aborted
		if status == 4:
			rospy.loginfo("Goal aborted")

		# Goal rejected
		if status == 5:
			rospy.loginfo("Goal rejected")
            
def main():

	rospy.init_node('movement_manager_node')
	
	cManager = ConnectionManager()
	mManager = MovementManager(cManager)
	

	ball = Ball()
	ball.position.position.x = -1.12
	ball.position.position.y = 0
	
	mManager.move_to_target(ball)
	

if __name__=='__main__':
	main()



