#!/usr/bin/env python3

#Information about file
#This file implements detecting balls
#TODO: ADD MORE INFO ABOUT THIS FILE

#General remarks
#this is absolute stupidity, if python had variables pointers we wouldn't need to do this bullcrap. Note: I fucking hate python (1)

import rospy
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import Image
from robotics_challenge.msg import SnapshotBallsLocation

class BallsDetector:
	def __init__(self):

		#Variables
		self.poses = []

	def does_frame_have_ball(self, stream):
		#TODO: Implement a function that is capable of taking bunch of images and finding balls in them, once the function finds a ball, it will be appended to the poses list
		
		#print('Image recieved!!!!')
		
		
		
		
		self.poses =[Pose(position = Point(x=1, y=1, z=0)),
		Pose(position = Point(x=1, y=1, z=0)),
		Pose(position = Point(x=1, y=1, z=0))]
		
	def clear_balls(self):
		self.poses = []

class ConnectionManager:
	def __init__(self, balls_location, camera_callback):

		#Variables
		self.balls_location = balls_location

		#Connections
		self.pub = rospy.Publisher('balls/location', SnapshotBallsLocation, queue_size=3)
		#TODO: Not sure if that implementation of taking camera readings is sutable for this application, hence I left it marked unfinished
		self.sub_camera = rospy.Subscriber("camera/rgb/image_raw", Image, camera_callback)

	def send_snapshot(self):

		snapshot = SnapshotBallsLocation()
		snapshot.positions = self.balls_location
		rospy.sleep(1)
		self.pub.publish(snapshot)

def startup_routiune(detector, cManager):

	#TODO: Make robot spin 360 (requires connection with movement manager)		

	is_robot_finished_spinning = input("Did the node finish spinning?")

	cManager.balls_location = detector.poses #Refer to General remarks point 1
	
	#Send the initially found balls
	cManager.send_snapshot()
	detector.clear_balls()	
	cManager.balls_location = detector.poses #Refer to General remarks point 1
	
	print('Sending initial balls location is done, decision_making can carry on')

def watch_new_balls(detector, cManager):
	print('Looking for new balls')
	while True:
		while detector.poses :
			print('I found balls, sending to decision maker')
			cManager.balls_location = detector.poses #Refer to General remarks point 1
			
			cManager.send_snapshot()
			cManager.balls_location = detector.poses #Refer to General remarks point 1
			detector.poses =[]
	#Initalize objects
	detector = BallsDetector()
	cManager = ConnectionManager(detector.poses, detector.does_frame_have_ball)

	#Call startup_rountiune
			
	
	rospy.sleep(1)
def main():
	#Initialize node
	rospy.init_node('detect_ball_node')

	#Initalize objects
	detector = BallsDetector()
	cManager = ConnectionManager(detector.poses, detector.does_frame_have_ball)

	#Call startup_rountiune
	startup_routiune(detector, cManager)
	
	#Detect rest of balls after initial balls are detected
	#watch_new_balls(detector, cManager)
	
	
if __name__ == "__main__":
	main()
	

	
	
