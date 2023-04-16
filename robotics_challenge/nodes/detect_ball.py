#!/usr/bin/env python3

#Information about file
#This file implements detecting balls
#TODO: ADD MORE INFO ABOUT THIS FILE

#General remarks
#this is absolute stupidity, if python had variables pointers we wouldn't need to do this bullcrap. Note: I fucking hate python (1)

import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import Image
from robotics_challenge.msg import SnapshotBallsLocation

class BallsDetector:
	def __init__(self):

		#Variables
		self.poses = []
		self.bridge = CvBridge()
		self.image_received = False
		self.ball_is_taken=False

	def img_callback(self, stream):
		#TODO: Implement a function that is capable of taking bunch of images and finding balls in them, once the function finds a ball, it will be appended to the poses list
		
		#print('Image recieved!!!!')
		# Convert image to OpenCV format
		try:
			cv_image = self.bridge.imgmsg_to_cv2(stream, "bgr8")
		except CvBridgeError as e:
			print(e)

		self.image_received = True
		self.image = cv_image
		#self.show_image(cv_image)
			
		self.does_frame_have_ball(cv_image)


	def does_frame_have_ball(self, img):


		
		hsv_frame  =  cv2.cvtColor(img,  cv2.COLOR_BGR2HSV)

		hsv_frame  =  cv2.resize(hsv_frame,(640,300))
		img  =  cv2.resize(img,(640,300))
		
		circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,20,
			    param1=50,param2=30,minRadius=0,maxRadius=0)
			    
		circles = np.uint16(np.around(circles))
		
		for i in circles[0,:]:
			# draw the outer circle
			cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
			# draw the center of the circle
			cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

		'''
		low_H=0
		low_S=100
		low_V=100
		high_H=18
		high_S=255
		high_V=255


		#low_H=25
		#low_S=100
		#low_V=100
		#high_H=32
		#high_S=255
		#high_V=255


		mask_frame=cv2.inRange(hsv_frame,  (low_H,  low_S,  low_V),  (high_H,  high_S,  high_V))
		cv2.imshow("mask", mask_frame)
		contours,  hierarchy  =  cv2.findContours(mask_frame,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		#_,  contours,  _=  cv2.findContours(mask_frame,  cv2.RETR_EXTERNAL,  cv2.CHAIN_APPROX_SIMPLE)

		X,Y,W,H=0,0,0,0
		x,y,w,h=0,0,0,0

		for  pic,  contour  in  enumerate(contours):
			area =cv2.contourArea(contour)

			if(area>30):

				x,y,w,h=cv2.boundingRect(contour)
			if(w*h>W*H):
				X,Y,W,H= x,y,w,h

		img  =  cv2.rectangle(img,  (X,  Y),(X  +  W,  Y  +  H),(0,  0,  255),  2)

		cx  =  X+(W/2)
		cy  =  Y+(W/2)
		'''
		cv2.imshow("window",  img)
		cv2.waitKey(3)
		
	def show_image(self,img):
		cv2.imshow("Image Window", img)
		cv2.waitKey(3)
				
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
			
	
	rospy.sleep(1)
	rospy.spin()
def main():
	#Initialize node
	rospy.init_node('detect_ball_node')

	#Initalize objects
	detector = BallsDetector()
	cManager = ConnectionManager(detector.poses, detector.img_callback)

	rospy.spin()


	#Call startup_rountiune
	#startup_routiune(detector, cManager)
	
	#Detect rest of balls after initial balls are detected
	#watch_new_balls(detector, cManager)
	
	
if __name__ == "__main__":
	main()
	

	
	
