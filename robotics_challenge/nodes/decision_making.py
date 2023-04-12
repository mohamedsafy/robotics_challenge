#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
from robotics_challenge.msg import SnapshotBallsLocation, SnapshotBallsOrder


class Decision_maker:
	
	def __init__(self):
		
		#ball sequence
		self.balls=[]
		self.seq=[1,2,3]
		
		#Initialize node and subscriber
		rospy.init_node("desision_making_node")
		self.sub = rospy.Subscriber("balls/location", SnapshotBallsLocation, self.make_descisions)
		self.pub = rospy.Publisher("balls/order", Int32, queue_size=3)
	
	def make_descisions(self, snapshot):
		
		#set sequence
		#TODO: Implement Action server client here 
		rospy.sleep(1)
		print('1')
		self.pub.publish(1)
			
def main():
	maker= Decision_maker()
	print('HEYYYYYYYY IM HEEEEEREEEEEE I EXISTTTTTTTTT !!!!!!!!!! HEYYYYYYYYY')
	rospy.spin()

if __name__ == "__main__":
    main()
