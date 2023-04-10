#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose

class Decision_maker:
	
	def __init__(self):
		
		#ball sequence
		self.balls=[]
		self.seq=[]
		
		#Initialize node and subscriber
		rospy.init_node("desision_making_node")
		sub = rospy.Subscriber("balls_location", Pose, self.collect_balls)
	def collect_balls(self, ball):
		self.balls.append(ball)
		print('ball added')
	def make_descisions(self):
		
		#set sequence
		self.seq=[1,2,3]
		print(self.seq)


def main():
	maker= Decision_maker()
	maker.make_descisions()
	
	rospy.spin()

if __name__ == "__main__":
    main()
