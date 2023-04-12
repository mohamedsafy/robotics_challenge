#!/usr/bin/env python3


#this is a test line
import rospy
from std_msgs.msg import String

def main():
	rospy.init_node("fake_camera")
	pub = rospy.Publisher("fake_camera", String, queue_size=100)
	
	rospy.sleep(0.5)
	pub.publish("Baby don't hurt me, don't hurt me, no more!")
	
if __name__ == "__main__":
	main()
