#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Point

class Balls_Detector:
 
 def __init__(self):
 
  #Variables
  self.poses = []
 
  #Initialize node and publisher
  rospy.init_node('detect_ball_node')
  self.pub = rospy.Publisher('balls_location', Pose, queue_size=3)
 
 def start_detection(self):
 #group 2 will be required to write the approperate code to detect balls location, may use lidar readings, camear, discover it!
 #it is now replaced by 3 predetermined balls
  self.poses =[Pose(position = Point(x=1, y=1, z=0)),
	       Pose(position = Point(x=1, y=1, z=0)),
	       Pose(position = Point(x=1, y=1, z=0))]
 


def main():
 print("hello ?")
 detector = Balls_Detector()
 detector.start_detection()
 for pose in detector.poses:
  print("publishing")
  rospy.sleep(1)
  detector.pub.publish(pose)
  

if __name__ == "__main__":
    main()
