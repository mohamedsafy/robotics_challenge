#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

class LidarObjectDetector:
    def __init__(self):
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

        self.prev_scan = None

    def lidar_callback(self, scan):
        if self.prev_scan is None:
            self.prev_scan = scan
            return

        for i in range(len(scan.ranges)):
            if scan.ranges[i] < 0.5 and self.prev_scan.ranges[i] > 0.5:
                print("Object detected at angle:", i*scan.angle_increment)

        self.prev_scan = scan

if __name__ == '__main__':
    rospy.init_node('lidar_object_detector')
    detector = LidarObjectDetector()
    rospy.spin()
