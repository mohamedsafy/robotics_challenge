#!/usr/bin/env python3

import rospy
import os
from nav_msgs.msg import OccupancyGrid

rospy.init_node('test_map_server')
map = OccupancyGrid()


def callback(map):
    listed = []
    listed = list(map.data)
    
    #x = (index % map.info.width) * map.info.resolution + -10
    #y = (index / map.info.width) * map.info.resolution + -10
    #print(x,y)
    x1,y1 = -1,-1
    x2,y2 = -1,-5
    index1 = int((y1) / map.info.resolution * map.info.width + (x1) / map.info.resolution)
    index2 = int((y2) / map.info.resolution * map.info.width + (x2) / map.info.resolution)

    print('updating')

    listed[0] = 100


    map.data = tuple(listed)
    pub = rospy.Publisher('/modified_map', OccupancyGrid, queue_size=1)
    pub.publish(map)

sub = rospy.Subscriber('/map', OccupancyGrid, callback)

#map = rospy.wait_for_message('/map', OccupancyGrid)

rospy.spin()
