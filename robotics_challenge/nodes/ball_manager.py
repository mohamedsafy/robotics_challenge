#!/usr/bin/env python3

import rospy
from robotics_challenge.msg import Ball, SnapshotBalls


class BallsManager():
    def __init__(self):

        self.balls = []
        self.ball_sub = rospy.Subscriber('/balls_location', Ball, self.callback)
    
    def callback(self, ball):
        ball.position3d.x = round(ball.position3d.x,1)
        ball.position3d.y = round(ball.position3d.y,1)

        found = False
        #Check if ball exist
        for ball_d in self.balls:
            if (ball_d.position3d.x == ball.position3d.x,1) and (ball_d.position3d.y == ball.position3d.y):
                found = True
        if not found:
            print(ball)
            print('added')
            self.balls.append(ball)
        
if __name__=='__main__':
    rospy.init_node('ball_manager_node')
    bManager = BallsManager()

    rospy.spin()
