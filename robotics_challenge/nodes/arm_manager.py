#!/usr/bin/env python3
import rospy
import moveit_commander
import moveit_msgs.msg
from std_srvs.srv import Empty

OPEN = 0.9
CLOSE = 0

class ArmController:
    
    def __init__(self):
        moveit_commander.roscpp_initialize([])
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.grip = moveit_commander.MoveGroupCommander("gripper")
        self.robot = moveit_commander.RobotCommander('robot_description')
        self.gripper = self.robot.get_joint('gripper')
    def handle_pickup(self, req):
        self.pick_ball([0.2, 0, 0.029])
    def handle_drop(self, req):
        self.place_ball([0.21, 0, 0.0275])
    def set_pos(self, position):
        self.arm.set_position_target(position)
        self.arm.go(wait=True)
    def set_pos_name(self, name):
        self.arm.set_named_target(name)
        self.arm.go(wait=True)
    def open_gripper(self):
        values = self.grip.get_current_joint_values()
        values[0] = 0.015
        self.grip.set_joint_value_target(values)
        self.grip.go(wait=True)
        #self.gripper.move(self.gripper.max_bound() * OPEN, True)

    def close_gripper(self):
        values = self.grip.get_current_joint_values()
        values[0] = 0.0
        self.grip.set_joint_value_target(values)
        self.grip.go(wait=True)
        #self.gripper.move(self.gripper.max_bound() * CLOSE, True)

    def pick_ball(self, position):
        position_above_ball = [position[0], position[1], position[2] + 0.1]
        self.set_pos(position_above_ball)
        self.open_gripper()
        position_below_ball = [position[0] - 0.005, position[1], position[2]]
        self.set_pos(position_below_ball)
        #position_below_ball = [position[0] - 0.01, position[1], position[2]]
        #self.set_pos(position_below_ball)
        self.close_gripper()
        self.set_pos_name('home')
    def place_ball(self, position):
        #self.set_pos(position)
        self.set_pos([0.25,0,0.03])
        self.open_gripper()
        self.set_pos_name('home')

def main():
    rospy.init_node('arm_manager')
    arm = ArmController()
    s1 = rospy.Service('ball_pickup', Empty, arm.handle_pickup)
    s2 = rospy.Service('ball_drop', Empty, arm.handle_drop)
    #arm.pick_ball([0.2, 0, 0.029])
    #arm.set_pos([0.2, 0, 0.029])
    #arm.close_gripper()
    #arm.open_gripper()
    rospy.spin()
if __name__ =="__main__":
    main()
