#!/usr/bin/env python

import rospy
from pyrobot import Robot

def init_arm_and_camera():
    
    bot = Robot("locobot")
    bot.camera.set_pan_tilt(0, 0.7, wait=True)
    bot.arm.go_home()
    bot.arm.set_joint_positions([1.96, 0.52, -0.51, 1.67, 0.01], plan=False)

if __name__=="__main__":
    
    init_arm_and_camera()
