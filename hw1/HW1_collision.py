#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW1 for RBE 595/CS 525 Motion Planning
#code based on the simplemanipulation.py example
import time
import openravepy

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);



    #### YOUR CODE HERE ####
    robot.SetTransform([1, 0, 0, 0, 0, -1.4, 0.05])
    print 'Collision Check before :', env.CheckCollision(robot,env.GetBodies()[0])
    with env:
        #jointnames = ['l_shoulder_pan_joint','l_shoulder_lift_joint','l_upper_arm_roll_joint', 'l_elbow_flex_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
        jointnames1 = ['r_shoulder_pan_joint','r_shoulder_lift_joint','r_upper_arm_roll_joint', 'r_elbow_flex_joint','r_forearm_roll_joint','r_wrist_flex_joint','r_wrist_roll_joint']
        
        #robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        #robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996,1.00]);
        #robot.SetActiveDOFValues([1,1,1,1,1,1,1]);
        #robot.GetController().SetDesired(robot.GetDOFValues());

        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames1])
        robot.SetActiveDOFValues([0, 0, 0, -0.150000054, 0, -0.100000036, 0])
        robot.GetController().SetDesired(robot.GetDOFValues());
        print 'Collision Check after :', env.CheckCollision(robot,env.GetBodies()[0])


    waitrobot(robot)



    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")
