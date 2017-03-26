#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW1 for RBE 595/CS 525 Motion Planning
#code based on the simplemanipulation.py example
import time, numpy
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
    

    #Desired Positions	
    #table id=5
    #translation = (0.15839 -1.15869 0.7400)
    #quaternion  = (1.00000 0.00000 0.00000 0.00000)
    #table id=7
    #translation = (0.28435  0.73313 0.7400)
    #quaternion  = (1.00000 0.00000 0.00000 0.00000)

    table5 = [[1,0,0,0.15839],[0,1,0,-1.1869],[0,0,1,0.7400],[0,0,0,1]]
    table6 = [[1,0,0,0.28435],[0,1,0,0.73313],[0,0,1,0.7400],[0,0,0,1]]	

    with env:
	count = 0
    	for body in env.GetBodies():
		count = count + 1;
		if count == 6:
			
			body.SetTransform(numpy.array(table5))

		if count == 7:
			
			body.SetTransform(numpy.array(table6))


    
    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")

